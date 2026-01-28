import time
import logging
from enum import Enum, auto

from .config import (
    RobotConfig,
    PIDParams,
    temp_pid_overrides,
    SYSTEM_TIMING,
    TUNING_HEURISTICS,
)
from .robot_hardware import RobotHardware, IMUReading
from .pid import PIDController
from .leds import LedController
from .tuner import ContinuousTuner
from .battery import BatteryEstimator
from .utils import (
    RateLimiter,
    LogThrottler,
    setup_logging,
    ComplementaryFilter,
    check_force_calibration_flag,
)

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """
    State machine enumeration for the robot's lifecycle.
    """

    SETUP = auto()  # Hardware initialization and waiting
    CALIBRATE = auto()  # Determining the vertical "zero" point
    TUNE = auto()  # Initial "Rough" Auto-Tuning (Ziegler-Nichols)
    BALANCE = auto()  # Main balancing loop
    RECOVER = auto()  # Wait for human assistance after falling
    EXIT = auto()  # Clean shutdown


class RobotController:
    """
    Main Controller Class.
    Orchestrates the robot's state machine, sensor updates, control loops,
    and hardware interactions.
    """

    def __init__(self):
        """Initialize the Controller and all sub-systems."""
        setup_logging()

        # Check force calibration before loading config
        if check_force_calibration_flag():
            logger.info("Forcing calibration: Using default configuration.")
            self.config = RobotConfig(pid=PIDParams())
        else:
            self.config = RobotConfig.load()

        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            gyro_axis=self.config.gyro_pitch_axis,
            gyro_invert=self.config.gyro_pitch_invert,
        )
        self.led = LedController()
        self.pid = PIDController(self.config.pid)
        self.tuner = ContinuousTuner(self.config.tuner)
        self.battery = BatteryEstimator(self.config.battery)
        self.battery_logger = LogThrottler(SYSTEM_TIMING.battery_log_interval)
        self.filter = ComplementaryFilter(self.config.complementary_alpha)

        self.running = True
        self.pitch = 0.0
        self.last_save_time = time.monotonic()
        self.config_dirty = False

        # State for tuning
        self.vibration_counter = 0

        # Loop state
        self.last_pitch_rate = 0.0

    def init(self) -> None:
        """Initialize the robot hardware and signal readiness."""
        self.hw.init()
        logger.info(">>> ROBOT ALIVE. Hold vertical for STEP 1.")

    def get_pitch(self, loop_delta_time: float) -> IMUReading:
        """
        Read IMU and update pitch angle using Sensor Fusion.
        :param loop_delta_time: Time elapsed since last loop in seconds.
        :return: Current IMU reading (angle, rates).
        """
        reading = self.hw.read_imu_converted()
        # Update the Complementary Filter state
        self.pitch = self.filter.update(
            reading.pitch_angle, reading.pitch_rate, loop_delta_time
        )
        return reading

    def run(self) -> None:
        """
        Main application loop managing the robot state machine.
        This loops indefinitely until EXIT state is reached.
        """
        state = RobotState.SETUP

        try:
            while self.running:
                match state:
                    case RobotState.SETUP:
                        state = self.run_setup()
                    case RobotState.CALIBRATE:
                        state = self.run_calibrate()
                    case RobotState.TUNE:
                        state = self.run_tune()
                    case RobotState.BALANCE:
                        state = self.run_balance()
                    case RobotState.RECOVER:
                        state = self.run_recover()
                    case RobotState.EXIT:
                        break

        except KeyboardInterrupt:
            logger.info("Keyboard Interrupt detected.")
        except Exception as e:
            logger.exception(f"Unexpected error: {e}")
        finally:
            # Ensure safe shutdown
            if self.config_dirty:
                self.config.save()
            self.hw.stop()
            self.led.signal_off()
            self.hw.cleanup()
            logger.info("Motors Stopped. Exiting.")

    def run_setup(self) -> RobotState:
        """
        State: SETUP
        Waits for a brief period to allow sensors to stabilize and user to position robot.
        """
        self.led.signal_setup()
        start_wait = time.monotonic()

        # Simple wait loop
        while time.monotonic() - start_wait < SYSTEM_TIMING.setup_wait:
            self.led.update()
            time.sleep(self.config.loop_time)

        return RobotState.CALIBRATE

    def run_calibrate(self) -> RobotState:
        """
        State: CALIBRATE
        Captures the current angle as the "Zero" point (Target Angle).
        User should be holding the robot vertical during this phase.
        """
        logger.info("-> Calibrating Vertical...")
        self.led.signal_setup()

        # Update pitch once to get initial reading
        self.get_pitch(self.config.loop_time)
        self.config.pid.target_angle = self.pitch

        logger.info(f"-> Calibrated Vertical at: {self.config.pid.target_angle:.2f}")
        logger.info("-> Let it wobble gently (Step 2)...")

        # Pause to let user release the robot or prepare for tuning
        pause_start = time.monotonic()
        while time.monotonic() - pause_start < SYSTEM_TIMING.calibration_pause:
            self.led.update()
            time.sleep(self.config.loop_time)

        return RobotState.TUNE

    def run_tune(self) -> RobotState:
        """
        State: TUNE
        Performs an initial Auto-Tuning procedure if calibration was fresh.
        Uses a Ziegler-Nichols inspired heuristic:
        1. Increase Kp (Proportional Gain) until oscillation is detected.
        2. Back off Kp significantly.
        3. Set Ki and Kd based on the Kp value that caused oscillation.
        """
        logger.info("-> Auto-Tuning...")
        self.led.signal_tuning()

        rate = RateLimiter(1.0 / self.config.loop_time)

        # Ensure we start fresh
        self.pid.reset()

        found_tune = False

        # Temporarily zero I and D for tuning loop using context manager.
        # We only want to test P-stability here.
        with temp_pid_overrides(self.config.pid, ki=0.0, kd=0.0):
            while self.running:
                self.get_pitch(self.config.loop_time)
                self.led.update()

                error = self.config.pid.target_angle - self.pitch

                # Tune Logic: Increment Kp until vibration
                self.config.pid.kp += TUNING_HEURISTICS.kp_increment

                # Vibration detection (Zero-Crossing Counter)
                if (error > 0 and self.pid.last_error < 0) or (
                    error < 0 and self.pid.last_error > 0
                ):
                    self.vibration_counter += 1

                # If we cross zero enough times, we are oscillating
                if self.vibration_counter > self.config.vibration_threshold:
                    found_tune = True
                    break

                output = self.pid.update(error, self.config.loop_time)
                # Simple P loop for tuning, no turn correction
                self.hw.set_motors(output, output)

                rate.sleep()

        if found_tune:
            # Re-apply calculations based on the Critical Kp we reached
            self.config.pid.kp *= TUNING_HEURISTICS.kp_reduction
            self.config.pid.kd = self.config.pid.kp * TUNING_HEURISTICS.kd_ratio
            self.config.pid.ki = self.config.pid.kp * TUNING_HEURISTICS.ki_ratio
            logger.info(
                f"-> Tuned! Kp={self.config.pid.kp:.2f} Kd={self.config.pid.kd:.2f}"
            )
            self.config.save()
            self.vibration_counter = 0
            return RobotState.BALANCE

        return RobotState.EXIT

    def run_balance(self) -> RobotState:
        """
        State: BALANCE
        The main control loop.
        - Reads Sensors
        - Calculates PID Output
        - Runs Continuous Tuning (Fine-tuning)
        - Estimates Battery Level
        - Drives Motors
        - Checks for Falls
        """
        logger.info("-> Balancing...")
        self.led.signal_ready()

        # Reset control state to prevent jump
        self.pid.reset()
        self.last_pitch_rate = 0.0

        rate = RateLimiter(1.0 / self.config.loop_time)

        while self.running:
            reading = self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - self.pitch

            # Fall detection
            if abs(error) > self.config.fall_angle_limit:
                logger.warning("!!! FELL OVER !!!")
                self.hw.stop()
                if self.config_dirty:
                    self.config.save()
                    self.config_dirty = False
                return RobotState.RECOVER

            # Execute Control Step
            self._step_balance(reading, error, self.config.loop_time)

            rate.sleep()

        return RobotState.EXIT

    def _step_balance(
        self, reading: IMUReading, error: float, loop_delta_time: float
    ) -> None:
        """
        Execute one step of the balancing control loop.

        Sequence:
        1. PID Control -> Base Output
        2. Turn Correction -> Yaw Adjustment
        3. Continuous Tuning -> Adjust PID gains if needed
        4. Battery Compensation -> Scale Output based on voltage drop
        5. Drive Motors -> Final Command
        """
        # 1. PID Update
        output = self.pid.update(
            error, loop_delta_time, measurement_rate=reading.pitch_rate
        )

        # 2. Turn Correction (Simple P-controller on Yaw Rate)
        turn_correction = -reading.yaw_rate * 0.5

        # 3. Continuous Tuning
        adj = self.tuner.update(error)
        if adj.kp != 0 or adj.ki != 0 or adj.kd != 0:
            self.config.pid.kp = max(0.1, self.config.pid.kp + adj.kp)
            self.config.pid.ki = max(0.0, self.config.pid.ki + adj.ki)
            self.config.pid.kd = max(0.0, self.config.pid.kd + adj.kd)
            self.config_dirty = True
            logger.info(
                f"-> Tuned: P={self.config.pid.kp:.2f} I={self.config.pid.ki:.3f} D={self.config.pid.kd:.2f}"
            )

        # 4. Battery Estimation
        # Calculate Angular Acceleration (for responsiveness estimation)
        ang_accel = (reading.pitch_rate - self.last_pitch_rate) / loop_delta_time
        self.last_pitch_rate = reading.pitch_rate

        comp_factor = self.battery.update(output, ang_accel, loop_delta_time)

        # Log low battery occasionally
        if comp_factor < 0.95 and self.battery_logger.should_log():
            logger.warning(f"-> Low Battery? Compensating: {int(comp_factor * 100)}%")

        # 5. Drive Motors
        # Boost output by dividing by the factor (e.g. Output 50 / Factor 0.8 = Command 62)
        final_drive = output / comp_factor
        self.hw.set_motors(final_drive + turn_correction, final_drive - turn_correction)

        # Periodic Save
        if self.config_dirty and (
            time.monotonic() - self.last_save_time > SYSTEM_TIMING.save_interval
        ):
            self.config.save()
            self.last_save_time = time.monotonic()
            self.config_dirty = False

    def run_recover(self) -> RobotState:
        """
        State: RECOVER
        Wait for the robot to be placed upright again by the user.
        Once upright, start a countdown and resume balancing.
        """
        self.led.signal_off()
        self.hw.stop()

        rate = RateLimiter(1.0 / self.config.loop_time)

        while self.running:
            self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - self.pitch
            # Check if we are roughly upright again
            if abs(error) < 5.0:
                logger.info(f"-> Upright detected! (Pitch: {self.pitch:.2f})")
                self.led.countdown()
                return RobotState.BALANCE

            rate.sleep()

        return RobotState.EXIT


def main() -> None:
    """Entry point for the robot control application."""
    bot = RobotController()
    bot.init()
    bot.run()


if __name__ == "__main__":
    main()
