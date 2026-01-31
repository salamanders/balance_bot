import sys
import time
import logging
from enum import Enum, auto

from .config import (
    CONFIG_FILE,
    RobotConfig,
    PIDParams,
    temp_pid_overrides,
    SYSTEM_TIMING,
    TUNING_HEURISTICS,
)
from .robot_hardware import RobotHardware, IMUReading
from .wiring_check import WiringCheck
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
from .diagnostics import run_diagnostics

logger = logging.getLogger(__name__)


class RobotState(Enum):
    SETUP = auto()
    CALIBRATE = auto()
    TUNE = auto()
    BALANCE = auto()
    RECOVER = auto()
    EXIT = auto()


class RobotController:
    def __init__(self):
        setup_logging()

        self.force_tune = "--tune" in sys.argv
        self.has_saved_config = CONFIG_FILE.exists()
        force_calib = check_force_calibration_flag()

        # Check force calibration before loading config
        if force_calib:
            logger.info("Forcing calibration: Using default configuration.")
            self.config = RobotConfig(pid=PIDParams())
        else:
            self.config = RobotConfig.load()
            if self.has_saved_config:
                logger.info(">>> Config Found. Starting in PRODUCTION MODE.")
            else:
                logger.info(">>> No Config Found. Starting in FIRST RUN MODE.")

        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            gyro_axis=self.config.gyro_pitch_axis,
            gyro_invert=self.config.gyro_pitch_invert,
        )
        self.led = LedController(self.config.led)
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
        """Initialize the robot hardware."""
        self.hw.init()
        logger.info(">>> ROBOT ALIVE. Hold vertical for STEP 1.")

    def get_pitch(self, loop_delta_time: float) -> IMUReading:
        """
        Read IMU and update pitch angle.
        :param loop_delta_time: Time elapsed since last loop in seconds.
        :return: Current IMU reading.
        """
        reading = self.hw.read_imu_converted()
        self.pitch = self.filter.update(
            reading.pitch_angle, reading.pitch_rate, loop_delta_time
        )
        return reading

    def run(self) -> None:
        """Main application loop managing the robot state machine."""
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
            if self.config_dirty:
                self.config.save()
            self.hw.stop()
            self.led.signal_off()
            self.hw.cleanup()
            logger.info("Motors Stopped. Exiting.")

    def run_setup(self) -> RobotState:
        """
        Initial setup phase.
        Waits for a brief period to allow sensors to stabilize.
        """
        self.led.signal_setup()
        start_wait = time.monotonic()

        # Simple wait loop with filter warmup
        while time.monotonic() - start_wait < SYSTEM_TIMING.setup_wait:
            self.get_pitch(self.config.loop_time)
            self.led.update()
            time.sleep(self.config.loop_time)

        # Decide next state based on config availability
        if self.has_saved_config and not check_force_calibration_flag():
            if self.force_tune:
                return RobotState.TUNE
            return RobotState.BALANCE

        return RobotState.CALIBRATE

    def run_calibrate(self) -> RobotState:
        """
        Calibrate the vertical position.
        Sets the current pitch as the target angle (zero point).
        """
        logger.info("-> Calibrating Vertical...")
        self.led.signal_setup()

        # Update pitch once to get initial reading
        self.get_pitch(self.config.loop_time)
        self.config.pid.target_angle = self.pitch

        logger.info(f"-> Calibrated Vertical at: {self.config.pid.target_angle:.2f}")
        logger.info("-> Let it wobble gently (Step 2)...")

        # Pause
        pause_start = time.monotonic()
        while time.monotonic() - pause_start < SYSTEM_TIMING.calibration_pause:
            self.led.update()
            time.sleep(self.config.loop_time)

        if self.force_tune:
            return RobotState.TUNE

        # Default to BALANCE using conservative defaults if not forcing tune
        return RobotState.BALANCE

    def run_tune(self) -> RobotState:
        """
        Auto-tuning phase using Ziegler-Nichols inspired heuristic.
        Increases Kp until oscillation, then backs off and sets Ki/Kd.
        """
        logger.info("-> Auto-Tuning...")
        self.led.signal_tuning()

        rate = RateLimiter(1.0 / self.config.loop_time)

        # Ensure we start fresh
        self.pid.reset()

        found_tune = False

        # Temporarily zero I and D for tuning loop using context manager
        with temp_pid_overrides(self.config.pid, ki=0.0, kd=0.0):
            while self.running:
                self.get_pitch(self.config.loop_time)
                self.led.update()

                error = self.pitch - self.config.pid.target_angle

                # Tune Logic: Increment Kp until vibration
                self.config.pid.kp += TUNING_HEURISTICS.kp_increment

                # Vibration detection
                if (error > 0 and self.pid.last_error < 0) or (
                    error < 0 and self.pid.last_error > 0
                ):
                    self.vibration_counter += 1

                if self.vibration_counter > self.config.vibration_threshold:
                    found_tune = True
                    break

                output = self.pid.update(error, self.config.loop_time)
                # Simple P loop for tuning, no turn correction
                self.hw.set_motors(output, output)

                rate.sleep()

        if found_tune:
            # Re-apply calculations based on the Kp we reached
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
        Main balancing phase.
        Executes PID control loop, fall detection, and battery compensation.
        """
        logger.info("-> Balancing...")
        self.led.signal_ready()

        # Reset control state
        self.pid.reset()
        self.last_pitch_rate = 0.0

        rate = RateLimiter(1.0 / self.config.loop_time)

        while self.running:
            reading = self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.pitch - self.config.pid.target_angle

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
        :param reading: Current IMU reading.
        :param error: Current pitch error.
        :param loop_delta_time: Time elapsed since last loop.
        """
        # PID Update
        output = self.pid.update(
            error, loop_delta_time, measurement_rate=reading.pitch_rate
        )

        # Turn Correction
        turn_correction = -reading.yaw_rate * self.config.control.yaw_correction_factor

        # Continuous Tuning
        adj = self.tuner.update(error)
        if adj.kp != 0 or adj.ki != 0 or adj.kd != 0:
            self.config.pid.kp = max(0.1, self.config.pid.kp + adj.kp)
            self.config.pid.ki = max(0.0, self.config.pid.ki + adj.ki)
            self.config.pid.kd = max(0.0, self.config.pid.kd + adj.kd)
            self.config_dirty = True
            logger.info(
                f"-> Tuned: P={self.config.pid.kp:.2f} I={self.config.pid.ki:.3f} D={self.config.pid.kd:.2f}"
            )

        # Battery Estimation
        ang_accel = (reading.pitch_rate - self.last_pitch_rate) / loop_delta_time
        self.last_pitch_rate = reading.pitch_rate

        comp_factor = self.battery.update(output, ang_accel, loop_delta_time)

        # Log low battery occasionally
        if (
            comp_factor < self.config.control.low_battery_log_threshold
            and self.battery_logger.should_log()
        ):
            logger.warning(f"-> Low Battery? Compensating: {int(comp_factor * 100)}%")

        # Drive
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
        Recovery phase after a fall.
        Waits for the robot to be placed upright again.
        """
        self.led.signal_off()
        self.hw.stop()

        rate = RateLimiter(1.0 / self.config.loop_time)

        while self.running:
            self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.pitch - self.config.pid.target_angle
            if abs(error) < self.config.control.upright_threshold:
                logger.info(f"-> Upright detected! (Pitch: {self.pitch:.2f})")
                self.led.countdown()
                return RobotState.BALANCE

            rate.sleep()

        return RobotState.EXIT


def main() -> None:
    """Entry point for the robot control application."""
    if "--diagnose" in sys.argv:
        run_diagnostics()
        return

    if "--check-wiring" in sys.argv:
        try:
            WiringCheck().run()
        except KeyboardInterrupt:
            pass
        return

    bot = RobotController()
    bot.init()
    bot.run()


if __name__ == "__main__":
    main()
