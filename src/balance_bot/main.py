import sys
import time
import logging
from enum import Enum, auto
from pathlib import Path

from .config import RobotConfig, PIDParams, temp_pid_overrides
from .robot_hardware import RobotHardware, IMUReading
from .pid import PIDController
from .leds import LedController
from .tuner import ContinuousTuner
from .battery import BatteryEstimator
from .utils import RateLimiter, LogThrottler, setup_logging, ComplementaryFilter, check_force_calibration_flag

# Constants that are not in config (system behavior)
SETUP_WAIT_SEC = 2.0
CALIBRATION_PAUSE_SEC = 1.0
SAVE_INTERVAL_SEC = 30.0

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
        self.tuner = ContinuousTuner()
        self.battery = BatteryEstimator()
        self.battery_logger = LogThrottler(5.0)  # Log every 5 seconds
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
        self.hw.init()
        logger.info(">>> ROBOT ALIVE. Hold vertical for STEP 1.")

    def get_pitch(self, dt: float) -> IMUReading:
        reading = self.hw.read_imu_converted()
        self.pitch = self.filter.update(reading.pitch_angle, reading.pitch_rate, dt)
        return reading

    def run(self) -> None:
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
        # Initial Setup Phase
        self.led.signal_setup()
        start_wait = time.monotonic()

        # Simple wait loop
        while time.monotonic() - start_wait < SETUP_WAIT_SEC:
            self.led.update()
            time.sleep(self.config.loop_time)

        return RobotState.CALIBRATE

    def run_calibrate(self) -> RobotState:
        logger.info("-> Calibrating Vertical...")
        self.led.signal_setup()

        # Update pitch once to get initial reading
        self.get_pitch(self.config.loop_time)
        self.config.pid.target_angle = self.pitch

        logger.info(f"-> Calibrated Vertical at: {self.config.pid.target_angle:.2f}")
        logger.info("-> Let it wobble gently (Step 2)...")

        # Pause
        pause_start = time.monotonic()
        while time.monotonic() - pause_start < CALIBRATION_PAUSE_SEC:
            self.led.update()
            time.sleep(self.config.loop_time)

        return RobotState.TUNE

    def run_tune(self) -> RobotState:
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

                error = self.config.pid.target_angle - self.pitch

                # Tune Logic: Increment Kp until vibration
                self.config.pid.kp += 0.05

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
            self.config.pid.kp *= 0.6
            self.config.pid.kd = self.config.pid.kp * 0.05
            self.config.pid.ki = self.config.pid.kp * 0.005
            logger.info(
                f"-> Tuned! Kp={self.config.pid.kp:.2f} Kd={self.config.pid.kd:.2f}"
            )
            self.config.save()
            self.vibration_counter = 0
            return RobotState.BALANCE

        return RobotState.EXIT

    def run_balance(self) -> RobotState:
        logger.info("-> Balancing...")
        self.led.signal_ready()

        # Reset control state
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

    def _step_balance(self, reading: IMUReading, error: float, dt: float) -> None:
        """
        Execute one step of the balancing control loop.
        """
        # PID Update
        output = self.pid.update(error, dt, measurement_rate=reading.pitch_rate)

        # Turn Correction
        turn_correction = -reading.yaw_rate * 0.5

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
        ang_accel = (reading.pitch_rate - self.last_pitch_rate) / dt
        self.last_pitch_rate = reading.pitch_rate

        comp_factor = self.battery.update(output, ang_accel, dt)

        # Log low battery occasionally
        if comp_factor < 0.95 and self.battery_logger.should_log():
            logger.warning(f"-> Low Battery? Compensating: {int(comp_factor * 100)}%")

        # Drive
        final_drive = output / comp_factor
        self.hw.set_motors(
            final_drive + turn_correction, final_drive - turn_correction
        )

        # Periodic Save
        if self.config_dirty and (time.monotonic() - self.last_save_time > SAVE_INTERVAL_SEC):
            self.config.save()
            self.last_save_time = time.monotonic()
            self.config_dirty = False

    def run_recover(self) -> RobotState:
        self.led.signal_off()
        self.hw.stop()

        rate = RateLimiter(1.0 / self.config.loop_time)

        while self.running:
            self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - self.pitch
            if abs(error) < 5.0:
                logger.info(f"-> Upright detected! (Pitch: {self.pitch:.2f})")
                self.led.countdown()
                return RobotState.BALANCE

            rate.sleep()

        return RobotState.EXIT


def main() -> None:
    bot = RobotController()
    bot.init()
    bot.run()


if __name__ == "__main__":
    main()
