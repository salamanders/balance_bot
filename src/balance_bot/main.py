import sys
import time
import logging
from pathlib import Path

from .config import RobotConfig, PIDParams
from .robot_hardware import RobotHardware
from .pid import PIDController
from .leds import LedController
from .tuner import ContinuousTuner
from .battery import BatteryEstimator
from .utils import RateLimiter, LogThrottler, setup_logging

# Constants that are not in config (system behavior)
FORCE_CALIB_FILE = Path("force_calibration.txt")
SETUP_WAIT_SEC = 2.0
CALIBRATION_PAUSE_SEC = 1.0
SAVE_INTERVAL_SEC = 30.0

logger = logging.getLogger(__name__)


class RobotController:
    def __init__(self):
        setup_logging()

        # Check force calibration before loading config
        if self._check_force_calibration():
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

        self.running = True
        self.pitch = 0.0
        self.last_save_time = time.monotonic()
        self.config_dirty = False

        # State for tuning
        self.vibration_counter = 0

    def _check_force_calibration(self) -> bool:
        if FORCE_CALIB_FILE.exists():
            logger.info(f"Force calibration file found: {FORCE_CALIB_FILE}")
            return True
        if "--force-calibration" in sys.argv:
            logger.info("Force calibration flag found")
            return True
        return False

    def init(self) -> None:
        self.hw.init()
        logger.info(">>> ROBOT ALIVE. Hold vertical for STEP 1.")

    def get_pitch(self, dt: float) -> tuple[float, float, float]:
        acc_angle, gyro_rate, yaw_rate = self.hw.read_imu_processed()

        # Complementary Filter
        alpha = self.config.complementary_alpha
        self.pitch = (alpha * (self.pitch + gyro_rate * dt)) + (
            (1.0 - alpha) * acc_angle
        )

        return self.pitch, gyro_rate, yaw_rate

    def run(self) -> None:
        # Initial Setup Phase
        self.led.signal_setup()
        start_wait = time.monotonic()

        # Simple wait loop
        while time.monotonic() - start_wait < SETUP_WAIT_SEC:
            self.led.update()
            time.sleep(self.config.loop_time)

        try:
            # Mode dispatch
            self.run_calibrate()
            self.run_tune()

            # Main Balance Loop
            self.run_balance()

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

    def run_calibrate(self) -> None:
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

    def run_tune(self) -> None:
        logger.info("-> Auto-Tuning...")
        self.led.signal_tuning()

        rate = RateLimiter(1.0 / self.config.loop_time)

        while self.running:
            pitch, _, _ = self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - pitch

            # Tune Logic: Increment Kp until vibration
            self.config.pid.kp += 0.05

            # Vibration detection
            if (error > 0 and self.pid.last_error < 0) or (
                error < 0 and self.pid.last_error > 0
            ):
                self.vibration_counter += 1

            if self.vibration_counter > self.config.vibration_threshold:
                self.config.pid.kp *= 0.6
                self.config.pid.kd = self.config.pid.kp * 0.05
                self.config.pid.ki = self.config.pid.kp * 0.005
                logger.info(
                    f"-> Tuned! Kp={self.config.pid.kp:.2f} Kd={self.config.pid.kd:.2f}"
                )
                self.config.save()
                self.vibration_counter = 0
                return  # Exit tuning, go to balance

            output = self.config.pid.kp * error
            # Simple P loop for tuning, no turn correction
            self.hw.set_motors(output, output)
            self.pid.last_error = error

            rate.sleep()

    def run_balance(self) -> None:
        logger.info("-> Balancing...")
        self.led.signal_ready()

        last_pitch_rate = 0.0
        rate = RateLimiter(1.0 / self.config.loop_time)

        while self.running:
            pitch, pitch_rate, yaw_rate = self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - pitch

            # Fall detection
            if abs(error) > self.config.fall_angle_limit:
                logger.warning("!!! FELL OVER !!!")
                self.hw.stop()
                if self.config_dirty:
                    self.config.save()
                    self.config_dirty = False

                self.run_recover()
                # Restore state after recover
                self.led.signal_ready()
                self.pid.reset()
                rate.reset() # Reset timer to avoid catch-up
                continue

            # PID Update
            output = self.pid.update(error, self.config.loop_time)

            # Turn Correction
            turn_correction = -yaw_rate * 0.5

            # Continuous Tuning
            kp_n, ki_n, kd_n = self.tuner.update(error)
            if kp_n != 0 or ki_n != 0 or kd_n != 0:
                self.config.pid.kp = max(0.1, self.config.pid.kp + kp_n)
                self.config.pid.ki = max(0.0, self.config.pid.ki + ki_n)
                self.config.pid.kd = max(0.0, self.config.pid.kd + kd_n)
                self.config_dirty = True
                logger.info(
                    f"-> Tuned: P={self.config.pid.kp:.2f} I={self.config.pid.ki:.3f} D={self.config.pid.kd:.2f}"
                )

            # Battery Estimation
            ang_accel = (pitch_rate - last_pitch_rate) / self.config.loop_time
            last_pitch_rate = pitch_rate

            comp_factor = self.battery.update(output, ang_accel, self.config.loop_time)
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

            rate.sleep()

    def run_recover(self) -> None:
        self.led.signal_off()
        self.hw.stop()

        rate = RateLimiter(1.0 / self.config.loop_time)

        while self.running:
            pitch, _, _ = self.get_pitch(self.config.loop_time)
            self.led.update()

            error = self.config.pid.target_angle - pitch
            if abs(error) < 5.0:
                logger.info(f"-> Upright detected! (Pitch: {pitch:.2f})")
                self.led.countdown()
                return  # Back to balance

            rate.sleep()


def main() -> None:
    bot = RobotController()
    bot.init()
    bot.run()


if __name__ == "__main__":
    main()
