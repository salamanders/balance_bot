import sys
import time
import logging

from .config import (
    CONFIG_FILE,
    RobotConfig,
    PIDParams,
    SYSTEM_TIMING,
    STARTUP_RAMP_SPEED,
    CRASH_ANGLE,
    BALANCING_THRESHOLD,
)
from .robot_hardware import RobotHardware, IMUReading
from .wiring_check import WiringCheck
from .pid import PIDController
from .leds import LedController
from .tuner import ContinuousTuner, BalancePointFinder
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


class RobotController:
    """
    Main Controller Class.
    Orchestrates hardware, state transitions, and the main control loop.
    """

    def __init__(self):
        setup_logging()

        self.force_tune = "--tune" in sys.argv
        self.has_saved_config = CONFIG_FILE.exists()
        force_calib = check_force_calibration_flag()

        # Check force calibration before loading config
        if force_calib:
            logger.info("Forcing calibration: Using default configuration.")
            self.config = RobotConfig(pid=PIDParams())
            self.first_run = True
        else:
            self.config = RobotConfig.load()
            if self.has_saved_config:
                logger.info(">>> Config Found. Starting in PRODUCTION MODE.")
                self.first_run = False
            else:
                logger.info(">>> No Config Found. Starting in FIRST RUN MODE.")
                self.first_run = True

        if self.first_run:
            # Zero out PID for learning phase
            self.config.pid.kp = 0.0
            self.config.pid.ki = 0.0
            self.config.pid.kd = 0.0

        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            gyro_axis=self.config.gyro_pitch_axis,
            gyro_invert=self.config.gyro_pitch_invert,
            accel_vertical_axis=self.config.accel_vertical_axis,
            accel_vertical_invert=self.config.accel_vertical_invert,
            accel_forward_axis=self.config.accel_forward_axis,
            accel_forward_invert=self.config.accel_forward_invert,
            i2c_bus=self.config.i2c_bus,
        )
        self.led = LedController(self.config.led)
        self.pid = PIDController(self.config.pid)
        self.tuner = ContinuousTuner(self.config.tuner)
        self.balance_finder = BalancePointFinder(self.config.tuner)
        self.battery = BatteryEstimator(self.config.battery)
        self.battery_logger = LogThrottler(SYSTEM_TIMING.battery_log_interval)
        self.filter = ComplementaryFilter(self.config.complementary_alpha)

        self.running = True
        self.pitch = 0.0
        self.last_save_time = time.monotonic()
        self.config_dirty = False

        # Loop state
        self.last_pitch_rate = 0.0

    def init(self) -> None:
        """Initialize the robot hardware and print startup message."""
        self.hw.init()
        logger.info(">>> ROBOT ALIVE. Hold vertical for STEP 1.")

    def get_pitch(self, loop_delta_time: float) -> IMUReading:
        """
        Read IMU and update the Complementary Filter state.

        :param loop_delta_time: Time elapsed since last loop in seconds.
        :return: Current IMU reading with calculated pitch angle and rates.
        """
        reading = self.hw.read_imu_converted()
        self.pitch = self.filter.update(
            reading.pitch_angle, reading.pitch_rate, loop_delta_time
        )
        return reading

    def run(self) -> None:
        """
        Unified Main Loop.
        Handles Setup, Calibration, Balancing, Recovery, and Tuning.
        """
        # 1. Warmup
        logger.info("-> Warming up sensors...")
        self.led.signal_setup()
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < SYSTEM_TIMING.setup_wait:
            self.get_pitch(self.config.loop_time)
            self.led.update()
            time.sleep(self.config.loop_time)

        # 2. Calibration (First Run or Forced)
        if self.first_run or check_force_calibration_flag():
            self.run_auto_calibrate()

        # 3. Initialize Loop
        tuning_aggression = 5.0 if self.first_run else 1.0
        if self.force_tune:
            tuning_aggression = 5.0

        logger.info(f"-> Starting Control Loop. Aggression: {tuning_aggression:.2f}")
        self.led.signal_ready()

        rate = RateLimiter(1.0 / self.config.loop_time)
        self.pid.reset()
        was_crashed = True  # Assume crashed/resting at start to trigger checks

        try:
            while self.running:
                # Read Sensors
                reading = self.get_pitch(self.config.loop_time)
                self.led.update()

                # --- CRASH SAFETY ---
                if abs(self.pitch) > CRASH_ANGLE:
                    if not was_crashed:
                        logger.warning("!!! CRASH DETECTED !!!")
                    self.hw.stop()
                    was_crashed = True
                    rate.sleep()
                    continue

                # --- RECOVERY / SOFT START ---
                if was_crashed:
                    # If Kp is low (learning), we skip soft start and let the tuner ramp it up.
                    # If Kp is normal, and we are leaning > 5 deg, we use soft start.
                    if self.config.pid.kp >= 1.0 and abs(self.pitch) > 5.0:
                        logger.info(
                            "-> Restored from crash. Attempting Soft Recovery..."
                        )
                        if not self.recover_from_rest():
                            was_crashed = True
                            continue  # Recovery failed/aborted

                    was_crashed = False
                    self.pid.reset()
                    logger.info("-> Active Balancing Started.")

                # --- CONTROL STEP ---
                error = self.pitch - self.config.pid.target_angle

                self._step_balance(
                    reading, error, self.config.loop_time, tuning_aggression
                )

                # Decay Aggression
                if tuning_aggression > 0.1:
                    tuning_aggression *= 0.9995

                rate.sleep()

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

    def run_auto_calibrate(self) -> None:
        """
        Interactive calibration sequence to find the center of balance.
        Steps:
        1. User places robot on front training wheels.
        2. User places robot on back training wheels.
        3. Center is calculated and saved.
        """
        self.led.signal_setup()
        logger.info(">>> AUTO CALIBRATION SEQUENCE <<<")
        print("\n" + "=" * 40)
        print("STEP 1: Place robot resting on FRONT training wheels (leaning forward).")
        print("Ensure it is stable.")
        input("Press ENTER when ready...")

        # Wait for filter to settle
        logger.info("Measuring...")
        min_angle = self._wait_for_stable_pitch()
        logger.info(f"-> Measured Front Angle: {min_angle:.2f}")

        print("\nSTEP 2: Place robot resting on BACK training wheels (leaning backward).")
        print("Ensure it is stable.")
        input("Press ENTER when ready...")

        # Wait for filter to settle
        logger.info("Measuring...")
        max_angle = self._wait_for_stable_pitch()
        logger.info(f"-> Measured Back Angle: {max_angle:.2f}")

        target = (min_angle + max_angle) / 2
        self.config.pid.target_angle = target
        self.config.save()

        logger.info(f"-> Calibration Complete. Target Angle: {target:.2f}")
        print("=" * 40 + "\n")

    def _wait_for_stable_pitch(self, duration: float = 2.0) -> float:
        """Run the filter loop for a duration to let the pitch settle."""
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            self.get_pitch(self.config.loop_time)
            time.sleep(self.config.loop_time)
        return self.pitch

    def recover_from_rest(self) -> bool:
        """
        Attempt to gently stand up from a resting position.
        Uses a moving setpoint to lift the robot.
        """
        logger.info("-> Attempting Soft Recovery...")

        # Update pitch to get current angle
        self.get_pitch(self.config.loop_time)
        start_pitch = self.pitch

        # If we are already upright enough, return True
        if abs(start_pitch) < 5.0:
            return True

        current_setpoint = start_pitch
        rate = RateLimiter(1.0 / self.config.loop_time)

        # Use a context manager to temporarily override target_angle
        # We manually update target_angle inside the loop, but this ensures restore
        original_target = self.config.pid.target_angle

        try:
            while self.running:
                reading = self.get_pitch(self.config.loop_time)
                self.led.update()

                # Move setpoint towards 0
                if current_setpoint > 0:
                    current_setpoint -= STARTUP_RAMP_SPEED
                    if current_setpoint < 0:
                        current_setpoint = 0
                else:
                    current_setpoint += STARTUP_RAMP_SPEED
                    if current_setpoint > 0:
                        current_setpoint = 0

                self.config.pid.target_angle = current_setpoint
                error = self.pitch - self.config.pid.target_angle

                # Safety Check
                if abs(self.pitch) > CRASH_ANGLE:
                    logger.error("-> Crash during recovery!")
                    self.hw.stop()
                    return False

                # PID Update
                output = self.pid.update(
                    error, self.config.loop_time, measurement_rate=reading.pitch_rate
                )
                self.hw.set_motors(output, output)

                # Exit condition: Setpoint reached 0 AND Pitch is near 0
                if abs(current_setpoint) < 0.1 and abs(self.pitch) < 5.0:
                    logger.info("-> Recovery Complete.")
                    return True

                rate.sleep()

        except Exception as e:
            logger.error(f"Recovery Error: {e}")
        finally:
            self.config.pid.target_angle = original_target
            self.hw.stop()

        return False

    def _step_balance(
        self,
        reading: IMUReading,
        error: float,
        loop_delta_time: float,
        tuning_scale: float = 1.0,
    ) -> None:
        """
        Execute one step of the balancing control loop.
        """
        # PID Update
        output = self.pid.update(
            error, loop_delta_time, measurement_rate=reading.pitch_rate
        )

        # Turn Correction
        turn_correction = (
            -reading.yaw_rate * self.config.control.yaw_correction_factor
        )

        # Continuous Tuning
        adj = self.tuner.update(error, scale=tuning_scale)
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
            logger.warning(
                f"-> Low Battery? Compensating: {int(comp_factor * 100)}%"
            )

        # Drive
        final_drive = output / comp_factor
        self.hw.set_motors(
            final_drive + turn_correction, final_drive - turn_correction
        )

        # Balance Point Calibration
        if abs(self.pitch) < BALANCING_THRESHOLD:
            bal_adj = self.balance_finder.update(final_drive, reading.pitch_rate)
            if bal_adj != 0:
                new_target = self.config.pid.target_angle + bal_adj
                limit = self.config.tuner.balance_max_deviation
                # Check absolute limit
                if -limit <= new_target <= limit:
                    self.config.pid.target_angle = new_target
                    self.config_dirty = True
                    logger.info(
                        f"-> Balance Corrected: Target={new_target:.2f} (Adj: {bal_adj})"
                    )
                else:
                    logger.warning(
                        f"-> Balance Correction Ignored: Target {new_target:.2f} exceeds limit {limit}"
                    )

        # Periodic Save
        if self.config_dirty and (
            time.monotonic() - self.last_save_time > SYSTEM_TIMING.save_interval
        ):
            self.config.save()
            self.last_save_time = time.monotonic()
            self.config_dirty = False


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
