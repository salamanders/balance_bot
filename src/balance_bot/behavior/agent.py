import sys
import time
import logging

from ..config import (
    CONFIG_FILE,
    RobotConfig,
    PIDParams,
    SYSTEM_TIMING,
)
from ..utils import RateLimiter, LogThrottler, setup_logging, check_force_calibration_flag
from ..reflex.balance_core import BalanceCore, MotionRequest, TuningParams
from ..adaptation.recovery import RecoveryManager
from ..adaptation.tuner import ContinuousTuner, BalancePointFinder
from ..adaptation.battery import BatteryEstimator
from .leds import LedController

logger = logging.getLogger(__name__)

class Agent:
    """
    Tier 3: The Cortex.
    Orchestrates the robot's behavior, manages state, and schedules sub-systems.
    """

    def __init__(self):
        setup_logging()

        # 1. Configuration
        self.force_tune = "--tune" in sys.argv
        self.has_saved_config = CONFIG_FILE.exists()
        force_calib = check_force_calibration_flag()

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

        # 2. Subsystems
        # Tier 1
        self.core = BalanceCore(self.config)

        # Tier 2
        self.tuner = ContinuousTuner(self.config.tuner)
        self.tuner.reset_aggression(self.first_run or self.force_tune)
        self.balance_finder = BalancePointFinder(self.config.tuner)
        self.battery = BatteryEstimator(self.config.battery)
        self.recovery = RecoveryManager()

        # Tier 3
        self.led = LedController(self.config.led)
        self.battery_logger = LogThrottler(SYSTEM_TIMING.battery_log_interval)

        # State
        self.running = True
        self.config_dirty = False
        self.last_save_time = time.monotonic()
        self.ticks = 0

    def init(self) -> None:
        """Initialize hardware and signal readiness."""
        logger.info(">>> ROBOT ALIVE. Hold vertical for STEP 1.")
        # Core init happens in __init__ currently, but we can verify here?
        pass

    def run(self) -> None:
        """
        Main Event Loop.
        """
        # 1. Warmup
        logger.info("-> Warming up sensors...")
        self.led.signal_setup()
        start_wait = time.monotonic()
        while time.monotonic() - start_wait < SYSTEM_TIMING.setup_wait:
            # We must spin the core to settle the filter
            self.core.update(MotionRequest(), TuningParams(0,0,0,0), self.config.loop_time)
            self.led.update()
            time.sleep(self.config.loop_time)

        # 2. Calibration
        if self.first_run or check_force_calibration_flag():
            self.run_auto_calibrate()

        # 3. Main Loop
        logger.info(f"-> Starting Control Loop. Aggression: {self.tuner.get_current_scale():.2f}")
        self.led.signal_ready()

        rate = RateLimiter(1.0 / self.config.loop_time)

        # Internal State tracking for Adaptation
        last_pitch_rate = 0.0
        # Initialize dummy telemetry for the first cycle
        last_telemetry = None

        try:
            while self.running:
                self.ticks += 1

                # --- PREPARE INPUTS (Adaptation Phase) ---
                # Use data from the PREVIOUS frame to adjust parameters for THIS frame.

                # Defaults
                tune_kp = self.config.pid.kp
                tune_ki = self.config.pid.ki
                tune_kd = self.config.pid.kd
                target_offset = 0.0

                if last_telemetry:
                    # 1. Recovery Logic
                    # Returns an absolute target angle if recovering, or None.
                    rec_target = self.recovery.update(
                        last_telemetry.crashed,
                        last_telemetry.pitch_angle,
                        tune_kp
                    )

                    if rec_target is not None:
                        # Convert Absolute Target -> Offset
                        target_offset = rec_target - self.config.pid.target_angle

                    # 2. Continuous Tuning (Every tick or subsampled?)
                    # Tuner expects to run every tick to fill its buffer
                    # Error = Pitch - Target
                    # Note: We use the target from LAST frame (approximation)
                    curr_error = last_telemetry.pitch_angle - self.config.pid.target_angle

                    # Only tune if not recovering
                    if rec_target is None:
                        adj = self.tuner.update(curr_error)
                        if adj.kp != 0 or adj.ki != 0 or adj.kd != 0:
                            self.config.pid.kp = max(0.1, self.config.pid.kp + adj.kp)
                            self.config.pid.ki = max(0.0, self.config.pid.ki + adj.ki)
                            self.config.pid.kd = max(0.0, self.config.pid.kd + adj.kd)
                            self.config_dirty = True
                            logger.info(
                                f"-> Tuned: P={self.config.pid.kp:.2f} I={self.config.pid.ki:.3f} D={self.config.pid.kd:.2f}"
                            )
                            # Update local vars
                            tune_kp = self.config.pid.kp
                            tune_ki = self.config.pid.ki
                            tune_kd = self.config.pid.kd

                    # 3. Balance Point Finding
                    # Runs only when balanced
                    if not last_telemetry.crashed and rec_target is None:
                        # Compensate motor output for battery to get "effort"
                        effort = last_telemetry.motor_output / self.battery.compensation_factor
                        bal_adj = self.balance_finder.update(effort, last_telemetry.pitch_rate)

                        if bal_adj != 0:
                            new_target = self.config.pid.target_angle + bal_adj
                            limit = self.config.tuner.balance_max_deviation
                            if abs(new_target) <= limit: # Simplified check assuming 0 center
                                self.config.pid.target_angle = new_target
                                self.config_dirty = True
                                logger.info(f"-> Balance Corrected: Target={new_target:.2f}")

                    # 4. Battery Estimation
                    ang_accel = (last_telemetry.pitch_rate - last_pitch_rate) / self.config.loop_time
                    last_pitch_rate = last_telemetry.pitch_rate

                    comp_factor = self.battery.update(
                        last_telemetry.motor_output,
                        ang_accel,
                        self.config.loop_time
                    )

                    if comp_factor < self.config.control.low_battery_log_threshold and self.battery_logger.should_log():
                         logger.warning(f"-> Low Battery? Compensating: {int(comp_factor * 100)}%")

                # --- TIER 3: BEHAVIOR (Cognition) ---
                # Very simple "Wait" behavior for now.
                if self.ticks % 10 == 0:
                    self.led.update()
                    if self.config_dirty and (time.monotonic() - self.last_save_time > SYSTEM_TIMING.save_interval):
                        self.config.save()
                        self.last_save_time = time.monotonic()
                        self.config_dirty = False

                # --- TIER 1: REFLEX (Execution) ---
                tuning_params = TuningParams(
                    kp=tune_kp,
                    ki=tune_ki,
                    kd=tune_kd,
                    target_angle_offset=target_offset
                )

                motion_req = MotionRequest(velocity=0.0, turn_rate=0.0)

                last_telemetry = self.core.update(
                    motion_req,
                    tuning_params,
                    self.config.loop_time,
                    battery_compensation=self.battery.compensation_factor
                )

                rate.sleep()

        except KeyboardInterrupt:
            logger.info("Keyboard Interrupt.")
        finally:
            self.core.cleanup()
            self.led.signal_off()
            if self.config_dirty:
                self.config.save()

    def run_auto_calibrate(self) -> None:
        """
        Interactive calibration sequence.
        """
        self.led.signal_setup()
        logger.info(">>> AUTO CALIBRATION SEQUENCE <<<")
        print("\n" + "=" * 40)
        print("STEP 1: Place robot resting on FRONT training wheels (leaning forward).")
        print("Ensure it is stable.")
        input("Press ENTER when ready...")

        logger.info("Measuring...")
        min_angle = self._wait_for_stable_pitch()
        logger.info(f"-> Measured Front Angle: {min_angle:.2f}")

        print("\nSTEP 2: Place robot resting on BACK training wheels (leaning backward).")
        print("Ensure it is stable.")
        input("Press ENTER when ready...")

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
        last_pitch = 0.0
        while time.monotonic() < end_time:
            telemetry = self.core.update(MotionRequest(), TuningParams(0,0,0,0), self.config.loop_time)
            last_pitch = telemetry.pitch_angle
            time.sleep(self.config.loop_time)
        return last_pitch
