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
            self._perform_discovery()

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

    def _perform_discovery(self) -> None:
        logger.info(">>> STARTING AUTONOMOUS DISCOVERY <<<")

        # 1. Measure Back Limit
        logger.info("-> Measuring Back Limit...")
        back_angle = self._measure_stable_angle()
        logger.info(f"-> Back Angle: {back_angle:.2f}")

        # 2. The Flop
        self._drive_until_tip(direction=-1.0)

        # 3. Measure Front Limit
        logger.info("-> Measuring Front Limit...")
        front_angle = self._measure_stable_angle()
        logger.info(f"-> Front Angle: {front_angle:.2f}")

        # 4. Bootstrap
        midpoint = (back_angle + front_angle) / 2
        logger.info(f"-> Calculated Midpoint: {midpoint:.2f}")

        self.config.pid.target_angle = midpoint
        self.config.pid.kp = 3.0
        self.config_dirty = True
        self.first_run = False

        # 5. The Kick-Up
        self._perform_rocking_maneuver(midpoint)

    def _perform_rocking_maneuver(self, target_angle: float) -> None:
        """
        Rock back and then throw forward to catch balance.
        """
        logger.info("-> Kick-Up: Rocking Back...")

        # Phase 1: Rock Back (200ms)
        end_time = time.monotonic() + 0.2
        while time.monotonic() < end_time:
            self.core.update(MotionRequest(), TuningParams(0, 0, 0, 0), self.config.loop_time)
            # Slam backward
            self.core.hw.set_motors(-100, -100)
            time.sleep(self.config.loop_time)

        logger.info("-> Kick-Up: Throwing Forward...")

        # Phase 2: Throw Forward until upright
        # Timeout after 2.0s to prevent runaway
        end_time = time.monotonic() + 2.0
        while time.monotonic() < end_time:
            telemetry = self.core.update(MotionRequest(), TuningParams(0, 0, 0, 0), self.config.loop_time)
            # Throw forward
            self.core.hw.set_motors(100, 100)

            error = abs(telemetry.pitch_angle - target_angle)
            if error < 5.0:
                logger.info(f"-> Catch! Error: {error:.2f}")
                break

            time.sleep(self.config.loop_time)

        # Do not stop motors here; we exit directly into the main loop
        # which will pick up control immediately.

    def _measure_stable_angle(self, duration: float = 1.0) -> float:
        """Measure average pitch over a duration."""
        end_time = time.monotonic() + duration
        pitch_sum = 0.0
        count = 0
        while time.monotonic() < end_time:
            telemetry = self.core.update(MotionRequest(), TuningParams(0, 0, 0, 0), self.config.loop_time)
            pitch_sum += telemetry.pitch_angle
            count += 1
            time.sleep(self.config.loop_time)
        return pitch_sum / max(1, count)

    def _drive_until_tip(self, direction: float = -1.0) -> None:
        """
        Ramp motors in direction until pitch changes significantly.
        Assumes Kp=0 (No Reflex).
        """
        logger.info(f"-> Flop: Ramping {'Backward' if direction < 0 else 'Forward'}...")

        start_angle = self.core.pitch
        ramp_speed = 0.0
        increment = 1.0  # Ramp speed increment per tick

        # Safety timeout
        start_time = time.monotonic()
        while time.monotonic() - start_time < 5.0:
            # 1. Update Core (Keep filter alive)
            telemetry = self.core.update(MotionRequest(), TuningParams(0, 0, 0, 0), self.config.loop_time)

            # 2. Ramp Motors manually (Override Core)
            ramp_speed += (direction * increment)
            self.core.hw.set_motors(ramp_speed, ramp_speed)

            # 3. Check for tip
            if abs(telemetry.pitch_angle - start_angle) > 10.0:
                logger.info(f"-> Tipped! Angle delta: {abs(telemetry.pitch_angle - start_angle):.1f}")
                break

            time.sleep(self.config.loop_time)

        self.core.hw.stop()
        # Wait for settle
        time.sleep(1.0)
