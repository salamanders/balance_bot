import sys
import time
import logging
import json
import concurrent.futures
from dataclasses import asdict

from ..configuration import (
    HARDWARE_CONFIG_FILE,
    LEARNING_STATE_FILE,
    HardwareConfig,
    LearningState,
)
from ..utils import (
    RateLimiter,
    LogThrottler,
    setup_logging,
    check_force_calibration_flag,
)
from ..reflex.balance_core import BalanceCore, MotionRequest, TuningParams
from ..adaptation.recovery import RecoveryManager
from ..adaptation.tuner import ContinuousTuner, BalancePointFinder
from ..adaptation.battery import BatteryEstimator
from .leds import LedController
from ..enums import Orientation, Direction, BotState

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
        self.has_saved_config = HARDWARE_CONFIG_FILE.exists()
        force_calib = check_force_calibration_flag()

        if force_calib:
            logger.info("Forcing calibration: Using default configuration.")
            self.hw_config = HardwareConfig()
            self.learning_state = LearningState()
            self.first_run = True
        else:
            self.hw_config = HardwareConfig.load()
            self.learning_state = LearningState.load()

            if self.has_saved_config:
                if self.learning_state.balance_verified:
                    logger.info(">>> Production Mode: Balance Verified.")
                else:
                    logger.info(
                        ">>> Discovery Mode: Balance NOT Verified. (Hyper-Learning Enabled)"
                    )
                self.first_run = False
            else:
                logger.info(">>> No Config Found. Starting in FIRST RUN MODE.")
                self.first_run = True

        if self.first_run:
            # Zero out PID for learning phase
            self.learning_state.kp = 0.0
            self.learning_state.ki = 0.0
            self.learning_state.kd = 0.0

        # 2. Subsystems
        # Tier 1
        self.core = BalanceCore(self.hw_config, self.learning_state)

        # Tier 2
        self.tuner = ContinuousTuner(self.hw_config.tuner)
        self.tuner.reset_aggression(self.first_run or self.force_tune)
        self.balance_finder = BalancePointFinder(self.hw_config.tuner)
        self.battery = BatteryEstimator(self.hw_config.battery)
        self.recovery = RecoveryManager(self.hw_config.control)

        # Tier 3
        self.led = LedController(self.hw_config.led)
        self.battery_logger = LogThrottler(self.hw_config.timing.battery_log_interval)
        self.tuning_logger = LogThrottler(self.hw_config.timing.tuning_log_interval)

        # State
        self.running = True
        self.state = BotState.IDLE
        self.kickup_attempts = 0
        self.last_crash_time = 0.0

        self.config_dirty = False
        self.last_save_time = time.monotonic()
        self.ticks = 0
        self.io_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)

        # Pre-allocated zero tuning params for waiting/measuring loops
        self._zero_tuning = TuningParams(0.0, 0.0, 0.0, 0.0)
        self._zero_motion_enabled = MotionRequest(
            velocity=0.0, turn_rate=0.0, enable_control=True
        )
        self._zero_motion_disabled = MotionRequest(
            velocity=0.0, turn_rate=0.0, enable_control=False
        )

    def run(self) -> None:
        """
        Main Event Loop.
        """
        try:
            # 1. Warmup
            logger.info("-> Warming up sensors...")
            self.led.signal_setup()
            start_wait = time.perf_counter()
            rate = RateLimiter(1.0 / self.hw_config.loop_time)
            dt = self.hw_config.loop_time
            while time.perf_counter() - start_wait < self.hw_config.timing.setup_wait:
                # We must spin the core to settle the filter
                self.core.update(
                    self._zero_motion_disabled, self._zero_tuning, dt
                )
                self.led.update()
                dt = rate.sleep()

            # 2. Calibration / Startup (Legacy hook)
            if self.first_run or check_force_calibration_flag():
                logger.info(">>> First Run / Force Calib detected.")
                pass

            logger.info(
                f"-> Starting Control Loop. Aggression: {self.tuner.get_current_scale():.2f}"
            )
            logger.info("-> Optimizing I2C Retries for Real-Time Loop (1 Retry)")
            self.core.set_i2c_retries(1)

            self.led.signal_ready()

            rate = RateLimiter(1.0 / self.hw_config.loop_time)
            dt = self.hw_config.loop_time

            # Internal State tracking for Adaptation
            last_pitch_rate = 0.0
            last_telemetry = None

            # Reusable objects
            tuning_params = TuningParams(0.0, 0.0, 0.0, 0.0)
            motion_req = MotionRequest(
                velocity=0.0, turn_rate=0.0, enable_control=True
            )

            while self.running:
                self.ticks += 1

                # ---------------------------------------------------------
                # STATE MACHINE
                # ---------------------------------------------------------

                # Default behavior: Disable control unless BALANCING logic enables it
                motion_req.enable_control = False
                motion_req.velocity = 0.0
                motion_req.turn_rate = 0.0

                # Defaults for Tuning/Recovery (calculated from PREVIOUS frame)
                tune_kp = self.learning_state.kp
                tune_ki = self.learning_state.ki
                tune_kd = self.learning_state.kd
                target_offset = 0.0

                match self.state:
                    case BotState.IDLE:
                        # Logic:
                        # If pitch is small (< 10 deg) -> User stood it up -> BALANCING
                        # If pitch is large (> 10 deg) -> Resting -> KICKUP (if attempts < 3)
                        # Else -> Stay IDLE (Resting and out of attempts, or waiting)

                        pitch = self.core.pitch  # Get latest pitch from core (property)

                        if abs(pitch) < 10.0:
                            logger.info(f"-> Detected Upright ({pitch:.1f}). Transition to BALANCING.")
                            self.state = BotState.BALANCING
                            self.kickup_attempts = 0 # Reset attempts
                        elif abs(pitch) > 10.0:
                            # Resting. Check if we should kick up.
                            if self.kickup_attempts < 3:
                                logger.info(f"-> Resting ({pitch:.1f}). Transition to KICKUP (Attempt {self.kickup_attempts + 1}/3).")
                                self.state = BotState.KICKUP
                            else:
                                # Too many failures. Stay IDLE until human resets (stands it up).
                                if self.ticks % 500 == 0: # Log occasionally
                                    logger.warning("-> Max Kick-Up attempts reached. Waiting for manual reset.")
                                pass

                    case BotState.KICKUP:
                        # Execute Kick-Up Sequence (Blocking for now)
                        pwr = (
                            self.hw_config.control.kickup_power_forward
                            if self.core.pitch < 0
                            else self.hw_config.control.kickup_power_backward
                        )
                        success = self._incremental_kickup(
                            self.learning_state.target_angle, start_power=pwr
                        )

                        if success:
                            logger.info("-> Kick-Up Successful! Transition to BALANCING.")
                            self.state = BotState.BALANCING
                            self.kickup_attempts = 0
                            # _incremental_kickup ends with stop(), so next loop will catch.
                            # We might need to reset PID terms? Core handles it if we disabled control.
                        else:
                            logger.warning("-> Kick-Up Failed. Transition to IDLE.")
                            self.state = BotState.IDLE
                            self.kickup_attempts += 1
                            # Give it a moment to settle in IDLE before retrying is handled by IDLE logic naturally
                            # because IDLE checks pitch.

                        # Since kickup consumed time, reset dt
                        dt = self.hw_config.loop_time
                        last_telemetry = None # Telemetry is stale

                    case BotState.BALANCING:
                        motion_req.enable_control = True

                        # 1. Crash Detection
                        # We use the LAST known pitch from telemetry if available, else current core.pitch
                        current_pitch = last_telemetry.pitch_angle if last_telemetry else self.core.pitch

                        if abs(current_pitch) > self.hw_config.crash_angle:
                            logger.warning(f"-> Crash Detected ({current_pitch:.1f} > {self.hw_config.crash_angle}). Transition to CRASHED.")
                            self.core.hw.stop()
                            self.state = BotState.CRASHED
                            self.last_crash_time = time.monotonic()
                            motion_req.enable_control = False

                        # 2. Adaptation (Recovery, Tuning, Balance Point)
                        elif last_telemetry:
                            # A. Recovery
                            rec_target = self.recovery.update(
                                False, # Not crashed (handled above)
                                last_telemetry.pitch_angle,
                                tune_kp
                            )

                            if rec_target is not None:
                                target_offset = rec_target - self.learning_state.target_angle

                            # B. Tuning (Only if not recovering)
                            curr_error = last_telemetry.pitch_angle - self.learning_state.target_angle
                            if rec_target is None:
                                adj = self.tuner.update(curr_error)
                                if adj.kp != 0 or adj.ki != 0 or adj.kd != 0:
                                    self.learning_state.kp = max(0.1, self.learning_state.kp + adj.kp)
                                    self.learning_state.ki = max(0.0, self.learning_state.ki + adj.ki)
                                    self.learning_state.kd = max(0.0, self.learning_state.kd + adj.kd)
                                    self.config_dirty = True
                                    if self.tuning_logger.should_log():
                                        logger.info(f"-> Tuned: P={self.learning_state.kp:.2f} I={self.learning_state.ki:.3f} D={self.learning_state.kd:.2f}")
                                    tune_kp, tune_ki, tune_kd = self.learning_state.kp, self.learning_state.ki, self.learning_state.kd

                            # C. Balance Finder (Only if balanced and stationary)
                            if (rec_target is None
                                and motion_req.velocity == 0.0
                                and motion_req.turn_rate == 0.0):

                                aggression = 10.0 if not self.learning_state.balance_verified else 1.0
                                effort = last_telemetry.motor_output / self.battery.compensation_factor
                                bal_adj = self.balance_finder.update(effort, last_telemetry.pitch_rate, aggression=aggression)

                                if bal_adj != 0:
                                    new_target = self.learning_state.target_angle + bal_adj
                                    if abs(new_target) <= self.hw_config.tuner.balance_max_deviation:
                                        self.learning_state.target_angle = new_target
                                        self.config_dirty = True
                                        logger.info(f"-> Balance Corrected: Target={new_target:.2f}")

                                        if not self.learning_state.balance_verified and abs(effort) < 10.0:
                                            logger.info(">>> Balance Stabilized! Saving to Config.")
                                            self.learning_state.balance_verified = True
                                            self.config_dirty = True

                    case BotState.CRASHED:
                        # Safety wait
                        motion_req.enable_control = False
                        if time.monotonic() - self.last_crash_time > 2.0:
                            # Check if we are still resting/crashed or if human picked us up?
                            # Actually, just transition to IDLE. IDLE will decide what to do next.
                            logger.info("-> Crash Timeout Expired. Transition to IDLE.")
                            self.state = BotState.IDLE

                # ---------------------------------------------------------
                # EXECUTION
                # ---------------------------------------------------------

                # Check background tasks (Saving)
                if self.ticks % 10 == 0:
                    self.led.update()
                    if self.config_dirty and (time.monotonic() - self.last_save_time > self.hw_config.timing.save_interval):
                        try:
                            # Serialize snapshot
                            config_snapshot = asdict(self.learning_state)
                            self.io_executor.submit(self._save_config_worker, config_snapshot)
                            self.last_save_time = time.monotonic()
                            self.config_dirty = False
                        except Exception as e:
                            logger.error(f"Failed to initiate async config save: {e}")

                # Update Battery Logic (Always run to keep voltage filter updated)
                if last_telemetry:
                    ang_accel = (last_telemetry.pitch_rate - last_pitch_rate) / dt
                    last_pitch_rate = last_telemetry.pitch_rate
                    comp_factor = self.battery.update(last_telemetry.motor_output, ang_accel, dt)
                    if comp_factor < self.hw_config.control.low_battery_log_threshold and self.battery_logger.should_log():
                        logger.warning(f"-> Low Battery? Compensating: {int(comp_factor * 100)}%")
                else:
                    # Fallback if no telemetry (e.g. after Kickup)
                    comp_factor = 1.0

                # Reflex Update
                tuning_params.kp = tune_kp
                tuning_params.ki = tune_ki
                tuning_params.kd = tune_kd
                tuning_params.target_angle_offset = target_offset

                # Core Update
                last_telemetry = self.core.update(
                    motion_req,
                    tuning_params,
                    dt,
                    battery_compensation=self.battery.compensation_factor,
                )

                dt = rate.sleep()

        except KeyboardInterrupt:
            logger.info("Keyboard Interrupt.")
        finally:
            self.core.cleanup()
            self.led.signal_off()
            if self.config_dirty:
                self.learning_state.save()
            self.io_executor.shutdown(wait=True)

    def _save_config_worker(self, config_data: dict) -> None:
        """Background worker to write config to disk."""
        try:
            # Serialize in background thread
            json_content = json.dumps(config_data, indent=4)
            LEARNING_STATE_FILE.write_text(json_content)
            logger.info("Config saved (Async).")
        except Exception as e:
            logger.error(f"Error saving config asynchronously: {e}")

    def _wait_for_settle(
        self, duration: float = 1.0, rate_threshold: float = 10.0
    ) -> None:
        """Wait for the robot to settle (low pitch rate)."""
        # logger.info("-> Waiting for settle...") # Reduce log spam
        end_time = time.perf_counter() + duration
        rate = RateLimiter(1.0 / self.hw_config.loop_time)
        dt = self.hw_config.loop_time
        while True:
            # Keep filter alive
            telemetry = self.core.update(
                self._zero_motion_enabled, self._zero_tuning, dt
            )

            if time.perf_counter() > end_time:
                # Check rate
                if abs(telemetry.pitch_rate) < rate_threshold:
                    break
                else:
                    # Extend wait if still moving
                    end_time = time.perf_counter() + 0.5

            dt = rate.sleep()

    def _sleep_with_update(self, duration: float) -> None:
        """Sleep for duration while keeping the core filter updated."""
        end_time = time.perf_counter() + duration
        rate = RateLimiter(1.0 / self.hw_config.loop_time)
        dt = self.hw_config.loop_time
        while time.perf_counter() < end_time:
            self.core.update(self._zero_motion_enabled, self._zero_tuning, dt)
            dt = rate.sleep()

    def _incremental_kickup(self, target_angle: float, start_power: float) -> bool:
        """
        Incrementally attempt to kick up to balance.
        Returns True if successful, False if failed.
        """
        power = start_power
        step = 5.0
        max_power = 100.0

        start_pitch = self.core.pitch
        kick_direction = Direction.BACKWARD if start_pitch < 0 else Direction.FORWARD

        start_label = (
            Orientation.BACK.upper()
            if kick_direction == Direction.BACKWARD
            else Orientation.FRONT.upper()
        )

        logger.info(
            f"-> Starting Incremental Kick-Up from {start_label}. Target: {target_angle:.2f}"
        )

        try:
            while power <= max_power:
                self._wait_for_settle()

                # Safety Check: Are we still in position?
                wrong_position = False
                if kick_direction == Direction.BACKWARD and self.core.pitch > -10:
                    wrong_position = True
                elif kick_direction == Direction.FORWARD and self.core.pitch < 10:
                    wrong_position = True

                if wrong_position:
                    logger.warning(f"-> Not at {start_label} Limit? Repositioning...")
                    fix_power = 60.0 * (-kick_direction)

                    self.core.hw.set_motors(fix_power, fix_power)
                    self._sleep_with_update(0.5)
                    self.core.hw.stop()
                    self._wait_for_settle()

                    if (kick_direction == Direction.BACKWARD and self.core.pitch > -10) or (
                        kick_direction == Direction.FORWARD and self.core.pitch < 10
                    ):
                        logger.warning("-> Reposition failed or confused. Aborting kickup.")
                        return False

                logger.info(
                    f"-> Kick-Up Attempt: Power {power:.1f} Direction {kick_direction}"
                )

                # 1. Lift
                drive_val = power * kick_direction
                self.core.hw.set_motors(drive_val, drive_val)

                self._sleep_with_update(0.25)

                # 2. Catch (Enter PID Loop)
                logger.info("-> Attempting Catch...")
                catch_start = time.perf_counter()
                caught = False

                catch_params = TuningParams(
                    kp=self.learning_state.kp * 1.5,
                    ki=self.learning_state.ki,
                    kd=self.learning_state.kd * 2.0,
                    target_angle_offset=0,
                )

                rate = RateLimiter(1.0 / self.hw_config.loop_time)
                dt = self.hw_config.loop_time
                while time.perf_counter() - catch_start < 2.5:
                    telem = self.core.update(self._zero_motion_enabled, catch_params, dt)

                    error = abs(telem.pitch_angle - target_angle)
                    if error < 5.0 and abs(telem.pitch_rate) < 30.0:
                        pass # Looks good

                    if abs(telem.pitch_angle - target_angle) > 40.0:
                        pass # Failed?

                    dt = rate.sleep()

                # Check result
                final_error = abs(self.core.pitch - target_angle)
                if final_error < 10.0:
                    logger.info("-> Catch Success!")
                    return True

                self.core.hw.stop()
                logger.info("-> Catch Failed. Retrying...")
                power += step

        except Exception as e:
            logger.error(f"Kick-Up Exception: {e}")
            self.core.hw.stop()
            return False

        logger.error("-> Failed to Kick-Up (Max Power Reached).")
        return False
