import concurrent.futures
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any

from .leds import LedController
from .states import AgentContext, BotState, IdleState
from ..adaptation.battery import BatteryEstimator
from ..adaptation.recovery import RecoveryManager
from ..adaptation.tuner import ContinuousTuner, BalancePointFinder
from ..configuration import (

    HardwareConfig,
    LearningState,
    PIDParams,
)
from ..enums import Orientation, Direction
from ..reflex.balance_core import BalanceCore, MotionRequest, TuningParams
from ..telemetry import TelemetryBlackbox
from ..utils import (
    RateLimiter,
    LogThrottler,
    setup_logging,
    check_force_calibration_flag,
)
from ..watchdog import SurvivalWatchdog

logger = logging.getLogger(__name__)


class Agent:
    """
    Tier 3: The Cortex.
    Orchestrates the robot's behavior, manages state, and schedules sub-systems.
    """

    def __init__(self, watchdog: SurvivalWatchdog | None = None):
        setup_logging()
        self.watchdog = watchdog

        # 1. Configuration
        self.force_tune = "--tune" in sys.argv
        self.has_saved_config = Path("learning_state.json").exists()
        force_calib = check_force_calibration_flag()

        if force_calib:
            logger.info("Forcing calibration: Using default configuration.")
            self.hw_config = HardwareConfig.load()
            self.learning_state = LearningState(pid=PIDParams())
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
            self.learning_state.pid.kp = 0.0
            self.learning_state.pid.ki = 0.0
            self.learning_state.pid.kd = 0.0

        # 2. Subsystems
        # Tier 1
        self.core = BalanceCore(self.hw_config, self.learning_state, watchdog=self.watchdog)

        # Tier 2
        self.tuner = ContinuousTuner(self.learning_state.tuner)
        self.tuner.reset_aggression(self.first_run or self.force_tune)
        self.balance_finder = BalancePointFinder(self.learning_state.tuner)
        self.battery = BatteryEstimator(self.learning_state.battery)
        self.recovery = RecoveryManager(self.learning_state.control)

        # Tier 3
        self.led = LedController(self.learning_state.led)
        self.battery_logger = LogThrottler(self.learning_state.timing.battery_log_interval)
        self.tuning_logger = LogThrottler(self.learning_state.timing.tuning_log_interval)
        self.blackbox = TelemetryBlackbox()

        # State
        self.running = True
        self.state = IdleState()
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

    def run(self) -> None:  # noqa: C901
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
            while time.perf_counter() - start_wait < self.learning_state.timing.setup_wait:
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
            self.blackbox.start()

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
                if self.watchdog:
                    self.watchdog.heartbeat()

                self.ticks += 1

                # ---------------------------------------------------------
                # STATE MACHINE
                # ---------------------------------------------------------

                # Default behavior: Disable control unless BALANCING logic enables it
                motion_req.enable_control = False
                motion_req.velocity = 0.0
                motion_req.turn_rate = 0.0

                # Defaults for Tuning/Recovery (calculated from PREVIOUS frame)
                tune_kp = self.learning_state.pid.kp
                tune_ki = self.learning_state.pid.ki
                tune_kd = self.learning_state.pid.kd
                target_offset = 0.0

                # Check background tasks (Saving)
                if self.ticks % 10 == 0:
                    self.led.update()
                    if self.config_dirty and (
                            time.monotonic() - self.last_save_time > self.learning_state.timing.save_interval):
                        try:
                            config_snapshot = self.learning_state.model_dump()
                            self.io_executor.submit(self._save_config_worker, config_snapshot)
                            self.last_save_time = time.monotonic()
                            self.config_dirty = False
                        except Exception as e:
                            logger.error(f"Failed to initiate async config save: {e}")

                # Update Battery Logic (Always run to keep voltage filter updated)
                if last_telemetry:
                    ang_accel = (last_telemetry.pitch_rate - last_pitch_rate) / dt
                    last_pitch_rate = last_telemetry.pitch_rate
                    _comp_factor = self.battery.update(last_telemetry.motor_output, ang_accel)
                    if float(_comp_factor) < float(
                            self.learning_state.control.low_battery_log_threshold) and self.battery_logger.should_log():
                        logger.warning(f"-> Low Battery? Compensating: {int(_comp_factor * 100)}%")
                else:
                    # Fallback if no telemetry (e.g. after Kickup)
                    pass

                # STATE PATTERN UPDATE
                context = AgentContext(
                    core=self.core,
                    config=self.hw_config,
                    learning_state=self.learning_state,
                    led=self.led,
                    battery=self.battery,
                    recovery=self.recovery,
                    tuner=self.tuner,
                    watchdog=self.watchdog
                )

                next_state = self.state.update(context, dt, motion_req, tuning_params, last_telemetry, self.ticks)
                if type(next_state) is not type(self.state):
                    self.state.exit(context)
                    self.state = next_state
                    self.state.enter(context)

                # Check background tasks (Saving)
                if self.ticks % 10 == 0:
                    self.led.update()
                    if self.config_dirty and (
                            time.monotonic() - self.last_save_time > self.learning_state.timing.save_interval):
                        try:
                            config_snapshot = self.learning_state.model_dump()
                            self.io_executor.submit(self._save_config_worker, config_snapshot)
                            self.last_save_time = time.monotonic()
                            self.config_dirty = False
                        except Exception as e:
                            logger.error(f"Failed to initiate async config save: {e}")

                # Update Battery Logic (Always run to keep voltage filter updated)
                if last_telemetry:
                    ang_accel = (last_telemetry.pitch_rate - last_pitch_rate) / dt
                    last_pitch_rate = last_telemetry.pitch_rate
                    _comp_factor = self.battery.update(last_telemetry.motor_output, ang_accel)
                    if float(_comp_factor) < float(
                            self.learning_state.control.low_battery_log_threshold) and self.battery_logger.should_log():
                        logger.warning(f"-> Low Battery? Compensating: {int(_comp_factor * 100)}%")
                else:
                    # Fallback if no telemetry (e.g. after Kickup)
                    pass

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

                self.blackbox.log_tick(
                    type(self.state).__name__,
                    last_telemetry.pitch_angle,
                    last_telemetry.pitch_rate,
                    last_telemetry.yaw_rate,
                    last_telemetry.left_pwm,
                    last_telemetry.right_pwm,
                    last_telemetry.target_angle
                )

                dt = rate.sleep()

        except KeyboardInterrupt:
            if self.watchdog and self.watchdog.triggered:
                logger.error("WATCHDOG PANIC: Interrupting Agent.")
                raise RuntimeError("Watchdog Panic! Main thread was stuck.") from None
            logger.info("Keyboard Interrupt.")
        finally:
            self.blackbox.stop()
            self.core.cleanup()
            self.led.signal_off()
            if self.config_dirty:
                self.learning_state.save()
            self.io_executor.shutdown(wait=True)

    @staticmethod
    def _save_config_worker(config_data: dict[str, Any]) -> None:
        """Background worker to write config to disk."""
        try:
            # Serialize in background thread
            json_content = json.dumps(config_data, indent=4)
            Path("learning_state.json").write_text(json_content)
            logger.info("Config saved (Async).")
        except Exception as e:
            logger.error(f"Error saving config asynchronously: {e}")

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
                if self.watchdog:
                    self.watchdog.heartbeat()
                self._wait_for_settle()

                # Safety Check: Are we still in position?
                if not self._check_and_fix_position(kick_direction, start_label):
                    return False

                logger.info(
                    f"-> Kick-Up Attempt: Power {power:.1f} Direction {kick_direction}"
                )

                # 1. Lift
                drive_val = float(power) * float(kick_direction.value)
                self.core.hw.set_motors(drive_val, drive_val)

                self._sleep_with_update(0.25)

                # 2. Catch (Enter PID Loop)
                if self._attempt_catch(target_angle):
                    return True

                self.core.hw.stop()
                logger.info("-> Catch Failed. Retrying...")
                power += step

        except Exception as e:
            logger.error(f"Kick-Up Exception: {e}")
            self.core.hw.stop()
            return False

        logger.error("-> Failed to Kick-Up (Max Power Reached).")
        self.state = BotState.FATAL_ERROR
        return False
