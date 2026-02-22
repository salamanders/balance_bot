import sys
import time
import logging
import json
import concurrent.futures
from ..config import (
    CONFIG_FILE,
    RobotConfig,
    PIDParams,
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
from ..enums import Orientation, Direction

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
                if self.config.balance_verified:
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
        self.recovery = RecoveryManager(self.config.control)

        # Tier 3
        self.led = LedController(self.config.led)
        self.battery_logger = LogThrottler(self.config.timing.battery_log_interval)
        self.tuning_logger = LogThrottler(self.config.timing.tuning_log_interval)

        # State
        self.running = True
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
            rate = RateLimiter(1.0 / self.config.loop_time)
            dt = self.config.loop_time
            while time.perf_counter() - start_wait < self.config.timing.setup_wait:
                # We must spin the core to settle the filter
                self.core.update(
                    self._zero_motion_disabled, self._zero_tuning, dt
                )
                self.led.update()
                dt = rate.sleep()

            # 2. Calibration / Startup
            # Note: WiringCheck now handles all discovery. We just need to stand up.
            if self.first_run or check_force_calibration_flag():
                logger.info(
                    ">>> First Run / Force Calib detected. Assuming WiringCheck passed."
                )
                pass

            # Normal Startup: Check if we need to Kick Up
            # If we are resting on either wheel (Pitch > 10 or < -10), kick up.
            if abs(self.core.pitch) > 10.0:
                logger.info(
                    f">>> Resting on Wheel (Pitch={self.core.pitch:.1f}). Initiating Kick-Up."
                )
                try:
                    # Start with saved kickup power
                    # If Pitch < 0 (Back), we kick Back->Front (Forward Power)
                    # If Pitch > 0 (Front), we kick Front->Back (Backward Power)
                    pwr = (
                        self.config.control.kickup_power_forward
                        if self.core.pitch < 0
                        else self.config.control.kickup_power_backward
                    )
                    self._incremental_kickup(
                        self.config.pid.target_angle, start_power=pwr
                    )
                except Exception as e:
                    logger.error(f"Kick-Up Failed: {e}")
                    return

            # 3. Main Loop
            logger.info(
                f"-> Starting Control Loop. Aggression: {self.tuner.get_current_scale():.2f}"
            )

            # Optimize I2C retries for high-frequency loop (Fail Fast)
            logger.info("-> Optimizing I2C Retries for Real-Time Loop (1 Retry)")
            self.core.set_i2c_retries(1)

            self.led.signal_ready()

            rate = RateLimiter(1.0 / self.config.loop_time)
            # Start with nominal dt
            dt = self.config.loop_time

            # Internal State tracking for Adaptation
            last_pitch_rate = 0.0
            # Initialize dummy telemetry for the first cycle
            last_telemetry = None

            # Pre-allocate TuningParams for high-frequency reuse
            tuning_params = TuningParams(0.0, 0.0, 0.0, 0.0)
            motion_req = MotionRequest(
                velocity=0.0, turn_rate=0.0, enable_control=True
            )

            while self.running:
                self.ticks += 1

                # Default Motion Request (Velocity 0)
                # Check for Resting State (Parking)
                # If we are resting on a bumper and not commanded to move, disable control.
                enable_control = True
                posture = self.core.hw.get_posture_state()
                if posture == "RESTING":
                    # Check if we are still (not tumbling)
                    p_rate = abs(last_telemetry.pitch_rate) if last_telemetry else 0.0
                    if p_rate < 10.0:
                        enable_control = False

                motion_req.velocity = 0.0
                motion_req.turn_rate = 0.0
                motion_req.enable_control = enable_control

                # --- PREPARE INPUTS (Adaptation Phase) ---
                # Use data from the PREVIOUS frame to adjust parameters for THIS frame.

                # Defaults
                tune_kp = self.config.pid.kp
                tune_ki = self.config.pid.ki
                tune_kd = self.config.pid.kd
                target_offset = 0.0

                if last_telemetry:
                    # Only run adaptation if control is enabled
                    if enable_control:
                        # 1. Recovery Logic
                        # Returns an absolute target angle if recovering, or None.
                        rec_target = self.recovery.update(
                            last_telemetry.crashed, last_telemetry.pitch_angle, tune_kp
                        )

                        if rec_target is not None:
                            # Convert Absolute Target -> Offset
                            target_offset = rec_target - self.config.pid.target_angle

                        # 2. Continuous Tuning (Every tick or subsampled?)
                        # Tuner expects to run every tick to fill its buffer
                        # Error = Pitch - Target
                        # Note: We use the target from LAST frame (approximation)
                        curr_error = (
                            last_telemetry.pitch_angle - self.config.pid.target_angle
                        )

                        # Only tune if not recovering
                        if rec_target is None:
                            adj = self.tuner.update(curr_error)
                            if adj.kp != 0 or adj.ki != 0 or adj.kd != 0:
                                self.config.pid.kp = max(
                                    0.1, self.config.pid.kp + adj.kp
                                )
                                self.config.pid.ki = max(
                                    0.0, self.config.pid.ki + adj.ki
                                )
                                self.config.pid.kd = max(
                                    0.0, self.config.pid.kd + adj.kd
                                )
                                self.config_dirty = True
                                if self.tuning_logger.should_log():
                                    logger.info(
                                        f"-> Tuned: P={self.config.pid.kp:.2f} I={self.config.pid.ki:.3f} D={self.config.pid.kd:.2f}"
                                    )
                                # Update local vars
                                tune_kp = self.config.pid.kp
                                tune_ki = self.config.pid.ki
                                tune_kd = self.config.pid.kd

                        # 3. Balance Point Finding
                        # Runs only when balanced AND NOT MOVING
                        if (
                            not last_telemetry.crashed
                            and rec_target is None
                            and motion_req.velocity == 0.0
                            and motion_req.turn_rate == 0.0
                        ):
                            # Hyper-Learning Logic
                            if not self.config.balance_verified:
                                aggression = 10.0
                            else:
                                aggression = 1.0

                            # Compensate motor output for battery to get "effort"
                            effort = (
                                last_telemetry.motor_output
                                / self.battery.compensation_factor
                            )
                            bal_adj = self.balance_finder.update(
                                effort, last_telemetry.pitch_rate, aggression=aggression
                            )

                            if bal_adj != 0:
                                new_target = self.config.pid.target_angle + bal_adj
                                limit = self.config.tuner.balance_max_deviation
                                if (
                                    abs(new_target) <= limit
                                ):  # Simplified check assuming 0 center
                                    self.config.pid.target_angle = new_target
                                    self.config_dirty = True
                                    logger.info(
                                        f"-> Balance Corrected: Target={new_target:.2f}"
                                    )

                                    # Graduation Logic
                                    if (
                                        not self.config.balance_verified
                                        and abs(effort) < 10.0
                                    ):
                                        logger.info(
                                            ">>> Balance Stabilized! Saving to Config."
                                        )
                                        self.config.balance_verified = True
                                        self.config_dirty = True
                    else:
                        # Control Disabled (Resting/Idle)
                        # Do not run Tuner/BalanceFinder/Recovery updates to avoid polluting state with "resting" data.
                        pass

                    # 4. Battery Estimation (Always run to track usage/voltage drops even if idle?)
                    # Actually, if idle, output is 0.
                    ang_accel = (last_telemetry.pitch_rate - last_pitch_rate) / dt
                    last_pitch_rate = last_telemetry.pitch_rate

                    comp_factor = self.battery.update(
                        last_telemetry.motor_output, ang_accel, dt
                    )

                    if (
                        comp_factor < self.config.control.low_battery_log_threshold
                        and self.battery_logger.should_log()
                    ):
                        logger.warning(
                            f"-> Low Battery? Compensating: {int(comp_factor * 100)}%"
                        )

                # --- TIER 3: BEHAVIOR (Cognition) ---
                # Very simple "Wait" behavior for now.
                if self.ticks % 10 == 0:
                    self.led.update()
                    if self.config_dirty and (
                        time.monotonic() - self.last_save_time
                        > self.config.timing.save_interval
                    ):
                        # Asynchronous Configuration Save
                        try:
                            config_snapshot = self.config.model_dump()
                            # Only capture snapshot in main thread; serialize in background
                            self.io_executor.submit(
                                self._save_config_worker, config_snapshot
                            )

                            self.last_save_time = time.monotonic()
                            self.config_dirty = False
                        except Exception as e:
                            logger.error(f"Failed to initiate async config save: {e}")

                # --- TIER 1: REFLEX (Execution) ---
                # Update existing object to avoid allocation
                tuning_params.kp = tune_kp
                tuning_params.ki = tune_ki
                tuning_params.kd = tune_kd
                tuning_params.target_angle_offset = target_offset

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
                self.config.save()
            self.io_executor.shutdown(wait=True)

    def _save_config_worker(self, config_data: dict) -> None:
        """Background worker to write config to disk."""
        try:
            # Serialize in background thread
            json_content = json.dumps(config_data, indent=4)
            CONFIG_FILE.write_text(json_content)
            logger.info("Config saved (Async).")
        except Exception as e:
            logger.error(f"Error saving config asynchronously: {e}")

    def _wait_for_settle(
        self, duration: float = 1.0, rate_threshold: float = 10.0
    ) -> None:
        """Wait for the robot to settle (low pitch rate)."""
        logger.info("-> Waiting for settle...")
        end_time = time.perf_counter() + duration
        rate = RateLimiter(1.0 / self.config.loop_time)
        dt = self.config.loop_time
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
        rate = RateLimiter(1.0 / self.config.loop_time)
        dt = self.config.loop_time
        while time.perf_counter() < end_time:
            self.core.update(self._zero_motion_enabled, self._zero_tuning, dt)
            dt = rate.sleep()

    def _incremental_kickup(self, target_angle: float, start_power: float) -> None:
        """
        Incrementally attempt to kick up to balance.
        Supports starting from either Back (Negative Pitch) or Front (Positive Pitch).
        """
        power = start_power
        step = 5.0
        max_power = 100.0

        # Detect direction based on current pitch
        # If Pitch < 0 (Back): Drive NEGATIVE (Backward) to kick Front up.
        # If Pitch > 0 (Front): Drive POSITIVE (Forward) to kick Back up.
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

        while power <= max_power:
            self._wait_for_settle()

            # Check if we are still at the starting limit
            # If start Back (dir=-1): pitch must be < -10. If > -10, we are too upright/forward.
            # If start Front (dir=1): pitch must be > 10. If < 10, we are too upright/back.
            wrong_position = False
            if kick_direction == Direction.BACKWARD and self.core.pitch > -10:
                wrong_position = True
            elif kick_direction == Direction.FORWARD and self.core.pitch < 10:
                wrong_position = True

            if wrong_position:
                logger.warning(f"-> Not at {start_label} Limit? Repositioning...")
                # Drive OPPOSITE to kick_direction to fall back to start.
                # If kick_dir = -1 (Need to go Neg), we are too Pos. Drive Pos to fall back. (fix = +1)
                # If kick_dir = +1 (Need to go Pos), we are too Neg. Drive Neg to fall back. (fix = -1)
                fix_power = 60.0 * (-kick_direction)

                self.core.hw.set_motors(fix_power, fix_power)
                self._sleep_with_update(0.5)
                self.core.hw.stop()
                self._wait_for_settle()

                # Re-evaluate direction just in case, though we stick to original plan
                if (kick_direction == Direction.BACKWARD and self.core.pitch > -10) or (
                    kick_direction == Direction.FORWARD and self.core.pitch < 10
                ):
                    logger.warning("-> Reposition failed or confused. Retrying loop.")
                    continue

            logger.info(
                f"-> Kick-Up Attempt: Power {power:.1f} Direction {kick_direction}"
            )

            # 1. Lift
            # Drive in the kick_direction
            drive_val = power * kick_direction
            self.core.hw.set_motors(drive_val, drive_val)

            self._sleep_with_update(0.25)

            # 2. Catch (Enter PID Loop)
            logger.info("-> Attempting Catch...")
            catch_start = time.perf_counter()
            caught = False

            # Use a slightly stiff PID for the catch
            catch_params = TuningParams(
                kp=self.config.pid.kp * 1.5,
                ki=self.config.pid.ki,
                kd=self.config.pid.kd * 2.0,
                target_angle_offset=0,
            )

            # We want to run this catch loop for enough time to stabilize
            rate = RateLimiter(1.0 / self.config.loop_time)
            dt = self.config.loop_time
            while time.perf_counter() - catch_start < 2.5:
                # We are now in a mini control loop
                telem = self.core.update(self._zero_motion_enabled, catch_params, dt)

                # Check if stable
                error = abs(telem.pitch_angle - target_angle)
                if error < 5.0 and abs(telem.pitch_rate) < 30.0:
                    # We are upright-ish!
                    pass

                # Check failure (Hard crash to either side)
                # Note: We started at Back (-40ish). If we are back there, we failed.
                # If we went to Front (> 40), we failed.
                if abs(telem.pitch_angle - target_angle) > 40.0:
                    # Allow some initial swing, but if we stay crashed...
                    # For now, just let the PID try its best.
                    pass

                dt = rate.sleep()

            # Check result after catch attempt
            final_error = abs(self.core.pitch - target_angle)
            if final_error < 10.0:
                logger.info("-> Catch Success!")
                caught = True

            if caught:
                return  # Success!

            self.core.hw.stop()
            logger.info("-> Catch Failed. Retrying...")
            power += step

        raise RuntimeError("Failed to Kick-Up.")
