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

        # 2. Calibration / Startup
        if self.first_run or check_force_calibration_flag():
            try:
                self._perform_discovery()
            except Exception as e:
                logger.error(f"Discovery Failed: {e}")
                self.core.cleanup()
                return
        else:
            # Normal Startup: Check if we need to Kick Up
            # If we are resting on either wheel (Pitch > 10 or < -10), kick up.
            if abs(self.core.pitch) > 10.0:
                logger.info(f">>> Resting on Wheel (Pitch={self.core.pitch:.1f}). Initiating Kick-Up.")
                try:
                    # Start with saved kickup power
                    self._incremental_kickup(
                        self.config.pid.target_angle,
                        start_power=self.config.control.kickup_power
                    )
                except Exception as e:
                    logger.error(f"Kick-Up Failed: {e}")
                    self.core.cleanup()
                    return

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

                # Default Motion Request (Velocity 0)
                motion_req = MotionRequest(velocity=0.0, turn_rate=0.0)

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
                    # Runs only when balanced AND NOT MOVING
                    if (
                        not last_telemetry.crashed
                        and rec_target is None
                        and motion_req.velocity == 0.0
                        and motion_req.turn_rate == 0.0
                    ):
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

        self._wait_for_settle()
        start_pitch = self.core.pitch

        back_angle = 0.0
        front_angle = 0.0
        kickup_power_est = 30.0

        if start_pitch < -5.0:
            start_side = "back"
            other_side = "front"
        elif start_pitch > 5.0:
            start_side = "front"
            other_side = "back"
        else:
            logger.error(f"Cannot perform discovery: Robot is too upright (Pitch: {start_pitch:.1f})")
            return

        logger.info(f"-> Detected {start_side.upper()} Start.")

        # 1. Measure Start Limit
        logger.info(f"-> Measuring {start_side.capitalize()} Limit...")
        angle_1 = self._measure_stable_angle()
        logger.info(f"-> {start_side.capitalize()} Angle: {angle_1:.2f}")

        if start_side == "back":
            back_angle = angle_1
        else:
            front_angle = angle_1

        # 2. Flop to Other Side
        logger.info(f"-> Flop to {other_side.capitalize()}...")
        # Against gravity (Start->Other)
        kickup_power_est = self._incremental_flop(target_side=other_side)

        # 3. Measure Other Limit
        logger.info(f"-> Measuring {other_side.capitalize()} Limit...")
        self._wait_for_settle()
        angle_2 = self._measure_stable_angle()
        logger.info(f"-> {other_side.capitalize()} Angle: {angle_2:.2f}")

        if other_side == "back":
            back_angle = angle_2
        else:
            front_angle = angle_2

        # 4. Return to Start
        logger.info(f"-> Return to Start ({start_side.capitalize()})...")
        self._incremental_flop(target_side=start_side)
        self._wait_for_settle()

        # 5. Bootstrap
        midpoint = (back_angle + front_angle) / 2
        logger.info(f"-> Calculated Midpoint: {midpoint:.2f}")

        self.config.pid.target_angle = midpoint
        # Set conservative starting PID
        self.config.pid.kp = 10.0
        self.config.pid.ki = 0.0 # Keep I/D low initially
        self.config.pid.kd = 0.2

        # Save discovered kick-up power
        self.config.control.kickup_power = kickup_power_est

        self.config_dirty = True
        self.first_run = False

        # 6. Incremental Kick-Up
        logger.info(f"-> Starting Kick-Up Sequence using est power {kickup_power_est:.1f}...")
        self._incremental_kickup(target_angle=midpoint, start_power=kickup_power_est)

    def _wait_for_settle(self, duration: float = 1.0, rate_threshold: float = 10.0) -> None:
        """Wait for the robot to settle (low pitch rate)."""
        logger.info("-> Waiting for settle...")
        end_time = time.monotonic() + duration
        while True:
            # Keep filter alive
            telemetry = self.core.update(MotionRequest(), TuningParams(0, 0, 0, 0), self.config.loop_time)

            if time.monotonic() > end_time:
                # Check rate
                if abs(telemetry.pitch_rate) < rate_threshold:
                    break
                else:
                    # Extend wait if still moving
                    end_time = time.monotonic() + 0.5

            time.sleep(self.config.loop_time)

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

    def _sleep_with_update(self, duration: float) -> None:
        """Sleep for duration while keeping the core filter updated."""
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time:
            self.core.update(MotionRequest(), TuningParams(0, 0, 0, 0), self.config.loop_time)
            time.sleep(self.config.loop_time)

    def _incremental_flop(self, target_side: str) -> float:
        """
        Incrementally apply power until the robot flops to the target side.
        target_side: 'front' or 'back'.
        Returns the power level that caused the flop.
        """
        if target_side == "front":
            # From Back to Front. Drive Negative (Backward).
            direction = -1.0
        else:
            # From Front to Back. Drive Positive (Forward).
            direction = 1.0

        power = 30.0 # Start small
        step = 5.0
        max_power = 100.0

        while power <= max_power:
            self._wait_for_settle()

            logger.info(f"-> Attempting Flop to {target_side.upper()} with Power {power:.1f}...")

            # Impulse
            self.core.hw.set_motors(direction * power, direction * power)
            self._sleep_with_update(0.4) # Short burst
            self.core.hw.stop()

            # Wait for result
            end_wait = time.monotonic() + 1.5
            success = False
            while time.monotonic() < end_wait:
                telem = self.core.update(MotionRequest(), TuningParams(0,0,0,0), self.config.loop_time)

                # Check if we crossed the vertical significantly
                # Front target (Pos angle) means we passed > 10
                # Back target (Neg angle) means we passed < -10
                if (target_side == "front" and telem.pitch_angle > 10.0) or \
                   (target_side == "back" and telem.pitch_angle < -10.0):
                    success = True
                    break
                time.sleep(self.config.loop_time)

            if success:
                logger.info(f"-> Flop Success at Power {power:.1f}")
                self._wait_for_settle() # Let it crash and settle
                return power

            logger.info("-> Failed. Retrying...")
            power += step

        raise RuntimeError(f"Failed to flop to {target_side} even at max power.")

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
        kick_direction = -1.0 if start_pitch < 0 else 1.0

        start_label = "BACK" if kick_direction < 0 else "FRONT"

        logger.info(f"-> Starting Incremental Kick-Up from {start_label}. Target: {target_angle:.2f}")

        while power <= max_power:
            self._wait_for_settle()

            # Check if we are still at the starting limit
            # If start Back (dir=-1): pitch must be < -10. If > -10, we are too upright/forward.
            # If start Front (dir=1): pitch must be > 10. If < 10, we are too upright/back.
            wrong_position = False
            if kick_direction < 0 and self.core.pitch > -10:
                wrong_position = True
            elif kick_direction > 0 and self.core.pitch < 10:
                wrong_position = True

            if wrong_position:
                logger.warning(f"-> Not at {start_label} Limit? Repositioning...")
                # Drive OPPOSITE to kick_direction to fall back to start.
                # If kick_dir = -1 (Need to go Neg), we are too Pos. Drive Pos to fall back. (fix = +1)
                # If kick_dir = +1 (Need to go Pos), we are too Neg. Drive Neg to fall back. (fix = -1)
                fix_power = 30.0 * (-kick_direction)

                self.core.hw.set_motors(fix_power, fix_power)
                self._sleep_with_update(0.5)
                self.core.hw.stop()
                self._wait_for_settle()

                # Re-evaluate direction just in case, though we stick to original plan
                if (kick_direction < 0 and self.core.pitch > -10) or \
                   (kick_direction > 0 and self.core.pitch < 10):
                       logger.warning("-> Reposition failed or confused. Retrying loop.")
                       continue

            logger.info(f"-> Kick-Up Attempt: Power {power:.1f} Direction {kick_direction}")

            # 1. Lift
            # Drive in the kick_direction
            drive_val = power * kick_direction
            self.core.hw.set_motors(drive_val, drive_val)

            self._sleep_with_update(0.25)

            # 2. Catch (Enter PID Loop)
            logger.info("-> Attempting Catch...")
            catch_start = time.monotonic()
            caught = False

            # Use a slightly stiff PID for the catch
            catch_params = TuningParams(
                kp=self.config.pid.kp * 1.5,
                ki=self.config.pid.ki,
                kd=self.config.pid.kd * 2.0,
                target_angle_offset=0
            )

            # We want to run this catch loop for enough time to stabilize
            while time.monotonic() - catch_start < 2.5:
                # We are now in a mini control loop
                telem = self.core.update(MotionRequest(), catch_params, self.config.loop_time)

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

                time.sleep(self.config.loop_time)

            # Check result after catch attempt
            final_error = abs(self.core.pitch - target_angle)
            if final_error < 10.0:
                logger.info("-> Catch Success!")
                caught = True

            if caught:
                return # Success!

            self.core.hw.stop()
            logger.info("-> Catch Failed. Retrying...")
            power += step

        raise RuntimeError("Failed to Kick-Up.")
