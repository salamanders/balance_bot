import logging
import time
from dataclasses import dataclass
from typing import Any

from ..adaptation.battery import BatteryEstimator
from ..adaptation.recovery import RecoveryManager
from ..adaptation.tuner import ContinuousTuner
from ..behavior.leds import LedController
from ..configuration import HardwareConfig, LearningState
from ..enums import Direction
from ..reflex.balance_core import BalanceCore, BalanceTelemetry, MotionRequest, TuningParams
from ..utils import RateLimiter
from ..watchdog import SurvivalWatchdog

logger = logging.getLogger(__name__)


@dataclass
class AgentContext:
    core: BalanceCore
    config: HardwareConfig
    learning_state: LearningState
    led: LedController
    battery: BatteryEstimator
    recovery: RecoveryManager
    tuner: ContinuousTuner
    watchdog: SurvivalWatchdog | None
    deadman_server: Any = None


class BotState:
    def enter(self, context: AgentContext) -> None:
        pass

    def update(
        self,
        context: AgentContext,
        dt: float,
        motion_req: MotionRequest,
        tuning_params: TuningParams,
        last_telemetry: BalanceTelemetry | None,
        ticks: int,
    ) -> "BotState":
        return self

    def exit(self, context: AgentContext) -> None:
        pass


class IdleState(BotState):
    def __init__(self, kickup_attempts: int = 0):
        self.kickup_attempts = kickup_attempts

    def update(
        self,
        context: AgentContext,
        dt: float,
        motion_req: MotionRequest,
        tuning_params: TuningParams,
        last_telemetry: BalanceTelemetry | None,
        ticks: int,
    ) -> BotState:
        # Hold in IDLE if deadman switch is enabled but not actively engaged
        if context.deadman_server is not None and not context.deadman_server.is_alive():
            if ticks % 200 == 0:
                logger.info("-> [DEADMAN] Waiting for Hold-To-Run engagement on Web UI...")
            return self

        pitch = context.core.pitch

        if abs(pitch) < 10.0:
            logger.info(f"-> Detected Upright ({pitch:.1f}). Transition to BALANCING.")
            return BalancingState()
        elif abs(pitch) > 10.0:
            if self.kickup_attempts < 3:
                logger.info(
                    f"-> Resting ({pitch:.1f}). Transition to KICKUP (Attempt {self.kickup_attempts + 1}/3)."
                )
                return KickupState(attempts=self.kickup_attempts)
            else:
                if ticks % 500 == 0:
                    logger.warning("-> Max Kick-Up attempts reached. Waiting for manual reset.")
        return self


class KickupState(BotState):
    def __init__(self, attempts: int = 0):
        self.attempts = attempts
        self._zero_motion_disabled = MotionRequest(
            velocity=0.0, turn_rate=0.0, enable_control=False
        )
        self._catch_motion_enabled = MotionRequest(
            velocity=0.0, turn_rate=0.0, enable_control=True
        )
        self._zero_tuning = TuningParams(kp=0.0, ki=0.0, kd=0.0, target_angle_offset=0.0)

    def _settle(self, context: AgentContext, timeout: float = 3.0) -> None:
        """Loop with zero motion until the robot rests stably on its bumper."""
        rate = RateLimiter(1.0 / context.config.loop_time)
        dt = context.config.loop_time
        start = time.perf_counter()
        settle_rate_thresh = context.learning_state.control.rest_settle_rate
        stable_count = 0
        min_stable_ticks = 50  # 0.5s at 100Hz

        while time.perf_counter() - start < timeout:
            if context.watchdog:
                context.watchdog.heartbeat()
            telem = context.core.update(self._zero_motion_disabled, self._zero_tuning, dt)
            if abs(telem.pitch_rate) < settle_rate_thresh:
                stable_count += 1
                if stable_count >= min_stable_ticks:
                    break
            else:
                stable_count = 0
            dt = rate.sleep()

    def _attempt_catch(self, context: AgentContext, target_angle: float) -> bool:
        logger.info("-> Attempting Catch...")
        catch_start = time.perf_counter()

        # Conservative gains to prevent backlash slamming; ki=0.0 prevents integral windup
        catch_kp = (
            min(context.learning_state.pid.kp, 15.0) if context.learning_state.pid.kp > 0 else 10.0
        )
        catch_kd = max(context.learning_state.pid.kd, 0.5)
        catch_params = TuningParams(
            kp=catch_kp,
            ki=0.0,
            kd=catch_kd,
            target_angle_offset=0.0,
        )

        rate = RateLimiter(1.0 / context.config.loop_time)
        dt = context.config.loop_time
        stable_frames = 0
        while time.perf_counter() - catch_start < 2.5:
            if context.watchdog:
                context.watchdog.heartbeat()
            telem = context.core.update(self._catch_motion_enabled, catch_params, dt)

            error = abs(telem.pitch_angle - target_angle)
            if error < 5.0 and abs(telem.pitch_rate) < 25.0:
                stable_frames += 1
                if stable_frames >= 5:  # Confirmed stable near vertical
                    logger.info("-> Catch Success (Early equilibrium caught)!")
                    return True
            else:
                stable_frames = 0

            if error > 40.0:
                logger.warning(f"-> Catch Aborted: Overshot/fell (error={error:.1f}° > 40°)")
                return False

            dt = rate.sleep()

        final_error = abs(context.core.pitch - target_angle)
        if final_error < 10.0:
            logger.info("-> Catch Success!")
            return True
        return False

    def _rock_and_flip(self, context: AgentContext, target_angle: float) -> bool:
        ctrl = context.learning_state.control
        start_pitch = context.core.pitch
        kick_direction = Direction.FORWARD if start_pitch < 0 else Direction.BACKWARD

        configured_power = (
            ctrl.kickup_power_forward
            if kick_direction == Direction.FORWARD
            else ctrl.kickup_power_backward
        )
        if configured_power > 0.0:
            amplitude = configured_power
        else:
            amplitude = max(float(context.learning_state.min_power_visible) + 10.0, 15.0)

        max_pulses = ctrl.rock_max_pulses
        amplitude_step = ctrl.rock_amplitude_step
        max_duration = ctrl.rock_pulse_max_duration
        crossover_deg = ctrl.crossover_zone_deg
        min_rate = ctrl.min_carryover_rate

        logger.info(
            f"-> Starting Closed-Loop Rock-and-Flip from pitch={start_pitch:.1f}°. "
            f"Direction={kick_direction.name}, Start Amp={amplitude:.1f}%, Max Pulses={max_pulses}"
        )

        try:
            for pulse_idx in range(max_pulses):
                self._settle(context)

                # Re-evaluate kick direction based on settled pitch
                current_pitch = context.core.pitch
                kick_direction = Direction.FORWARD if current_pitch < 0 else Direction.BACKWARD
                drive_val = amplitude * float(kick_direction.value)

                logger.info(
                    f"-> Rock Pulse #{pulse_idx + 1}/{max_pulses}: "
                    f"Drive={drive_val:.1f}%, Pitch={current_pitch:.1f}°"
                )

                rate = RateLimiter(1.0 / context.config.loop_time)
                dt = context.config.loop_time
                pulse_deadline = time.perf_counter() + max_duration

                while time.perf_counter() < pulse_deadline:
                    if context.watchdog:
                        context.watchdog.heartbeat()
                    telem = context.core.pulse(drive_val, drive_val, dt)

                    # Check if angular rate is moving toward vertical setpoint
                    is_moving_toward_vert = (
                        telem.pitch_rate > 0
                        if kick_direction == Direction.FORWARD
                        else telem.pitch_rate < 0
                    )

                    # Flip trigger: entered crossover zone with sufficient carryover rate
                    if (
                        abs(telem.pitch_angle - target_angle) < crossover_deg
                        and is_moving_toward_vert
                        and abs(telem.pitch_rate) >= min_rate
                    ):
                        logger.info(
                            f"-> Flip Trigger Fired! Pitch={telem.pitch_angle:.1f}°, "
                            f"Rate={telem.pitch_rate:.1f}°/s. Switching to Catch."
                        )
                        context.core.hw.stop()
                        if self._attempt_catch(context, target_angle):
                            return True
                        break  # Catch failed, end this pulse

                    dt = rate.sleep()

                context.core.hw.stop()
                amplitude = min(100.0, amplitude + amplitude_step)

        except Exception as e:
            logger.error(f"Rock-and-Flip Exception: {e}")
            context.core.hw.stop()
            return False

        logger.error("-> Failed Rock-and-Flip: Max pulses exhausted.")
        return False

    def update(
        self,
        context: AgentContext,
        dt: float,
        motion_req: MotionRequest,
        tuning_params: TuningParams,
        last_telemetry: BalanceTelemetry | None,
        ticks: int,
    ) -> BotState:
        success = self._rock_and_flip(context, context.learning_state.pid.target_angle)

        if success:
            logger.info("-> Kick-Up Successful! Transition to BALANCING.")
            return BalancingState()
        else:
            logger.warning("-> Kick-Up Failed. Transition to IDLE.")
            return IdleState(kickup_attempts=self.attempts + 1)


class BalancingState(BotState):
    def __init__(self) -> None:
        super().__init__()
        self.start_time = time.monotonic()

    def update(
        self,
        context: AgentContext,
        dt: float,
        motion_req: MotionRequest,
        tuning_params: TuningParams,
        last_telemetry: BalanceTelemetry | None,
        ticks: int,
    ) -> BotState:
        # Deadman Switch Protection: Disarm immediately if button is released or connection dropped
        if context.deadman_server is not None and not context.deadman_server.is_alive():
            logger.warning("-> [DEADMAN] Switch released during balance. Disarming motors.")
            context.core.hw.stop()
            motion_req.enable_control = False
            return IdleState()

        if (
            context.watchdog is not None
            and context.watchdog.experiment_duration is not None
            and (time.monotonic() - self.start_time > context.watchdog.experiment_duration)
        ):
            logger.info(
                f"-> Experiment Limit ({context.watchdog.experiment_duration:.1f}s) Reached. Halting safely."
            )
            context.core.hw.stop()
            motion_req.enable_control = False
            return FatalErrorState()

        motion_req.enable_control = True
        current_pitch = last_telemetry.pitch_angle if last_telemetry else context.core.pitch

        if abs(current_pitch) > context.learning_state.crash_angle:
            logger.warning(
                f"-> Crash Detected ({current_pitch:.1f} > {context.learning_state.crash_angle}). Transition to CRASHED."
            )
            context.core.hw.stop()
            motion_req.enable_control = False
            return CrashedState()

        elif last_telemetry:
            # Reusing existing tuning_params instead of local agent logic to pass back modifications
            curr_error = last_telemetry.pitch_angle - context.learning_state.pid.target_angle
            adj = context.tuner.update(curr_error)
            if adj.kp != 0 or adj.ki != 0 or adj.kd != 0:
                context.learning_state.pid.kp = max(0.1, context.learning_state.pid.kp + adj.kp)
                context.learning_state.pid.ki = max(0.0, context.learning_state.pid.ki + adj.ki)
                context.learning_state.pid.kd = max(0.0, context.learning_state.pid.kd + adj.kd)
                tuning_params.kp, tuning_params.ki, tuning_params.kd = (
                    context.learning_state.pid.kp,
                    context.learning_state.pid.ki,
                    context.learning_state.pid.kd,
                )

            if motion_req.velocity == 0.0 and motion_req.turn_rate == 0.0:
                aggression = 10.0 if not context.learning_state.balance_verified else 1.0
                effort = last_telemetry.motor_output / context.battery.compensation_factor
                if abs(curr_error) < 5.0 < abs(effort) and abs(last_telemetry.pitch_rate) < 20.0:
                    sign = 1 if effort > 0 else -1
                    context.learning_state.pid.target_angle += sign * (
                        context.config.loop_time * aggression
                    )

        return self


class CrashedState(BotState):
    def __init__(self) -> None:
        self.crash_time = time.monotonic()

    def update(
        self,
        context: AgentContext,
        dt: float,
        motion_req: MotionRequest,
        tuning_params: TuningParams,
        last_telemetry: BalanceTelemetry | None,
        ticks: int,
    ) -> BotState:
        motion_req.enable_control = False
        context.recovery.update(True, context.core.pitch, context.learning_state.pid.kp)

        if time.monotonic() - self.crash_time > 3.0:
            logger.info("-> Crash Timeout Expired. Transition to FATAL ERROR / HALT.")
            return FatalErrorState()
        return self


class FatalErrorState(BotState):
    def update(
        self,
        context: AgentContext,
        dt: float,
        motion_req: MotionRequest,
        tuning_params: TuningParams,
        last_telemetry: BalanceTelemetry | None,
        ticks: int,
    ) -> BotState:
        motion_req.enable_control = False
        if ticks % 200 == 0:
            logger.critical("-> FATAL ERROR STATE. PLEASE MANUALLY RESET ROBOT.")
        return self
