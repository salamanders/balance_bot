import logging
import time
from dataclasses import dataclass
from typing import Any

from ..adaptation.battery import BatteryEstimator
from ..adaptation.recovery import RecoveryManager
from ..adaptation.tuner import ContinuousTuner
from ..behavior.leds import LedController
from ..configuration import HardwareConfig, LearningState
from ..enums import Direction, Orientation
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
        self._zero_motion_enabled = MotionRequest(velocity=0.0, turn_rate=0.0, enable_control=False)
        self._catch_motion_enabled = MotionRequest(velocity=0.0, turn_rate=0.0, enable_control=True)
        self._zero_tuning = TuningParams(kp=0.0, ki=0.0, kd=0.0, target_angle_offset=0.0)

    def _wait_for_settle(
        self, context: AgentContext, duration: float = 1.0, rate_threshold: float = 10.0
    ) -> None:
        end_time = time.perf_counter() + duration
        rate = RateLimiter(1.0 / context.config.loop_time)
        dt = context.config.loop_time
        while True:
            if context.watchdog:
                context.watchdog.heartbeat()
            telemetry = context.core.update(self._zero_motion_enabled, self._zero_tuning, dt)

            if time.perf_counter() > end_time:
                if abs(telemetry.pitch_rate) < rate_threshold:
                    break
                else:
                    end_time = time.perf_counter() + 0.5
            dt = rate.sleep()

    def _sleep_with_update(self, context: AgentContext, duration: float) -> None:
        end_time = time.perf_counter() + duration
        rate = RateLimiter(1.0 / context.config.loop_time)
        dt = context.config.loop_time
        while time.perf_counter() < end_time:
            if context.watchdog:
                context.watchdog.heartbeat()
            context.core.update(self._zero_motion_enabled, self._zero_tuning, dt)
            dt = rate.sleep()

    def _check_and_fix_position(
        self, context: AgentContext, kick_direction: Direction, start_label: str
    ) -> bool:
        # No hardcoded repositioning loop. The robot attempts kick-up directly from its resting posture.
        return True

    def _attempt_catch(self, context: AgentContext, target_angle: float) -> bool:
        logger.info("-> Attempting Catch...")
        catch_start = time.perf_counter()

        catch_params = TuningParams(
            kp=context.learning_state.pid.kp * 1.5,
            ki=context.learning_state.pid.ki,
            kd=context.learning_state.pid.kd * 2.0,
            target_angle_offset=0.0,
        )

        rate = RateLimiter(1.0 / context.config.loop_time)
        dt = context.config.loop_time
        while time.perf_counter() - catch_start < 2.5:
            if context.watchdog:
                context.watchdog.heartbeat()
            telem = context.core.update(self._catch_motion_enabled, catch_params, dt)

            error = abs(telem.pitch_angle - target_angle)
            if error < 5.0 and abs(telem.pitch_rate) < 30.0:
                pass
            if abs(telem.pitch_angle - target_angle) > 40.0:
                pass
            dt = rate.sleep()

        final_error = abs(context.core.pitch - target_angle)
        if final_error < 10.0:
            logger.info("-> Catch Success!")
            return True
        return False

    def _incremental_kickup(
        self, context: AgentContext, target_angle: float, start_power: float
    ) -> bool:
        power = start_power
        step = 2.0
        max_power = 100.0

        start_pitch = context.core.pitch
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
                if context.watchdog:
                    context.watchdog.heartbeat()
                self._wait_for_settle(context)

                if not self._check_and_fix_position(context, kick_direction, start_label):
                    return False

                logger.info(f"-> Kick-Up Attempt: Power {power:.1f} Direction {kick_direction}")

                drive_val = float(power) * float(kick_direction.value)
                context.core.hw.set_motors(drive_val, drive_val)
                self._sleep_with_update(context, 0.25)

                if self._attempt_catch(context, target_angle):
                    return True

                context.core.hw.stop()
                logger.info("-> Catch Failed. Retrying...")
                power += step

        except Exception as e:
            logger.error(f"Kick-Up Exception: {e}")
            context.core.hw.stop()
            return False

        logger.error("-> Failed to Kick-Up (Max Power Reached).")
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
        pwr = (
            context.learning_state.control.kickup_power_forward
            if context.core.pitch < 0
            else context.learning_state.control.kickup_power_backward
        )
        success = self._incremental_kickup(
            context, context.learning_state.pid.target_angle, start_power=pwr
        )

        if success:
            logger.info("-> Kick-Up Successful! Transition to BALANCING.")
            return BalancingState()
        else:
            if (
                context.core.pitch > 80.0
            ):  # Fallback FATAL check for completely unrecoverable orientation
                pass  # FATAL checks handled in next state or here
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

        if time.monotonic() - self.start_time > 4.0:
            logger.info("-> 4-Second Experiment Limit Reached. Halting safely.")
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
