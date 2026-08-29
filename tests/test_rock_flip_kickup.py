from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

from balance_bot.behavior.states import AgentContext, BalancingState, IdleState, KickupState
from balance_bot.configuration import ControlConfig, HardwareConfig, LearningState
from balance_bot.reflex.balance_core import BalanceTelemetry


class MockCore:
    def __init__(self, hw_config: HardwareConfig, learning_state: LearningState) -> None:
        self.hw_config = hw_config
        self.learning_state = learning_state
        self.pitch = -20.0  # Resting backward on bumper
        self.pulse_calls: list[tuple[float, float, float]] = []
        self.update_calls: list[dict[str, Any]] = []
        self.hw = MagicMock()
        self.trajectory_generator: Any = None

    def pulse(
        self, left_pwm: float, right_pwm: float, loop_delta_time: float
    ) -> BalanceTelemetry:
        self.pulse_calls.append((left_pwm, right_pwm, loop_delta_time))
        if self.trajectory_generator:
            self.pitch, pitch_rate = next(self.trajectory_generator)
        else:
            self.pitch = -20.0
            pitch_rate = 0.0

        return BalanceTelemetry(
            pitch_angle=self.pitch,
            pitch_rate=pitch_rate,
            yaw_rate=0.0,
            error_count=0,
            motor_output=0.0,
            crashed=False,
            left_pwm=left_pwm,
            right_pwm=right_pwm,
            target_angle=self.learning_state.pid.target_angle,
        )

    def update(
        self, motion: Any, tuning: Any, loop_delta_time: float, battery_compensation: float = 1.0
    ) -> BalanceTelemetry:
        self.update_calls.append(
            {"enable_control": motion.enable_control, "tuning": tuning, "dt": loop_delta_time}
        )
        pitch_rate = 0.0
        if self.trajectory_generator:
            try:
                self.pitch, pitch_rate = next(self.trajectory_generator)
            except StopIteration:
                pass

        return BalanceTelemetry(
            pitch_angle=self.pitch,
            pitch_rate=pitch_rate,
            yaw_rate=0.0,
            error_count=0,
            motor_output=0.0,
            crashed=False,
            left_pwm=0.0,
            right_pwm=0.0,
            target_angle=self.learning_state.pid.target_angle,
        )


def _trajectory_insufficient_power() -> Any:
    """Robot budges slightly then stays near bumper."""
    while True:
        yield (-19.0, 1.0)


def _trajectory_successful_flip() -> Any:
    """Robot starts at -20, accelerates up through crossover zone into vertical stability."""
    # Settle ticks (settle needs 50 ticks < 5.0 deg/s)
    for _ in range(60):
        yield (-20.0, 0.0)
    # Pulse begins: pitch moves up rapidly
    yield (-18.0, 20.0)
    yield (-14.0, 45.0)  # Enters crossover zone (<15 deg) with rate 45.0 > 40.0
    # Catch phase begins: settles near 0 deg
    for _ in range(10):
        yield (0.5, 2.0)
    while True:
        yield (0.0, 0.0)


def test_rock_flip_prevents_f0_pulse_collapse() -> None:
    """Verify pulse spans multiple consecutive ticks without motor shutoff (Regression test for F0)."""
    hw_config = HardwareConfig(loop_time=0.01)
    learning_state = LearningState(
        control=ControlConfig(
            rock_max_pulses=1,
            rock_pulse_max_duration=0.05,  # 5 ticks
            min_carryover_rate=100.0,  # Ensure no early flip trigger
        )
    )

    core = MockCore(hw_config, learning_state)
    core.trajectory_generator = _trajectory_insufficient_power()

    watchdog = MagicMock()
    context = AgentContext(
        core=core,  # type: ignore[arg-type]
        config=hw_config,
        learning_state=learning_state,
        led=MagicMock(),
        battery=MagicMock(),
        recovery=MagicMock(),
        tuner=MagicMock(),
        watchdog=watchdog,
    )

    state = KickupState(attempts=0)
    next_state = state.update(
        context=context,
        dt=0.01,
        motion_req=MagicMock(),
        tuning_params=MagicMock(),
        last_telemetry=None,
        ticks=0,
    )

    # Must have pulsed multiple ticks continuously (at 100Hz, 0.05s >= 5 ticks)
    assert len(core.pulse_calls) >= 4
    # Motors must have received non-zero drive
    for left, right, _ in core.pulse_calls:
        assert left > 0.0
        assert right > 0.0

    # Failed after 1 pulse, transition to IdleState(attempts=1)
    assert isinstance(next_state, IdleState)
    assert next_state.kickup_attempts == 1


def test_rock_flip_successful_catch_transition() -> None:
    """Verify real-time flip detection triggers catch and transitions to BalancingState."""
    hw_config = HardwareConfig(loop_time=0.01)
    learning_state = LearningState(
        control=ControlConfig(
            rock_max_pulses=3,
            rock_pulse_max_duration=0.40,
            crossover_zone_deg=15.0,
            min_carryover_rate=40.0,
        )
    )

    core = MockCore(hw_config, learning_state)
    core.trajectory_generator = _trajectory_successful_flip()

    watchdog = MagicMock()
    context = AgentContext(
        core=core,  # type: ignore[arg-type]
        config=hw_config,
        learning_state=learning_state,
        led=MagicMock(),
        battery=MagicMock(),
        recovery=MagicMock(),
        tuner=MagicMock(),
        watchdog=watchdog,
    )

    state = KickupState(attempts=0)
    next_state = state.update(
        context=context,
        dt=0.01,
        motion_req=MagicMock(),
        tuning_params=MagicMock(),
        last_telemetry=None,
        ticks=0,
    )

    # Flip trigger should have fired, motors stopped, and catch succeeded
    assert isinstance(next_state, BalancingState)
    # Watchdog heartbeats were called
    assert watchdog.heartbeat.call_count > 0


def test_rock_flip_amplitude_progression() -> None:
    """Verify pulse amplitude grows by rock_amplitude_step on failed attempts."""
    hw_config = HardwareConfig(loop_time=0.01)
    learning_state = LearningState(
        control=ControlConfig(
            kickup_power_forward=20.0,
            rock_amplitude_step=5.0,
            rock_max_pulses=3,
            rock_pulse_max_duration=0.03,  # 3 ticks per pulse
            min_carryover_rate=100.0,  # prevent catch
        )
    )

    core = MockCore(hw_config, learning_state)
    core.trajectory_generator = _trajectory_insufficient_power()

    context = AgentContext(
        core=core,  # type: ignore[arg-type]
        config=hw_config,
        learning_state=learning_state,
        led=MagicMock(),
        battery=MagicMock(),
        recovery=MagicMock(),
        tuner=MagicMock(),
        watchdog=None,
    )

    state = KickupState(attempts=0)
    next_state = state.update(
        context=context,
        dt=0.01,
        motion_req=MagicMock(),
        tuning_params=MagicMock(),
        last_telemetry=None,
        ticks=0,
    )

    assert isinstance(next_state, IdleState)
    # Pulses started at 20.0, then 25.0, then 30.0
    pulse_amplitudes = [left for left, _, _ in core.pulse_calls]
    assert 20.0 in pulse_amplitudes
    assert 25.0 in pulse_amplitudes
    assert 30.0 in pulse_amplitudes
