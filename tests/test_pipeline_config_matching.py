from typing import Any
from unittest.mock import MagicMock

from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.discovery.discover_buses import DiscoverBusesStep
from balance_bot.discovery.hardware_init import HardwareInitStep
from balance_bot.discovery.step import StepStatus
from balance_bot.discovery.toddler_engine import ProprioceptiveToddlerStep

# Mocks and helper structures to simulate the steps


def assert_updates_valid(config_updates: dict[str, Any], state_updates: dict[str, Any]) -> None:
    # Ensure keys match model schema
    hw_fields = set(HardwareConfig.model_fields.keys())
    ls_fields = set(LearningState.model_fields.keys())

    for key in config_updates:
        assert key in hw_fields, f"Config update key '{key}' not found in HardwareConfig fields!"

    for key in state_updates:
        assert key in ls_fields, f"State update key '{key}' not found in LearningState fields!"

    # Also verify that pydantic doesn't throw on update
    HardwareConfig.model_validate(HardwareConfig().model_dump() | config_updates)

    dummy_state = LearningState()
    for k, v in state_updates.items():
        setattr(dummy_state, k, v)


def test_discover_buses_step(monkeypatch: Any) -> None:
    monkeypatch.setattr("balance_bot.discovery.discover_buses.scan_i2c", lambda _name, _check: 1)

    step = DiscoverBusesStep()
    status, config_updates, state_updates = step.run(MagicMock(), HardwareConfig(), LearningState())
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)


def test_hardware_init_step() -> None:
    hw_mock = MagicMock()
    hw_mock.pz = MagicMock()
    hw_mock.sensor = MagicMock()

    step = HardwareInitStep()
    status, config_updates, state_updates = step.run(hw_mock, HardwareConfig(), LearningState())
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)


def test_proprioceptive_toddler_step(monkeypatch: Any) -> None:
    step = ProprioceptiveToddlerStep()
    monkeypatch.setattr(step, "_find_single_motor_breakaway", lambda hw, rec, motor_idx, label: 7.0)
    monkeypatch.setattr(
        step,
        "_derive_kinematics_and_polarity",
        lambda hw, rec, test_pwm, label: (
            {
                "accel_vertical_axis": "z",
                "accel_vertical_invert": False,
                "gyro_pitch_axis": "x",
                "gyro_pitch_invert": False,
                "gyro_yaw_axis": "z",
                "gyro_yaw_invert": False,
                "accel_forward_axis": "y",
                "accel_forward_invert": False,
                "motor_r_invert": False,
            },
            False,
        ),
    )
    monkeypatch.setattr(step, "_sample_settled_pitch", lambda hw, rec, stage: 12.0)
    monkeypatch.setattr(
        step, "_run_symmetric_rock_and_check", lambda hw, rec, start_pwm, step, label: (20.0, -25.0)
    )

    status, config_updates, state_updates = step.run(MagicMock(), HardwareConfig(), LearningState())
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)
