import json
from typing import Any

import pytest

from balance_bot.configuration import ControlConfig, LearningState


def test_control_config_mutable() -> None:
    """Verify ControlConfig is no longer frozen."""
    cfg = ControlConfig()
    try:
        cfg.kickup_power_forward = 50.0
    except Exception as e:
        pytest.fail(f"ControlConfig should be mutable, but raised: {e}")
    assert cfg.kickup_power_forward == 50.0


def test_kickup_power_default() -> None:
    """Verify default kickup power and rock-and-flip parameters."""
    cfg = ControlConfig()
    assert cfg.kickup_power_forward == 0.0
    assert cfg.kickup_power_backward == 0.0
    assert cfg.rock_amplitude_step == 2.0
    assert cfg.rock_pulse_max_duration == 0.40
    assert cfg.rock_max_pulses == 6
    assert cfg.crossover_zone_deg == 15.0
    assert cfg.min_carryover_rate == 40.0
    assert cfg.rest_settle_rate == 5.0


def test_kickup_power_persistence(tmp_path: Any, monkeypatch: Any) -> None:
    """Verify kickup power and rock-and-flip params are saved and loaded correctly."""
    monkeypatch.chdir(tmp_path)

    # create config with custom kickup power
    cfg = LearningState.load()  # defaults
    cfg.control.kickup_power_forward = 45.5
    cfg.control.kickup_power_backward = 70.0
    cfg.control.rock_max_pulses = 8
    cfg.control.crossover_zone_deg = 18.0

    cfg.save()

    # Reload
    new_cfg = LearningState.load()
    assert new_cfg.control.kickup_power_forward == 45.5
    assert new_cfg.control.kickup_power_backward == 70.0
    assert new_cfg.control.rock_max_pulses == 8
    assert new_cfg.control.crossover_zone_deg == 18.0

    # Verify JSON content
    with open("learning_state.json") as f:
        data = json.load(f)

    assert "control" in data
    assert data["control"]["kickup_power_forward"] == 45.5
    assert data["control"]["kickup_power_backward"] == 70.0
    assert data["control"]["rock_max_pulses"] == 8
    assert data["control"]["crossover_zone_deg"] == 18.0
