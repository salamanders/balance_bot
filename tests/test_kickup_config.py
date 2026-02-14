import json
from balance_bot.config import RobotConfig, ControlConfig

def test_control_config_mutable():
    """Verify ControlConfig is no longer frozen."""
    cfg = ControlConfig()
    try:
        cfg.kickup_power_forward = 50.0
    except Exception as e:
        assert False, f"ControlConfig should be mutable, but raised: {e}"
    assert cfg.kickup_power_forward == 50.0

def test_kickup_power_default():
    """Verify default kickup power is 0.0."""
    cfg = ControlConfig()
    assert cfg.kickup_power_forward == 0.0
    assert cfg.kickup_power_backward == 0.0

def test_kickup_power_persistence(tmp_path, monkeypatch):
    """Verify kickup power is saved and loaded correctly."""
    monkeypatch.chdir(tmp_path)

    # create config with custom kickup power
    cfg = RobotConfig.load() # defaults
    cfg.control.kickup_power_forward = 45.5
    cfg.control.kickup_power_backward = 70.0

    cfg.save()

    # Reload
    new_cfg = RobotConfig.load()
    assert new_cfg.control.kickup_power_forward == 45.5
    assert new_cfg.control.kickup_power_backward == 70.0

    # Verify JSON content
    with open("pid_config.json", "r") as f:
        data = json.load(f)

    assert "control" in data
    assert data["control"]["kickup_power_forward"] == 45.5
    assert data["control"]["kickup_power_backward"] == 70.0
