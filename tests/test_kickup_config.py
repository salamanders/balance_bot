import json
from balance_bot.config import RobotConfig, ControlConfig

def test_control_config_mutable():
    """Verify ControlConfig is no longer frozen."""
    cfg = ControlConfig()
    try:
        cfg.kickup_power = 50.0
    except Exception as e:
        assert False, f"ControlConfig should be mutable, but raised: {e}"
    assert cfg.kickup_power == 50.0

def test_kickup_power_default():
    """Verify default kickup power is 60.0."""
    cfg = ControlConfig()
    assert cfg.kickup_power == 60.0

def test_kickup_power_persistence(tmp_path, monkeypatch):
    """Verify kickup power is saved and loaded correctly."""
    monkeypatch.chdir(tmp_path)

    # create config with custom kickup power
    cfg = RobotConfig.load() # defaults
    cfg.control.kickup_power = 45.5

    cfg.save()

    # Reload
    new_cfg = RobotConfig.load()
    assert new_cfg.control.kickup_power == 45.5

    # Verify JSON content
    with open("pid_config.json", "r") as f:
        data = json.load(f)

    assert "control" in data
    assert data["control"]["kickup_power"] == 45.5
