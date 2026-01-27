import json
import pytest
from balance_bot.config import RobotConfig, PIDParams

# Legacy JSON sample
LEGACY_JSON = """
{
    "pid_kp": 12.5,
    "pid_ki": 0.1,
    "pid_kd": 0.5,
    "target_angle": -2.0,
    "motor_l": 0,
    "motor_r": 1,
    "motor_l_invert": false,
    "motor_r_invert": true,
    "gyro_pitch_axis": "x",
    "gyro_pitch_invert": false
}
"""

NESTED_JSON = """
{
    "pid": {
        "kp": 15.0,
        "ki": 0.2,
        "kd": 0.6,
        "target_angle": 1.0,
        "integral_limit": 25.0
    },
    "motor_l": 1,
    "motor_r": 0,
    "motor_l_invert": true,
    "motor_r_invert": false,
    "gyro_pitch_axis": "y",
    "gyro_pitch_invert": true,
    "loop_time": 0.02
}
"""

def test_load_legacy_config(tmp_path, monkeypatch):
    config_file = tmp_path / "pid_config.json"
    config_file.write_text(LEGACY_JSON)

    # Mock CONFIG_FILE path in config module
    # Since we can't easily change the constant in the module, we can mock open?
    # Or simpler, we can change working directory?
    # Let's try mocking os.path.exists and open.
    # Actually, simpler is to just parse the logic, but we want to test the load method.

    # Let's change the working directory to tmp_path
    monkeypatch.chdir(tmp_path)

    config = RobotConfig.load()

    assert config.pid.kp == 12.5
    assert config.pid.ki == 0.1
    assert config.pid.kd == 0.5
    assert config.pid.target_angle == -2.0
    assert config.motor_r_invert is True
    assert config.pid.integral_limit == 20.0 # Default value

def test_load_nested_config(tmp_path, monkeypatch):
    config_file = tmp_path / "pid_config.json"
    config_file.write_text(NESTED_JSON)
    monkeypatch.chdir(tmp_path)

    config = RobotConfig.load()

    assert config.pid.kp == 15.0
    assert config.pid.target_angle == 1.0
    assert config.pid.integral_limit == 25.0
    assert config.gyro_pitch_axis == "y"
    assert config.loop_time == 0.02

def test_save_nested_config(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    pid = PIDParams(kp=10.0, integral_limit=30.0)
    config = RobotConfig(pid=pid, motor_l=5)

    config.save()

    with open("pid_config.json", "r") as f:
        data = json.load(f)

    assert "pid" in data
    assert data["pid"]["kp"] == 10.0
    assert data["pid"]["integral_limit"] == 30.0
    assert data["motor_l"] == 5
