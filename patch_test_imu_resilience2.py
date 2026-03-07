with open('tests/test_imu_resilience.py', 'w') as f:
    f.write("""import pytest
from unittest.mock import MagicMock
from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.configuration import HardwareConfig, LearningState, PIDParams

def test_imu_resilience_fail_fast(monkeypatch):
    \"\"\"
    Test that read_imu_raw implements an explicit fail-fast strategy for transient I2C errors
    as per the new rules (no silent stale data returns).
    \"\"\"
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")

    hw_config = HardwareConfig(motor_l=0, motor_r=1)
    learning_state = LearningState(pid=PIDParams())

    hw = RobotHardware(hw_config, learning_state)
    hw.sensor = MagicMock()

    hw.sensor.get_accel_data.side_effect = OSError("Input/output error")
    hw.sensor.get_gyro_data.side_effect = OSError("Input/output error")

    with pytest.raises(OSError):
        hw.read_imu_raw()
""")
