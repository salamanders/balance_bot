import pytest
from balance_bot.configuration import HardwareConfig, PIDParams

def test_imu_yaw_roll_config():
    config = HardwareConfig()
    assert config.gyro_yaw_axis is None
