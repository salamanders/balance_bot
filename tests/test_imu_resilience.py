import pytest
from balance_bot.configuration import HardwareConfig, PIDParams

def test_imu_resilience_config():
    config = HardwareConfig()
    assert config.imu_max_retries == 5
