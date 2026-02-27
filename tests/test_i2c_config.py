import pytest
from balance_bot.configuration import HardwareConfig, PIDParams

def test_i2c_config_defaults():
    config = HardwareConfig()
    assert config.motor_i2c_bus is None
