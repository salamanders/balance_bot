import pytest
from balance_bot.configuration import HardwareConfig, PIDParams, ControlConfig

def test_max_tilt_config():
    config = ControlConfig()
    assert config.max_tilt_angle == 10.0
