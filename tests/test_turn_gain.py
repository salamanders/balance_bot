import pytest
from balance_bot.configuration import HardwareConfig, PIDParams, ControlConfig

def test_turn_gain_config():
    config = ControlConfig()
    assert config.turn_gain == 30.0
