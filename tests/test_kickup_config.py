import pytest
from balance_bot.configuration import HardwareConfig, ControlConfig, LearningState

def test_kickup_config_defaults():
    config = ControlConfig()
    assert config.kickup_power_forward == 0.0
