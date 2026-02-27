import pytest
from balance_bot.configuration import HardwareConfig, PIDParams, LearningState

def test_gyro_drift_config():
    config = LearningState()
    assert config.gyro_bias_x == 0.0
