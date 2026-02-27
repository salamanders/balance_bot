import pytest
from balance_bot.configuration import HardwareConfig, PIDParams, LearningState

def test_pid_params_defaults():
    params = PIDParams()
    assert params.kp == 25.0
    assert params.ki == 0.0
