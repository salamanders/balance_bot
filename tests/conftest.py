from unittest.mock import MagicMock
import pytest
from balance_bot.behavior.agent import Agent, BotState
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import RobotHardware

@pytest.fixture
def mock_hardware_factory():
    def _create():
        hw = MagicMock(spec=RobotHardware)
        hw.pz = MagicMock()
        hw.sensor = MagicMock()
        # Mock methods that are called during init/run
        hw.init.return_value = None
        hw.read_imu_converted.return_value = MagicMock(pitch_angle=0.0, pitch_rate=0.0)
        return hw
    return _create
