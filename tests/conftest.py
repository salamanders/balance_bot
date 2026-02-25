import pytest
from unittest.mock import MagicMock
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import RobotHardware

@pytest.fixture
def mock_hardware_factory(monkeypatch):
    """
    Fixture that returns a factory function for creating RobotHardware instances
    with mocked sensors and varying configurations.
    """
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")

    def _create_hardware(**hw_config_kwargs):
        # Default safe config if not specified
        defaults = {
            'motor_l': 0,
            'motor_r': 1
        }
        # Update defaults with provided kwargs
        config_args = defaults.copy()
        config_args.update(hw_config_kwargs)

        config = HardwareConfig(**config_args)
        state = LearningState()

        # Initialize Hardware
        hw = RobotHardware(config, state)

        # Inject Mock Sensor
        hw.sensor = MagicMock()

        return hw

    return _create_hardware
