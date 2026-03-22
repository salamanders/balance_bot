from unittest.mock import MagicMock
import pytest

from balance_bot.discovery.hardware_init import HardwareInitStep
from balance_bot.discovery.step import StepStatus
from balance_bot.configuration import HardwareConfig, LearningState


@pytest.fixture
def hardware_init_step():
    return HardwareInitStep()


def test_hardware_init_success(hardware_init_step):
    """Test successful hardware initialization."""
    hw_mock = MagicMock()
    # Ensure they are not None to pass the alive check
    hw_mock.pz = MagicMock()
    hw_mock.sensor = MagicMock()

    status, config_updates, state_updates = hardware_init_step.run(
        hw_mock, HardwareConfig(), LearningState()
    )

    assert status == StepStatus.SUCCESS
    assert config_updates == {}
    assert state_updates == {'hardware_init_verified': True}
    hw_mock.initialize_drivers.assert_called_once()
    hw_mock.init.assert_called_once()


def test_hardware_init_drivers_exception(hardware_init_step):
    """Test failure when initialize_drivers raises an exception."""
    hw_mock = MagicMock()
    hw_mock.initialize_drivers.side_effect = Exception("I2C Bus Error")

    status, config_updates, state_updates = hardware_init_step.run(
        hw_mock, HardwareConfig(), LearningState()
    )

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    hw_mock.initialize_drivers.assert_called_once()
    hw_mock.init.assert_not_called()


def test_hardware_init_drivers_none_pz(hardware_init_step):
    """Test failure when drivers fail to initialize (pz is None)."""
    hw_mock = MagicMock()
    hw_mock.pz = None
    hw_mock.sensor = MagicMock()

    status, config_updates, state_updates = hardware_init_step.run(
        hw_mock, HardwareConfig(), LearningState()
    )

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    hw_mock.initialize_drivers.assert_called_once()
    hw_mock.init.assert_not_called()


def test_hardware_init_drivers_none_sensor(hardware_init_step):
    """Test failure when drivers fail to initialize (sensor is None)."""
    hw_mock = MagicMock()
    hw_mock.pz = MagicMock()
    hw_mock.sensor = None

    status, config_updates, state_updates = hardware_init_step.run(
        hw_mock, HardwareConfig(), LearningState()
    )

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    hw_mock.initialize_drivers.assert_called_once()
    hw_mock.init.assert_not_called()


def test_hardware_init_motor_init_exception(hardware_init_step):
    """Test failure when motor driver init raises an exception."""
    hw_mock = MagicMock()
    hw_mock.pz = MagicMock()
    hw_mock.sensor = MagicMock()
    hw_mock.init.side_effect = Exception("Motor Driver Init Error")

    status, config_updates, state_updates = hardware_init_step.run(
        hw_mock, HardwareConfig(), LearningState()
    )

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    hw_mock.initialize_drivers.assert_called_once()
    hw_mock.init.assert_called_once()


def test_is_verified_true(hardware_init_step):
    state = LearningState()
    state.hardware_init_verified = True
    assert hardware_init_step.is_verified(state) is True


def test_is_verified_false(hardware_init_step):
    state = LearningState()
    state.hardware_init_verified = False
    assert hardware_init_step.is_verified(state) is False

def test_hardware_init_name(hardware_init_step):
    assert hardware_init_step.name == "Initialize Hardware Drivers"
