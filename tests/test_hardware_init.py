from unittest.mock import MagicMock

from balance_bot.discovery.hardware_init import HardwareInitStep
from balance_bot.discovery.step import StepStatus
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import RobotHardware


def test_name() -> None:
    step = HardwareInitStep()
    assert step.name == "Initialize Hardware Drivers"


def test_is_verified() -> None:
    step = HardwareInitStep()
    state = LearningState()

    state.hardware_init_verified = False
    assert step.is_verified(state) is False

    state.hardware_init_verified = True
    assert step.is_verified(state) is True


def test_run_success() -> None:
    step = HardwareInitStep()
    hw = MagicMock(spec=RobotHardware)
    config = HardwareConfig()
    state = LearningState()

    # Setup mocks to succeed
    hw.pz = MagicMock()
    hw.sensor = MagicMock()

    status, config_updates, state_updates = step.run(hw, config, state)

    assert status == StepStatus.SUCCESS
    assert config_updates == {}
    assert state_updates == {'hardware_init_verified': True}

    hw.initialize_drivers.assert_called_once()
    hw.init.assert_called_once()


def test_run_initialize_drivers_exception() -> None:
    step = HardwareInitStep()
    hw = MagicMock(spec=RobotHardware)
    config = HardwareConfig()
    state = LearningState()

    hw.initialize_drivers.side_effect = Exception("Mock exception")

    status, config_updates, state_updates = step.run(hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}


def test_run_drivers_none() -> None:
    step = HardwareInitStep()
    hw = MagicMock(spec=RobotHardware)
    config = HardwareConfig()
    state = LearningState()

    # Mock initialize_drivers to succeed, but drivers are still None
    hw.pz = None
    hw.sensor = None

    status, config_updates, state_updates = step.run(hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    hw.init.assert_not_called()


def test_run_drivers_pz_none() -> None:
    step = HardwareInitStep()
    hw = MagicMock(spec=RobotHardware)
    config = HardwareConfig()
    state = LearningState()

    # Mock initialize_drivers to succeed, but drivers are still None
    hw.pz = None
    hw.sensor = MagicMock()

    status, config_updates, state_updates = step.run(hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    hw.init.assert_not_called()


def test_run_drivers_sensor_none() -> None:
    step = HardwareInitStep()
    hw = MagicMock(spec=RobotHardware)
    config = HardwareConfig()
    state = LearningState()

    # Mock initialize_drivers to succeed, but drivers are still None
    hw.pz = MagicMock()
    hw.sensor = None

    status, config_updates, state_updates = step.run(hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    hw.init.assert_not_called()


def test_run_init_exception() -> None:
    step = HardwareInitStep()
    hw = MagicMock(spec=RobotHardware)
    config = HardwareConfig()
    state = LearningState()

    hw.pz = MagicMock()
    hw.sensor = MagicMock()
    hw.init.side_effect = Exception("Mock exception")

    status, config_updates, state_updates = step.run(hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    hw.initialize_drivers.assert_called_once()
    hw.init.assert_called_once()
