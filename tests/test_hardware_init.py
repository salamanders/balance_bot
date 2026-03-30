from unittest.mock import MagicMock
import pytest

from balance_bot.discovery.hardware_init import HardwareInitStep
from balance_bot.discovery.step import StepStatus
from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.configuration import HardwareConfig, LearningState


@pytest.fixture()  # type: ignore[untyped-decorator]
def step() -> HardwareInitStep:
    return HardwareInitStep()


@pytest.fixture()  # type: ignore[untyped-decorator]
def mock_hw() -> MagicMock:
    hw = MagicMock(spec=RobotHardware)
    # By default, pretend hardware initialized successfully
    hw.pz = MagicMock()
    hw.sensor = MagicMock()
    return hw


@pytest.fixture()  # type: ignore[untyped-decorator]
def config() -> HardwareConfig:
    return HardwareConfig()


@pytest.fixture()  # type: ignore[untyped-decorator]
def state() -> LearningState:
    return LearningState()


def test_hardware_init_step_name(step: HardwareInitStep) -> None:
    assert step.name == "Initialize Hardware Drivers"


def test_hardware_init_step_is_verified(step: HardwareInitStep, state: LearningState) -> None:
    state.hardware_init_verified = False
    assert not step.is_verified(state)

    state.hardware_init_verified = True
    assert step.is_verified(state)


def test_hardware_init_step_success(step: HardwareInitStep, mock_hw: MagicMock, config: HardwareConfig, state: LearningState) -> None:
    status, config_updates, state_updates = step.run(mock_hw, config, state)

    mock_hw.initialize_drivers.assert_called_once()
    mock_hw.init.assert_called_once()

    assert status == StepStatus.SUCCESS
    assert config_updates == {}
    assert state_updates == {'hardware_init_verified': True}


def test_hardware_init_step_initialize_drivers_exception(step: HardwareInitStep, mock_hw: MagicMock, config: HardwareConfig, state: LearningState) -> None:
    mock_hw.initialize_drivers.side_effect = Exception("Mock init error")

    status, config_updates, state_updates = step.run(mock_hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    mock_hw.init.assert_not_called()


def test_hardware_init_step_pz_none(step: HardwareInitStep, mock_hw: MagicMock, config: HardwareConfig, state: LearningState) -> None:
    mock_hw.pz = None

    status, config_updates, state_updates = step.run(mock_hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    mock_hw.init.assert_not_called()


def test_hardware_init_step_sensor_none(step: HardwareInitStep, mock_hw: MagicMock, config: HardwareConfig, state: LearningState) -> None:
    mock_hw.sensor = None

    status, config_updates, state_updates = step.run(mock_hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
    mock_hw.init.assert_not_called()


def test_hardware_init_step_init_exception(step: HardwareInitStep, mock_hw: MagicMock, config: HardwareConfig, state: LearningState) -> None:
    mock_hw.init.side_effect = Exception("Mock motor init error")

    status, config_updates, state_updates = step.run(mock_hw, config, state)

    assert status == StepStatus.FATAL
    assert config_updates == {}
    assert state_updates == {}
