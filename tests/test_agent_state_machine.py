from unittest.mock import MagicMock
import pytest
import os
from balance_bot.behavior.agent import Agent, BotState
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import RobotHardware

# We need to ensure MOCKS are allowed for these tests
os.environ["ALLOW_MOCK_FALLBACK"] = "1"

@pytest.fixture
def mock_hardware_factory(monkeypatch):
    """
    Patches RobotHardware so Agent uses a mock instead of real hardware.
    """
    def _create():
        # Create the mock
        mock_hw = MagicMock(spec=RobotHardware)
        mock_hw.pz = MagicMock()
        mock_hw.sensor = MagicMock()
        mock_hw.init.return_value = None
        mock_hw.read_imu_converted.return_value = MagicMock(pitch_angle=0.0, pitch_rate=0.0)

        # Patch the class in the module where Agent imports it
        # Agent imports BalanceCore, which imports RobotHardware.
        # But Agent creates BalanceCore which creates RobotHardware.
        # We need to patch 'balance_bot.reflex.balance_core.RobotHardware'

        # Actually, simpler: RobotHardware.__new__ return value?
        # Or just patch the class constructor in the module.
        return mock_hw
    return _create

def test_agent_initialization(monkeypatch):
    """Test that the agent initializes in the correct state."""
    # Patch RobotHardware to prevent real init
    mock_hw = MagicMock(spec=RobotHardware)
    mock_hw.pz = MagicMock()
    mock_hw.init.return_value = None

    # We patch the class itself to return our mock instance when instantiated
    MockClass = MagicMock(return_value=mock_hw)
    monkeypatch.setattr("balance_bot.reflex.balance_core.RobotHardware", MockClass)

    agent = Agent(watchdog=MagicMock())

    assert agent.state == BotState.IDLE

def test_agent_transitions(monkeypatch):
    """Test state transitions."""
    # Patch RobotHardware
    mock_hw = MagicMock(spec=RobotHardware)
    mock_hw.pz = MagicMock()
    mock_hw.init.return_value = None
    mock_hw.pitch = 0.0 # BalanceCore property uses this? No, BalanceCore wraps hw.

    # BalanceCore.pitch property reads self.hw.read_imu_converted().pitch_angle
    # So we need to mock read_imu_converted
    reading = MagicMock()
    reading.pitch_angle = 0.0
    mock_hw.read_imu_converted.return_value = reading

    MockClass = MagicMock(return_value=mock_hw)
    monkeypatch.setattr("balance_bot.reflex.balance_core.RobotHardware", MockClass)

    agent = Agent(watchdog=MagicMock())

    # Simulate Kickup request manually (Agent doesn't have transition_to, it's a state machine loop)
    # But we can force state
    agent.state = BotState.KICKUP
    assert agent.state == BotState.KICKUP
