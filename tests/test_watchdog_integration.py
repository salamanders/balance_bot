from unittest.mock import MagicMock, patch
import pytest
from balance_bot.behavior.agent import Agent
from balance_bot.watchdog import SurvivalWatchdog
from balance_bot.hardware.robot_hardware import RobotHardware
import os

# Ensure mocks allowed
os.environ["ALLOW_MOCK_FALLBACK"] = "1"

def test_agent_watchdog_panic(monkeypatch):
    # Patch HW to avoid init errors
    mock_hw = MagicMock(spec=RobotHardware)
    mock_hw.pz = MagicMock()
    mock_hw.init.return_value = None
    MockClass = MagicMock(return_value=mock_hw)
    monkeypatch.setattr("balance_bot.reflex.balance_core.RobotHardware", MockClass)

    watchdog = MagicMock(spec=SurvivalWatchdog)
    # Simulate that the watchdog triggered the interrupt
    watchdog.triggered = True

    agent = Agent(watchdog=watchdog)

    assert agent.watchdog == watchdog

def test_agent_keyboard_interrupt_clean_exit(monkeypatch):
    # Patch HW
    mock_hw = MagicMock(spec=RobotHardware)
    mock_hw.pz = MagicMock()
    mock_hw.init.return_value = None
    # Mock stop specifically
    mock_hw.stop = MagicMock()

    MockClass = MagicMock(return_value=mock_hw)
    monkeypatch.setattr("balance_bot.reflex.balance_core.RobotHardware", MockClass)

    watchdog = MagicMock(spec=SurvivalWatchdog)
    watchdog.triggered = False

    agent = Agent(watchdog=watchdog)

    # Mock the core update to raise KeyboardInterrupt
    # agent.core is already created with our mock_hw
    agent.core.update = MagicMock(side_effect=KeyboardInterrupt)

    # Run should catch KBI and call core.cleanup which calls hw.stop
    agent.run()

    # Verify stop was called on the mock hardware we injected
    mock_hw.stop.assert_called()
