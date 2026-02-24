import sys
import pytest
from unittest.mock import MagicMock, patch

# Ensure import
sys.path.insert(0, ".")

from src.balance_bot.main import main

@patch("src.balance_bot.main.setup_logging")
@patch("src.balance_bot.main.SurvivalWatchdog")
@patch("src.balance_bot.main.JulesClient")
@patch("src.balance_bot.main.Agent")
@patch("src.balance_bot.configuration.LearningState")
@patch("src.balance_bot.configuration.HardwareConfig")
@patch("src.balance_bot.discovery.pipeline.SelfDiscoveryPipeline")
def test_main_toddler_phase(MockPipeline, MockHW, MockState, MockAgent, MockJules, MockWatchdog, MockLog):
    """Test that main runs Pipeline if needs_discovery is True."""

    # Mock State
    mock_state_inst = MagicMock()
    mock_state_inst.backlash_verified = False
    mock_state_inst.control.kickup_power_forward = 0.0
    MockState.load.return_value = mock_state_inst

    # Run
    with patch("sys.argv", ["main.py"]):
        main()

    # Verify
    MockPipeline.assert_called_once()
    MockPipeline.return_value.run.assert_called_once()
    MockAgent.assert_called_once()

@patch("src.balance_bot.main.setup_logging")
@patch("src.balance_bot.main.SurvivalWatchdog")
@patch("src.balance_bot.main.JulesClient")
@patch("src.balance_bot.main.Agent")
@patch("src.balance_bot.configuration.LearningState")
@patch("src.balance_bot.configuration.HardwareConfig")
@patch("src.balance_bot.discovery.pipeline.SelfDiscoveryPipeline")
def test_main_adult_phase(MockPipeline, MockHW, MockState, MockAgent, MockJules, MockWatchdog, MockLog):
    """Test that main skips Pipeline if verified."""

    # Mock State
    mock_state_inst = MagicMock()
    mock_state_inst.backlash_verified = True
    mock_state_inst.control.kickup_power_forward = 50.0
    MockState.load.return_value = mock_state_inst

    # Run
    with patch("sys.argv", ["main.py"]):
        main()

    # Verify
    MockPipeline.assert_not_called()
    MockAgent.assert_called_once()
