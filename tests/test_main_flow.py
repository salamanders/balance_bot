import sys
from unittest.mock import MagicMock, patch

# Ensure import
sys.path.insert(0, ".")

from balance_bot.main import main

@patch("balance_bot.main.setup_logging")
@patch("balance_bot.main.SurvivalWatchdog")
@patch("balance_bot.main.JulesClient")
@patch("balance_bot.main.Agent")
@patch("balance_bot.configuration.LearningState")
@patch("balance_bot.configuration.HardwareConfig")
@patch("balance_bot.discovery.pipeline.SelfDiscoveryPipeline")
def test_main_toddler_phase(mock_pipeline, mock_hw, mock_state, mock_agent, mock_jules, mock_watchdog, mock_log):
    """Test that the main loop runs Pipeline if needs_discovery is True."""

    # Mock State
    mock_state_inst = MagicMock()
    mock_state_inst.backlash_verified = False
    mock_state_inst.control.kickup_power_forward = 0.0
    mock_state.load.return_value = mock_state_inst

    # Run
    with patch("sys.argv", ["main.py"]):
        main()

    # Verify
    mock_pipeline.assert_called_once()
    mock_pipeline.return_value.run.assert_called_once()
    mock_agent.assert_called_once()

@patch("balance_bot.main.setup_logging")
@patch("balance_bot.main.SurvivalWatchdog")
@patch("balance_bot.main.JulesClient")
@patch("balance_bot.main.Agent")
@patch("balance_bot.configuration.LearningState")
@patch("balance_bot.configuration.HardwareConfig")
@patch("balance_bot.discovery.pipeline.SelfDiscoveryPipeline")
def test_main_adult_phase(mock_pipeline, mock_hw, mock_state, mock_agent, mock_jules, mock_watchdog, mock_log):
    """Test that main skips Pipeline if verified."""

    # Mock State
    mock_state_inst = MagicMock()
    mock_state_inst.backlash_verified = True
    mock_state_inst.control.kickup_power_forward = 50.0
    mock_state.load.return_value = mock_state_inst

    # Run
    with patch("sys.argv", ["main.py"]):
        main()

    # Verify
    mock_pipeline.assert_not_called()
    mock_agent.assert_called_once()
