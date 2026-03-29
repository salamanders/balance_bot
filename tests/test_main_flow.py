from typing import Any
import sys
from unittest.mock import MagicMock, patch

# Ensure import
sys.path.insert(0, "src")

from balance_bot.main import main

@patch("balance_bot.main.setup_logging")
@patch("balance_bot.main.SurvivalWatchdog")
@patch("balance_bot.main.JulesClient")
@patch("balance_bot.main.Agent")
@patch("balance_bot.configuration.LearningState")
@patch("balance_bot.configuration.HardwareConfig")
@patch("balance_bot.discovery.pipeline.SelfDiscoveryPipeline")
def test_main_toddler_phase(mock_pipeline: Any, _mock_hw: Any, mock_state: Any, mock_agent: Any, _mock_jules: Any, _mock_watchdog: Any, _mock_log: Any) -> None:
    """Test that the main loop runs Pipeline if needs_discovery is True."""

    # Mock State
    mock_state_inst = MagicMock()
    mock_state_inst.kickup_dynamics_verified = False
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
def test_main_adult_phase(mock_pipeline: Any, _mock_hw: Any, mock_state: Any, mock_agent: Any, _mock_jules: Any, _mock_watchdog: Any, _mock_log: Any) -> None:
    """Test that main skips Pipeline if verified."""

    # Mock State
    mock_state_inst = MagicMock()
    mock_state_inst.kickup_dynamics_verified = True
    mock_state.load.return_value = mock_state_inst

    # Run
    with patch("sys.argv", ["main.py"]):
        main()

    # Verify
    mock_pipeline.assert_not_called()
    mock_agent.assert_called_once()

@patch("balance_bot.main.setup_logging")
@patch("balance_bot.main.SurvivalWatchdog")
@patch("balance_bot.main.JulesClient")
@patch("balance_bot.main.Agent")
@patch("balance_bot.configuration.LearningState")
@patch("balance_bot.configuration.HardwareConfig")
@patch("balance_bot.discovery.pipeline.SelfDiscoveryPipeline")
@patch("builtins.open", side_effect=Exception("Disk full"))
@patch("builtins.print")
def test_main_crash_reporting_local_write_failure(
    mock_print: Any, _mock_open: Any, _mock_pipeline: Any, _mock_hw: Any, mock_state: Any, mock_agent: Any, mock_jules: Any, _mock_watchdog: Any, _mock_log: Any
) -> Any:
    """Test that main reports local fallback write failure when Jules upload fails."""

    # Trigger a crash during init
    mock_agent.side_effect = Exception("Fatal Init Error")

    # Mock State (needs discovery so we don't skip to adult phase, though it crashes anyway)
    mock_state_inst = MagicMock()
    mock_state_inst.kickup_dynamics_verified = True
    mock_state.load.return_value = mock_state_inst

    # Mock JulesClient to fail upload
    mock_jules_inst = MagicMock()
    mock_jules_inst.report_crash.return_value = (False, "fallback markdown content")
    mock_jules.return_value = mock_jules_inst

    # Run with auto-fix enabled to trigger crash reporting
    with patch("sys.argv", ["main.py", "--auto-fix"]):
        try:
            main()
        except Exception as e:
            assert str(e) == "Fatal Init Error"

    # Verify that local file write failure was caught and printed
    mock_print.assert_any_call("Failed to write local crash report: Disk full")
