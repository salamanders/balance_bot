import sys
from unittest.mock import MagicMock, patch

# Mock smbus2 and mpu6050 before importing anything else
sys.modules["smbus2"] = MagicMock()
sys.modules["mpu6050"] = MagicMock()

import json # noqa: E402
from balance_bot.behavior.agent import Agent # noqa: E402
from balance_bot.configuration import HardwareConfig, LearningState, PIDParams # noqa: E402

@patch("balance_bot.behavior.agent.BalanceCore")
@patch("balance_bot.behavior.agent.LedController")
@patch("balance_bot.behavior.agent.setup_logging")
def test_save_config_worker_writes_file(_mock_logging, _mock_led, _mock_core):
    """Verify that _save_config_worker correctly serializes and writes the config."""
    # Setup
    with patch("balance_bot.behavior.agent.LEARNING_STATE_FILE") as mock_file:
        agent = Agent()
        # Mocking the initialization that happens in __init__
        agent.hw_config = HardwareConfig()
        agent.learning_state = LearningState(pid=PIDParams(kp=10.0))

        config_data = {"test": "data", "pid": {"kp": 10.0}}

        # Execute
        agent._save_config_worker(config_data)

        # Verify
        mock_file.write_text.assert_called_once()
        args, _ = mock_file.write_text.call_args
        written_content = args[0]

        # Check if it is valid JSON and matches input
        assert json.loads(written_content) == config_data

@patch("balance_bot.behavior.agent.BalanceCore")
@patch("balance_bot.behavior.agent.LedController")
@patch("balance_bot.behavior.agent.setup_logging")
def test_save_config_worker_handles_exception(_mock_logging, _mock_led, _mock_core):
    """Verify that exceptions during save are logged and don't crash."""
    with patch("balance_bot.behavior.agent.LEARNING_STATE_FILE") as mock_file, \
         patch("balance_bot.behavior.agent.logger") as mock_logger:

        mock_file.write_text.side_effect = OSError("Disk full")

        agent = Agent()
        agent._save_config_worker({"test": "data"})

        mock_logger.error.assert_called_once()
        # The error message is formatted as f"Error saving config asynchronously: {e}"
        # We check if the exception message is in the logged error
        args, _ = mock_logger.error.call_args
        assert "Disk full" in args[0]
