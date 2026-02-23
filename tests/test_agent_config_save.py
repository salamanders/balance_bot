import sys
from unittest.mock import MagicMock, patch

# Mock smbus2 and mpu6050 before importing anything else
sys.modules["smbus2"] = MagicMock()
sys.modules["mpu6050"] = MagicMock()

import json
from balance_bot.behavior.agent import Agent
from balance_bot.configuration import HardwareConfig, LearningState

@patch("balance_bot.reflex.balance_core.BalanceCore")
@patch("balance_bot.behavior.agent.LedController")
@patch("balance_bot.behavior.agent.setup_logging")
def test_save_config_worker_writes_file(mock_logging, mock_led, mock_core):
    """Verify that _save_config_worker correctly serializes and writes the config."""
    # Setup
    with patch("balance_bot.behavior.agent.LEARNING_STATE_FILE") as mock_file:
        # Patch load methods to avoid file access
        with patch("balance_bot.configuration.HardwareConfig.load"), \
             patch("balance_bot.configuration.LearningState.load"):
            agent = Agent()

        config_data = {"kp": 10.0, "ki": 0.0, "kd": 0.0, "target_angle": 0.0, "balance_verified": False}

        # Execute
        agent._save_config_worker(config_data)

        # Verify
        mock_file.write_text.assert_called_once()
        args, _ = mock_file.write_text.call_args
        written_content = args[0]

        # Check if it is valid JSON and matches input
        assert json.loads(written_content) == config_data

@patch("balance_bot.reflex.balance_core.BalanceCore")
@patch("balance_bot.behavior.agent.LedController")
@patch("balance_bot.behavior.agent.setup_logging")
def test_save_config_worker_handles_exception(mock_logging, mock_led, mock_core):
    """Verify that exceptions during save are logged and don't crash."""
    with patch("balance_bot.behavior.agent.LEARNING_STATE_FILE") as mock_file, \
         patch("balance_bot.behavior.agent.logger") as mock_logger:

        # Patch load
        with patch("balance_bot.configuration.HardwareConfig.load"), \
             patch("balance_bot.configuration.LearningState.load"):
            agent = Agent()

        mock_file.write_text.side_effect = OSError("Disk full")

        agent._save_config_worker({"test": "data"})

        mock_logger.error.assert_called_once()
        args, _ = mock_logger.error.call_args
        assert "Disk full" in args[0]
