import json
import unittest
from unittest.mock import patch, MagicMock
from balance_bot.config import RobotConfig, PIDParams

class TestConfigPersistence(unittest.TestCase):

    @patch("balance_bot.config.CONFIG_FILE")
    def test_load_no_file(self, mock_file):
        """Test load returns default config when file does not exist."""
        mock_file.exists.return_value = False

        config = RobotConfig.load()

        self.assertIsInstance(config, RobotConfig)
        self.assertEqual(config.pid.kp, 25.0) # Default value
        mock_file.read_text.assert_not_called()

    @patch("balance_bot.config.CONFIG_FILE")
    def test_load_valid_json(self, mock_file):
        """Test load returns correct config from valid JSON."""
        mock_file.exists.return_value = True

        # Create a valid JSON structure with non-default values
        test_data = {
            "pid": {"kp": 50.0, "ki": 1.0, "kd": 2.0, "target_angle": 0.0, "integral_limit": 20.0},
            "motor_l": 1,
            "motor_r": 2
        }
        mock_file.read_text.return_value = json.dumps(test_data)

        config = RobotConfig.load()

        self.assertIsInstance(config, RobotConfig)
        self.assertEqual(config.pid.kp, 50.0)
        self.assertEqual(config.pid.ki, 1.0)
        self.assertEqual(config.pid.kd, 2.0)
        self.assertEqual(config.motor_l, 1)
        self.assertEqual(config.motor_r, 2)

    @patch("balance_bot.config.CONFIG_FILE")
    def test_load_empty_file(self, mock_file):
        """Test load returns default config when file is empty."""
        mock_file.exists.return_value = True
        mock_file.read_text.return_value = ""

        config = RobotConfig.load()

        self.assertIsInstance(config, RobotConfig)
        self.assertEqual(config.pid.kp, 25.0) # Default

    @patch("balance_bot.config.CONFIG_FILE")
    @patch("balance_bot.config.logger")
    def test_load_malformed_json(self, mock_logger, mock_file):
        """Test load handles malformed JSON and logs error."""
        mock_file.exists.return_value = True
        mock_file.read_text.return_value = "{ invalid json"

        config = RobotConfig.load()

        self.assertIsInstance(config, RobotConfig)
        self.assertEqual(config.pid.kp, 25.0) # Default
        mock_logger.error.assert_called()

    @patch("balance_bot.config.CONFIG_FILE")
    @patch("balance_bot.config.logger")
    def test_load_missing_required_field(self, mock_logger, mock_file):
        """Test load handles missing required fields (pid) and logs error."""
        mock_file.exists.return_value = True
        # Missing 'pid' which is required
        mock_file.read_text.return_value = '{"battery": {}}'

        config = RobotConfig.load()

        self.assertIsInstance(config, RobotConfig)
        self.assertEqual(config.pid.kp, 25.0) # Default
        mock_logger.error.assert_called()

    @patch("balance_bot.config.CONFIG_FILE")
    def test_save_success(self, mock_file):
        """Test save writes correct JSON to file."""
        config = RobotConfig(pid=PIDParams(kp=12.34))

        config.save()

        mock_file.write_text.assert_called_once()
        args, _ = mock_file.write_text.call_args
        written_content = args[0]

        # Verify written content is valid JSON and matches config
        loaded_data = json.loads(written_content)
        self.assertEqual(loaded_data["pid"]["kp"], 12.34)

    @patch("balance_bot.config.CONFIG_FILE")
    @patch("balance_bot.config.logger")
    def test_save_failure(self, mock_logger, mock_file):
        """Test save handles OSError and logs error."""
        mock_file.write_text.side_effect = OSError("Disk full")

        config = RobotConfig(pid=PIDParams())
        config.save()

        mock_logger.error.assert_called()
        self.assertIn("Error saving config", mock_logger.error.call_args[0][0])

if __name__ == "__main__":
    unittest.main()
