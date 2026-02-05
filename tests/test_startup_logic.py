import unittest
from unittest.mock import MagicMock, patch
import sys

# Adjust path to import src
sys.path.insert(0, "src")

from balance_bot.behavior.agent import Agent

class TestAgentStartup(unittest.TestCase):

    def setUp(self):
        # Patch CONFIG_FILE
        self.config_patcher = patch("balance_bot.behavior.agent.CONFIG_FILE")
        self.mock_config_file = self.config_patcher.start()

        # Patch BalanceCore
        self.core_patcher = patch("balance_bot.behavior.agent.BalanceCore")
        self.mock_core_cls = self.core_patcher.start()
        self.mock_core_instance = self.mock_core_cls.return_value
        self.mock_core_instance.update.return_value = MagicMock()

        # Patch RobotConfig
        self.robot_config_patcher = patch("balance_bot.behavior.agent.RobotConfig")
        self.mock_robot_config_cls = self.robot_config_patcher.start()

        # Patch ContinuousTuner
        self.tuner_patcher = patch("balance_bot.behavior.agent.ContinuousTuner")
        self.mock_tuner_cls = self.tuner_patcher.start()
        self.mock_tuner_instance = self.mock_tuner_cls.return_value
        self.mock_tuner_instance.get_current_scale.return_value = 1.0

        # Patch others to avoid side effects
        patch("balance_bot.behavior.agent.setup_logging").start()
        patch("balance_bot.behavior.agent.LedController").start()
        patch("balance_bot.behavior.agent.BalancePointFinder").start()
        patch("balance_bot.behavior.agent.BatteryEstimator").start()
        patch("balance_bot.behavior.agent.RecoveryManager").start()

        # Configure common mocks
        self.mock_config_file.exists.return_value = True

        self.mock_config_instance = MagicMock()
        self.mock_config_instance.pid = MagicMock()
        # Ensure we have target_angle
        self.mock_config_instance.pid.target_angle = 0.0
        self.mock_config_instance.loop_time = 0.01
        self.mock_robot_config_cls.load.return_value = self.mock_config_instance
        self.mock_robot_config_cls.return_value = self.mock_config_instance

    def tearDown(self):
        patch.stopall()

    def test_first_run_triggers_discovery(self):
        # Arrange
        self.mock_config_file.exists.return_value = False # First run

        agent = Agent()
        # Ensure first_run is True (logic in __init__)
        self.assertTrue(agent.first_run)

        # Mock specific methods to verify calls and avoid real logic
        agent._perform_discovery = MagicMock()
        agent.running = False # Skip main loop

        # Act
        agent.run()

        # Assert
        agent._perform_discovery.assert_called_once()

    def test_normal_run_on_back_triggers_kickup(self):
        # Arrange
        self.mock_config_file.exists.return_value = True # Saved config

        agent = Agent()
        self.assertFalse(agent.first_run)

        # Mock pitch to be "On Back" (-40.0)
        self.mock_core_instance.pitch = -40.0

        agent._incremental_kickup = MagicMock()
        agent.running = False

        # Act
        agent.run()

        # Assert
        agent._incremental_kickup.assert_called_once()

    def test_normal_run_upright_skips_kickup(self):
        # Arrange
        self.mock_config_file.exists.return_value = True

        agent = Agent()
        self.assertFalse(agent.first_run)

        # Mock pitch to be "Upright" (0.0)
        self.mock_core_instance.pitch = 0.0

        agent._incremental_kickup = MagicMock()
        agent.running = False

        # Act
        agent.run()

        # Assert
        agent._incremental_kickup.assert_not_called()

if __name__ == "__main__":
    unittest.main()
