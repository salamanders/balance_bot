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

        # Patch HardwareConfig
        self.robot_config_patcher = patch("balance_bot.behavior.agent.HardwareConfig")
        self.mock_robot_config_cls = self.robot_config_patcher.start()

        # Patch ContinuousTuner
        self.tuner_patcher = patch("balance_bot.behavior.agent.ContinuousTuner")
        self.mock_tuner_cls = self.tuner_patcher.start()
        self.mock_tuner_instance = self.mock_tuner_cls.return_value
        self.mock_tuner_instance.get_current_scale.return_value = 1.0

        # Patch RecoveryManager explicitly so we can verify calls
        self.recovery_patcher = patch("balance_bot.behavior.agent.RecoveryManager")
        self.mock_recovery_cls = self.recovery_patcher.start()

        # Patch others to avoid side effects
        patch("balance_bot.behavior.agent.setup_logging").start()
        patch("balance_bot.behavior.agent.LedController").start()
        patch("balance_bot.behavior.agent.BalancePointFinder").start()
        patch("balance_bot.behavior.agent.BatteryEstimator").start()

        # Configure common mocks
        self.mock_config_file.exists.return_value = True

        self.mock_config_instance = MagicMock()
        self.mock_config_instance.pid = MagicMock()
        self.mock_config_instance.control = MagicMock() # Ensure control config exists
        self.mock_config_instance.timing = MagicMock() # Add timing mock
        self.mock_config_instance.timing.setup_wait = 0.1
        self.mock_config_instance.timing.battery_log_interval = 1.0 # Also used
        self.mock_config_instance.timing.save_interval = 1.0 # Also used

        # Ensure we have target_angle
        self.mock_config_instance.pid.target_angle = 0.0
        self.mock_config_instance.loop_time = 0.01
        self.mock_robot_config_cls.load.return_value = self.mock_config_instance
        self.mock_robot_config_cls.return_value = self.mock_config_instance

    def tearDown(self):
        patch.stopall()

    def test_recovery_manager_initialized_with_config(self):
        """Test that RecoveryManager is initialized with control config."""
        # Act
        agent = Agent()

        # Assert
        self.mock_recovery_cls.assert_called_once_with(self.mock_config_instance.control)

    def test_normal_run_on_back_triggers_kickup(self):
        # Arrange
        self.mock_config_instance.timing.setup_wait = 0.0 # Skip warmup loop
        self.mock_config_file.exists.return_value = True # Saved config

        agent = Agent()
        self.assertFalse(agent.first_run)

        # Mock pitch to be "On Back" (-40.0)
        self.mock_core_instance.pitch = -40.0

        agent._incremental_kickup = MagicMock()
        agent.running = True

        # We need the loop to run at least twice:
        # 1. IDLE -> Detect Pitch -> Transition to KICKUP
        # 2. KICKUP -> Call _incremental_kickup

        call_count = 0
        def stop_loop():
            nonlocal call_count
            call_count += 1
            if call_count >= 2:
                agent.running = False
            return 0.01

        with patch('balance_bot.behavior.agent.RateLimiter') as mock_rate:
            mock_rate.return_value.sleep.side_effect = stop_loop
            # Also mock core.update to return None or dummy telemetry so loop doesn't crash
            agent.core.update.return_value = MagicMock()

            # Act
            agent.run()

        # Assert
        agent._incremental_kickup.assert_called_once()

    def test_normal_run_upright_skips_kickup(self):
        # Arrange
        self.mock_config_instance.timing.setup_wait = 0.0 # Skip warmup
        self.mock_config_file.exists.return_value = True

        agent = Agent()
        self.assertFalse(agent.first_run)

        # Mock pitch to be "Upright" (0.0)
        self.mock_core_instance.pitch = 0.0

        agent._incremental_kickup = MagicMock()
        agent.running = True

        # Run loop once: IDLE -> BALANCING (if upright)
        call_count = 0
        def stop_loop():
            nonlocal call_count
            call_count += 1
            if call_count >= 1:
                agent.running = False
            return 0.01

        with patch('balance_bot.behavior.agent.RateLimiter') as mock_rate:
            mock_rate.return_value.sleep.side_effect = stop_loop
            agent.core.update.return_value = MagicMock()

            # Act
            agent.run()

        # Assert
        agent._incremental_kickup.assert_not_called()

if __name__ == "__main__":
    unittest.main()
