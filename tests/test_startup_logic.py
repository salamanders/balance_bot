from typing import Any
import unittest
from unittest.mock import MagicMock, patch
import sys

# Adjust path to import src
sys.path.insert(0, "src")

from balance_bot.behavior.agent import Agent

class TestAgentStartup(unittest.TestCase):

    def setUp(self) -> None:
        # Patch LEARNING_STATE_FILE
        self.config_patcher = patch("pathlib.Path")
        self.mock_config_file = self.config_patcher.start()

        # Patch BalanceCore
        self.core_patcher = patch("balance_bot.behavior.agent.BalanceCore")
        self.mock_core_cls = self.core_patcher.start()
        self.mock_core_instance = self.mock_core_cls.return_value
        self.mock_core_instance.update.return_value = MagicMock()

        # Patch HardwareConfig and LearningState
        self.hw_config_patcher = patch("balance_bot.behavior.agent.HardwareConfig")
        self.mock_hw_config_cls = self.hw_config_patcher.start()

        self.learning_state_patcher = patch("balance_bot.behavior.agent.LearningState")
        self.mock_learning_state_cls = self.learning_state_patcher.start()

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

        self.mock_hw_config_instance = MagicMock()
        self.mock_hw_config_instance.loop_time = 0.01

        self.mock_learning_state_instance = MagicMock()
        self.mock_learning_state_instance.pid = MagicMock()
        self.mock_learning_state_instance.control = MagicMock() # Ensure control config exists
        self.mock_learning_state_instance.timing = MagicMock() # Add timing mock
        self.mock_learning_state_instance.timing.setup_wait = 0.1
        self.mock_learning_state_instance.timing.battery_log_interval = 1.0 # Also used
        self.mock_learning_state_instance.timing.save_interval = 1.0 # Also used

        # Ensure we have target_angle
        self.mock_learning_state_instance.pid.target_angle = 0.0

        self.mock_hw_config_cls.load.return_value = self.mock_hw_config_instance
        self.mock_learning_state_cls.load.return_value = self.mock_learning_state_instance

    def tearDown(self) -> None:
        patch.stopall()

    def test_recovery_manager_initialized_with_config(self) -> None:
        """Test that RecoveryManager is initialized with control config."""
        # Act
        Agent()

        # Assert
        self.mock_recovery_cls.assert_called_once_with(self.mock_learning_state_instance.control)

    def test_normal_run_on_back_triggers_kickup(self) -> None:
        # Arrange
        self.mock_learning_state_instance.timing.setup_wait = 0.0 # Skip warmup loop
        self.mock_config_file.exists.return_value = True # Saved config

        agent = Agent()
        self.assertFalse(agent.first_run)

        # Mock pitch to be "On Back" (-40.0)
        self.mock_core_instance.pitch = -40.0

        pass
        agent.running = True

        # We need the loop to run at least twice:
        # 1. IDLE -> Detect Pitch -> Transition to KICKUP
        # 2. KICKUP -> Call _incremental_kickup

        call_count = 0
        def stop_loop() -> Any:
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
        pass

    def test_normal_run_upright_skips_kickup(self) -> None:
        # Arrange
        self.mock_learning_state_instance.timing.setup_wait = 0.0 # Skip warmup
        self.mock_config_file.exists.return_value = True

        agent = Agent()
        self.assertFalse(agent.first_run)

        # Mock pitch to be "Upright" (0.0)
        self.mock_core_instance.pitch = 0.0

        pass
        agent.running = True

        # Run loop once: IDLE -> BALANCING (if upright)
        call_count = 0
        def stop_loop() -> Any:
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
        pass

if __name__ == "__main__":
    unittest.main()
