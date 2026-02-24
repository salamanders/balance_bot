import unittest
from unittest.mock import MagicMock, patch
import sys

# We need to ensure we can import from src
sys.path.insert(0, ".")

from src.balance_bot.main import main
from src.balance_bot.configuration import LearningState, HardwareConfig

class TestMainFlow(unittest.TestCase):

    @patch('src.balance_bot.main.setup_logging')
    @patch('src.balance_bot.main.SurvivalWatchdog')
    @patch('src.balance_bot.main.JulesClient')
    @patch('src.balance_bot.main.get_captured_logs')
    @patch('src.balance_bot.main.WiringCheck')
    @patch('src.balance_bot.main.Agent')
    @patch('src.balance_bot.configuration.LearningState')
    @patch('src.balance_bot.configuration.HardwareConfig')
    def test_main_toddler_phase(self, MockHardwareConfig, MockLearningState, MockAgent, MockWiringCheck, MockGetLogs, MockJules, MockWatchdog, MockSetupLogging):
        """Test that main runs WiringCheck if needs_discovery is True."""

        # Setup mocks
        mock_state = MagicMock()
        mock_state.backlash_verified = False # Incomplete
        mock_state.control.kickup_power_forward = 0.0
        MockLearningState.load.return_value = mock_state

        mock_wc_instance = MockWiringCheck.return_value
        mock_agent_instance = MockAgent.return_value

        # Run main with no args
        with patch('sys.argv', ['main.py']):
            main()

        # Assertions
        MockLearningState.load.assert_called()
        MockWiringCheck.assert_called()
        mock_wc_instance.run.assert_called_once()
        MockAgent.assert_called() # Should run Agent after discovery
        mock_agent_instance.run.assert_called_once()

    @patch('src.balance_bot.main.setup_logging')
    @patch('src.balance_bot.main.SurvivalWatchdog')
    @patch('src.balance_bot.main.JulesClient')
    @patch('src.balance_bot.main.get_captured_logs')
    @patch('src.balance_bot.main.WiringCheck')
    @patch('src.balance_bot.main.Agent')
    @patch('src.balance_bot.configuration.LearningState')
    @patch('src.balance_bot.configuration.HardwareConfig')
    def test_main_adult_phase(self, MockHardwareConfig, MockLearningState, MockAgent, MockWiringCheck, MockGetLogs, MockJules, MockWatchdog, MockSetupLogging):
        """Test that main skips WiringCheck if already learned."""

        # Setup mocks
        mock_state = MagicMock()
        mock_state.backlash_verified = True
        mock_state.control.kickup_power_forward = 50.0 # Non-zero
        MockLearningState.load.return_value = mock_state

        # Run main with no args
        with patch('sys.argv', ['main.py']):
            main()

        # Assertions
        MockLearningState.load.assert_called()
        MockWiringCheck.assert_not_called() # Should verify this!
        MockAgent.assert_called()
        MockAgent.return_value.run.assert_called_once()

    @patch('src.balance_bot.main.setup_logging')
    @patch('src.balance_bot.main.SurvivalWatchdog')
    @patch('src.balance_bot.main.JulesClient')
    @patch('src.balance_bot.main.get_captured_logs')
    @patch('src.balance_bot.main.WiringCheck')
    @patch('src.balance_bot.main.Agent')
    @patch('src.balance_bot.configuration.LearningState')
    @patch('src.balance_bot.configuration.HardwareConfig')
    def test_main_reset_brain(self, MockHardwareConfig, MockLearningState, MockAgent, MockWiringCheck, MockGetLogs, MockJules, MockWatchdog, MockSetupLogging):
        """Test that --reset-brain wipes config."""

        # Setup mocks
        mock_state = MagicMock()
        mock_state.backlash_verified = False # After reset, it might load default which is false
        # Wait, reset_brain saves new config. Then loads it.
        # But MockLearningState.load() returns mock_state.
        MockLearningState.load.return_value = mock_state

        # Run main with --reset-brain
        with patch('sys.argv', ['main.py', '--reset-brain']):
            main()

        # Assertions
        MockHardwareConfig.return_value.save.assert_called_once()
        MockLearningState.return_value.save.assert_called_once()

        # It should then load state and likely run discovery (since default state is empty)
        MockLearningState.load.assert_called()
        # WiringCheck depends on what load returns. If mock returns False, it runs.
        if not mock_state.backlash_verified:
            MockWiringCheck.assert_called()

if __name__ == '__main__':
    unittest.main()
