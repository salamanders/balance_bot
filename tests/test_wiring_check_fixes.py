import unittest
from unittest.mock import MagicMock, patch

from balance_bot.wiring_check import WiringCheck
from balance_bot.configuration import LearningState, HardwareConfig

class TestWiringCheckFixes(unittest.TestCase):

    def setUp(self):
        # Mock dependencies
        self.mock_hw = MagicMock()
        self.mock_watchdog = MagicMock()

        # Setup WiringCheck instance
        self.wc = WiringCheck(watchdog=self.mock_watchdog)
        self.wc.hw = self.mock_hw

        # Mock Configs
        self.wc.learning_state = MagicMock(spec=LearningState)
        # Default min_power_visible to avoid errors
        self.wc.learning_state.min_power_visible = 20
        self.wc.hw_config = MagicMock(spec=HardwareConfig)

    @patch("sys.exit", side_effect=SystemExit)
    def test_verify_final_configuration_death_loop_fix_straight(self, mock_exit):
        """
        Verify that verify_final_configuration clears learning flags on Straight Drive failure.
        """
        # Simulate Drive Straight Failure (High Yaw Rate)
        # drive_and_measure returns a result object
        mock_result = MagicMock()
        mock_result.abs_avg_yaw_rate = 50.0 # > 40.0 Threshold
        mock_result.avg_yaw_rate = 50.0 # Also set for logging

        self.wc.hw.drive_and_measure.return_value = mock_result

        # Use patch.object to spy on _update_learning_state
        with patch.object(self.wc, '_update_learning_state') as mock_update:
             with self.assertRaises(SystemExit):
                self.wc.verify_final_configuration()

             # Check that sys.exit(1) was called
             mock_exit.assert_called_with(1)

             # Check that flags were reset
             mock_update.assert_called_with(
                 motor_direction_verified=False,
                 motor_channels_verified=False,
                 motor_trim_verified=False
             )

    @patch("sys.exit", side_effect=SystemExit)
    def test_verify_final_configuration_death_loop_fix_turn_wrong_way(self, mock_exit):
        """
        Verify that verify_final_configuration clears learning flags on Turn Right failure (Wrong Way).
        """
        # 1. Drive Straight Pass
        mock_res_straight = MagicMock()
        mock_res_straight.abs_avg_yaw_rate = 0.0
        mock_res_straight.avg_yaw_rate = 0.0

        # 2. Turn Right Fail (Wrong Direction - Positive Yaw instead of Negative)
        mock_res_turn = MagicMock()
        mock_res_turn.avg_yaw_rate = 20.0 # > -10.0 (Failed) and > 10.0 (Wrong Way)
        mock_res_turn.abs_avg_yaw_rate = 20.0

        self.wc.hw.drive_and_measure.side_effect = [mock_res_straight, mock_res_turn]

        with patch.object(self.wc, '_update_learning_state') as mock_update:
            with self.assertRaises(SystemExit):
                self.wc.verify_final_configuration()

            mock_update.assert_called_with(
                 motor_direction_verified=False,
                 motor_channels_verified=False,
                 motor_trim_verified=False
            )
            mock_exit.assert_called_with(1)

    @patch("sys.exit", side_effect=SystemExit)
    def test_verify_final_configuration_death_loop_fix_turn_no_movement(self, mock_exit):
        """
        Verify that verify_final_configuration clears learning flags on Turn Right failure (No Movement).
        """
        # 1. Drive Straight Pass
        mock_res_straight = MagicMock()
        mock_res_straight.abs_avg_yaw_rate = 0.0
        mock_res_straight.avg_yaw_rate = 0.0

        # 2. Turn Right Fail (No significant turn)
        mock_res_turn = MagicMock()
        mock_res_turn.avg_yaw_rate = 0.0 # > -10.0 (Failed) but < 10.0
        mock_res_turn.abs_avg_yaw_rate = 0.0

        self.wc.hw.drive_and_measure.side_effect = [mock_res_straight, mock_res_turn]

        with patch.object(self.wc, '_update_learning_state') as mock_update:
            with self.assertRaises(SystemExit):
                self.wc.verify_final_configuration()

            mock_update.assert_called_with(
                 motor_direction_verified=False,
                 motor_channels_verified=False,
                 motor_trim_verified=False
            )
            mock_exit.assert_called_with(1)

if __name__ == "__main__":
    unittest.main()
