import unittest
from unittest.mock import MagicMock, patch, call
import sys
import math

# Ensure src is in path
sys.path.append("src")

from balance_bot.utils import Vector3
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import IMUReading, MeasureResult

class TestWiringAutonomy(unittest.TestCase):

    def setUp(self):
        # Mock dependencies
        self.mock_hw = MagicMock()
        self.mock_watchdog = MagicMock()

        # Create WiringCheck instance with mocks
        with patch("balance_bot.wiring_check.HardwareConfig.load") as mock_hw_load, \
             patch("balance_bot.wiring_check.LearningState.load") as mock_ls_load:

            mock_hw_load.return_value = HardwareConfig(motor_i2c_bus=1, imu_i2c_bus=1)
            # Create a real LearningState but mock save
            self.ls = LearningState()
            self.ls.min_power_visible = 20
            # Bypass Pydantic __setattr__ to mock method on instance
            object.__setattr__(self.ls, 'save', MagicMock())
            mock_ls_load.return_value = self.ls

            from balance_bot.wiring_check import WiringCheck
            self.wc = WiringCheck(watchdog=self.mock_watchdog)
            self.wc.hw = self.mock_hw

    def _create_measure_result(self, accel_vec=None, pitch=0.0):
        reading = MagicMock(spec=IMUReading)
        if accel_vec:
            reading.accel_raw = accel_vec
        reading.pitch_angle = pitch
        reading.pitch_rate = 0.0
        reading.yaw_rate = 0.0
        reading.roll_rate = 0.0
        return MeasureResult(duration=0.5, samples=[reading])

    def test_blind_flop_success_positive(self):
        """Test Blind Flop where +60 moves Back -> Front."""
        # P1: Back Vector
        p1 = Vector3(0, -1, 0) # Arbitrary Back
        # P2: Front Vector (Different)
        p2 = Vector3(0, 1, 0)  # Arbitrary Front (Angle 180)

        # Setup mocks
        # _measure_gravity_with_hardware calls drive_and_measure(0,0).
        # We need to distinguish calls.
        # But _measure_gravity_with_hardware just calls hw.drive_and_measure(0,0,1.0)

        # Sequence of calls to drive_and_measure:
        # 1. Measure P1 (0,0) -> Returns P1
        # 2. Flop (+60) -> Returns something (ignored)
        # 3. Measure P2 (0,0) -> Returns P2

        # We assume _measure_gravity_with_hardware logic handles extraction from MeasureResult.
        # We can mock _measure_gravity_with_hardware directly to simplify.

        with patch.object(self.wc, '_measure_gravity_with_hardware') as mock_measure:
            mock_measure.side_effect = [p1, p2]

            ret_p1, ret_p2 = self.wc._measure_gravity_vectors()

            self.assertEqual(ret_p1, p1)
            self.assertEqual(ret_p2, p2)

            # Verify Flop Drive
            # min_power=20, flop adds 30 -> 50
            self.mock_hw.drive_and_measure.assert_called_with(50, 50, 0.5)

    def test_blind_flop_fail_positive_success_negative(self):
        """Test Blind Flop where +60 fails, -60 moves."""
        p1 = Vector3(0, -1, 0)
        p2 = Vector3(0, -0.9, 0.1) # Similar to P1 (Angle small)
        p3 = Vector3(0, 1, 0)      # Different (Front)

        with patch.object(self.wc, '_measure_gravity_with_hardware') as mock_measure:
            mock_measure.side_effect = [p1, p2, p3]

            ret_p3, ret_p2 = self.wc._measure_gravity_vectors()

            # Should return (P3, P2) -> (Back, Front).
            # Logic: P3=Back, P2=Front (implied start position/failed move position).
            # Wait, my implementation returns (p3, p2).
            # If +60 failed (stayed P2~=P1), and -60 moved to P3.
            # My logic: "Assuming P2=FRONT, P3=BACK".
            # So returns (P3, P2).

            self.assertEqual(ret_p3, p3) # Back
            self.assertEqual(ret_p2, p2) # Front

            # Verify Calls
            # 1. Flop +50
            # 2. Flop -50
            calls = [call(50, 50, 0.5), call(-50, -50, 0.5)]
            self.mock_hw.drive_and_measure.assert_has_calls(calls, any_order=True)

    def test_determine_motor_direction_flop_needed(self):
        """Test Phase 4 needing a flop to Front."""
        # Setup Config with Axes (required for verification)
        from balance_bot.enums import Axis
        self.wc.hw_config = self.wc.hw_config.model_copy(update={
            'accel_forward_axis': Axis.Y,
            'accel_forward_invert': False,
            'accel_vertical_axis': Axis.Z,
            'gyro_pitch_axis': Axis.X
        })

        # Initial Pitch: -20 (Back)
        # After Flop +: -20 (Fail)
        # After Flop -: +20 (Success/Front)

        # Mock read_imu_converted results
        # 1. Initial check
        r1 = MagicMock(pitch_angle=-20.0)
        # 2. Check after +Flop
        r2 = MagicMock(pitch_angle=-20.0)
        # 3. Check after -Flop (inside if block)
        # Wait, if +Flop fails, we check r2. Then we drive -Flop.
        # Then we exit if block.
        # Then we re-read start_pitch.
        r3 = MagicMock(pitch_angle=20.0)

        self.mock_hw.read_imu_converted.side_effect = [r1, r2, r3, r3] # r3 repeated for "Kick Up" test logic

        # Mock drive_and_measure for Kick Up test
        # It calls _drive_and_wait -> drive_and_measure
        # We need realistic accel vector for verification (Forward Axis=Y)
        res_kick = self._create_measure_result(accel_vec=Vector3(0, 0.2, 0), pitch=10.0)
        self.mock_hw.drive_and_measure.return_value = res_kick

        # Patch input() to ensure we don't block (though we removed it, double check)
        with patch("builtins.print"):
            self.wc.determine_motor_direction()

            # Verify Flop calls
            # 1. +Test (+50)
            # 2. -Test (-50)
            # 3. Kick Up (+40)

            # min_power = 20. Flop = +30 -> 50.
            # Kick Up = +20 -> 40.

            # Note: order matters.
            # 1. drive(50, 50)
            # 2. drive(-50, -50)
            # 3. drive(40, 40)

            expected_calls = [
                call(50, 50, 0.5),
                call(-50, -50, 0.5),
                call(40, 40, 0.4)
            ]
            self.mock_hw.drive_and_measure.assert_has_calls(expected_calls)

    def test_force_posture_success_immediate(self):
        """Test _force_posture returns immediately if correct."""
        self.mock_hw.read_imu_converted.return_value = MagicMock(pitch_angle=20.0)

        with patch("builtins.print"):
            self.wc._force_posture("FRONT")

        self.mock_hw.drive_and_measure.assert_not_called()

    def test_force_posture_flop_to_front(self):
        """Test _force_posture uses Negative power to go to FRONT."""
        # Initial: -20 (Back)
        # Drive 1: -30 (Fail)
        # Read 2: -20
        # Drive 2: -40 (Success)
        # Read 3: 20

        r_fail = MagicMock(pitch_angle=-20.0)
        r_success = MagicMock(pitch_angle=20.0)
        self.mock_hw.read_imu_converted.side_effect = [r_fail, r_fail, r_success]

        with patch("builtins.print"):
            self.wc._force_posture("FRONT", max_attempts=3)

        # Expect Negative Power calls
        # Base = 20 + 10 = 30.
        # Attempt 0: 30 -> -30.
        # Attempt 1: 40 -> -40.
        calls = [call(-30, -30, 0.4), call(-40, -40, 0.4)]
        self.mock_hw.drive_and_measure.assert_has_calls(calls)

    def test_force_posture_flop_to_back(self):
        """Test _force_posture uses Positive power to go to BACK."""
        # Initial: 20 (Front)
        # Target: BACK

        r_fail = MagicMock(pitch_angle=20.0)
        r_success = MagicMock(pitch_angle=-20.0)
        self.mock_hw.read_imu_converted.side_effect = [r_fail, r_success]

        with patch("builtins.print"):
            self.wc._force_posture("BACK", max_attempts=3)

        # Expect Positive Power calls
        # Base = 30.
        # Attempt 0: 30 -> +30.
        calls = [call(30, 30, 0.4)]
        self.mock_hw.drive_and_measure.assert_has_calls(calls)

    def test_force_posture_failure_exit(self):
        """Test _force_posture exits if max attempts reached."""
        r_fail = MagicMock(pitch_angle=-20.0)
        self.mock_hw.read_imu_converted.return_value = r_fail

        with patch("builtins.print"), patch("sys.exit") as mock_exit:
            self.wc._force_posture("FRONT", max_attempts=2)

            mock_exit.assert_called_with(1)
            self.assertEqual(self.mock_hw.drive_and_measure.call_count, 2)

    def test_deduce_left_right_swaps_polarity_flags(self):
        """
        Expect failure: verify that motor_l_invert and motor_r_invert are passed to _update_hw_config.
        """
        # Set Up Config specifically for this test
        self.wc.hw_config = self.wc.hw_config.model_copy(update={
            'motor_l': 0,
            'motor_r': 1,
            'motor_l_invert': True,
            'motor_r_invert': False,
            'gyro_yaw_invert': False
        })
        # Mock Axes
        from balance_bot.enums import Axis
        self.wc.hw_config = self.wc.hw_config.model_copy(update={
            'gyro_yaw_axis': Axis.Z
        })

        # Mock read_imu_raw (Gravity Down)
        # Mocking at RobotHardware level because WiringCheck calls self.hw.read_imu_raw() (in _deduce_left_right_autonomous indirectly?)
        # Actually it calls self.hw.read_imu_raw() in deduce_left_right_autonomous step 1

        self.mock_hw.read_imu_raw.return_value = (Vector3(0, 0, -1), Vector3(0, 0, 0))

        # Mock execute_maneuver (CCW Turn -> Ch0 is Right -> Swap Needed)
        mock_result = MagicMock()
        sample = MagicMock()
        sample.gyro_raw = Vector3(0, 0, 15) # Positive Yaw (Large enough > 10.0)
        mock_result.samples = [sample]
        self.mock_hw.execute_maneuver.return_value = mock_result

        self.mock_hw.wait_for_stability.return_value = None

        with patch.object(self.wc, '_update_hw_config') as mock_update:
            with patch("builtins.print"):
                self.wc.deduce_left_right_autonomous()

            mock_update.assert_called()
            kwargs = mock_update.call_args[1]

            # Assert channels are swapped (Current behavior - should pass)
            self.assertEqual(kwargs.get('motor_l'), 1)
            self.assertEqual(kwargs.get('motor_r'), 0)

            # Assert polarity is swapped (Buggy behavior - should fail)
            # Old L_inv=True, R_inv=False.
            # New L should be False, New R should be True.
            self.assertIn('motor_l_invert', kwargs, "motor_l_invert was not updated!")
            self.assertIn('motor_r_invert', kwargs, "motor_r_invert was not updated!")
            self.assertEqual(kwargs['motor_l_invert'], False)
            self.assertEqual(kwargs['motor_r_invert'], True)

if __name__ == "__main__":
    unittest.main()
