import sys
import unittest
from unittest.mock import MagicMock, patch, call
from balance_bot.enums import Axis
import glm

# Mock smbus2 before import
if 'smbus2' not in sys.modules:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.hardware.robot_hardware import MeasureResult, IMUReading

class TestWiringTrainingWheels(unittest.TestCase):

    def setUp(self):
        # Patch dependencies
        self.smbus_patch = patch("balance_bot.utils.smbus")
        self.hw_patch = patch("balance_bot.wiring_check.RobotHardware")
        self.hw_config_patch = patch("balance_bot.wiring_check.HardwareConfig")
        self.learning_state_patch = patch("balance_bot.wiring_check.LearningState")

        self.input_patch = patch("builtins.input")
        self.sleep_patch = patch("time.sleep")

        self.mock_smbus = self.smbus_patch.start()
        self.mock_hw_class = self.hw_patch.start()
        self.mock_hw_config_class = self.hw_config_patch.start()
        self.mock_learning_state_class = self.learning_state_patch.start()
        self.mock_input = self.input_patch.start()
        self.mock_sleep = self.sleep_patch.start()

        # Setup Mocks return values
        self.mock_hw_config = MagicMock()
        self.mock_hw_config_class.load.return_value = self.mock_hw_config
        # Allow model_copy to work
        self.mock_hw_config.model_copy.return_value = self.mock_hw_config

        self.mock_learning_state = MagicMock()
        self.mock_learning_state_class.load.return_value = self.mock_learning_state

        # Setup WiringCheck
        self.wc = WiringCheck()

        # Setup Config Defaults
        self.wc.learning_state.min_power_visible = 20
        self.wc.hw_config.motor_l = 0
        self.wc.hw_config.motor_r = 1
        self.wc.hw_config.motor_l_invert = False
        self.wc.hw_config.motor_r_invert = False

        # Setup Default Axes (Assume configured)
        self.wc.hw_config.accel_forward_axis = Axis.Y
        self.wc.hw_config.accel_forward_invert = False
        self.wc.hw_config.gyro_yaw_axis = Axis.Z
        self.wc.hw_config.gyro_yaw_invert = False
        self.wc.hw_config.gyro_pitch_axis = Axis.X
        self.wc.hw_config.gyro_pitch_invert = False

        # Setup Hardware Mock Instance
        self.hw_mock = self.mock_hw_class.return_value
        self.wc.hw = self.hw_mock

        # Mock get_mapped_value to return 0 by default
        self.hw_mock.get_mapped_value.return_value = 0.0

    def tearDown(self):
        self.smbus_patch.stop()
        self.hw_patch.stop()
        self.hw_config_patch.stop()
        self.learning_state_patch.stop()
        self.input_patch.stop()
        self.sleep_patch.stop()

    def test_direction_accel_change(self):
        """Test 'The Attempt' where Pitch is static but Accel confirms forward motion."""

        # 1. Start Pitch (Leaning Forward, e.g. +30)
        start_reading = MagicMock()
        start_reading.pitch_angle = 30.0

        # 2. End Pitch (Static, still +30)
        # 3. Accel Reading (Significant Forward Accel)
        # We need drive_and_measure to return a result with samples.

        # Samples should have high average forward accel.
        # Forward Axis is Y.
        # Accel Raw needs to have Y component.
        sample1 = IMUReading(
            pitch_angle=30.0,
            roll_angle=0,
            yaw_rate=0,
            pitch_rate=0,
            roll_rate=0,
            accel_raw=glm.vec3(0, 0.5, 0), # Y=0.5 (Forward)
            gyro_raw=glm.vec3(0,0,0)
        )
        sample2 = IMUReading(
            pitch_angle=30.1, # Tiny change
            roll_angle=0,
            yaw_rate=0,
            pitch_rate=0,
            roll_rate=0,
            accel_raw=glm.vec3(0, 0.6, 0), # Y=0.6
            gyro_raw=glm.vec3(0,0,0)
        )

        result = MeasureResult(
            duration=0.3,
            samples=[sample1, sample2]
        )

        # Mock HW responses
        self.hw_mock.read_imu_converted.return_value = start_reading
        self.hw_mock.drive_and_measure.return_value = result

        # Run
        self.wc.determine_motor_direction()

        # Verify Success
        # In current wiring_check.py, determine_motor_direction uses verify_with_retries
        # and updates self.learning_state.motor_direction_verified
        self.assertTrue(self.wc.learning_state.motor_direction_verified)

        # Verify no inversion happened (update_hw_config not called with inverts, or current config matches)
        # Since we mocked hw_config, we check if model_copy was called with invert=True
        # Actually determine_motor_direction calls _update_hw_config only on FAILURE/Inversion.
        # On success it calls _update_learning_state.
        self.assertFalse(self.wc.hw_config.motor_l_invert)

    def test_left_right_arc_ccw(self):
        """Test 'The Arc' where High Power on Right causes Left (CCW) Turn."""

        # Logic:
        # Drive Ch0 (assumed Left) at High Power
        # Drive Ch1 (assumed Right) at Low Power
        # Result: If Ch0 IS Left, it drives faster -> Right Turn (CW).
        # Result: If Ch0 IS Right, it drives faster -> Left Turn (CCW).

        # We simulate CCW Turn (Positive Yaw).
        # This implies Ch0 is actually Right.
        # Expect Swap.

        # Mock Up Vector (Gravity down on Z)
        self.hw_mock.read_imu_raw.return_value = (glm.vec3(0,0,-1), glm.vec3(0,0,0))

        # Mock Gyro Samples for Spin
        # CCW Spin around Z (Up) -> Positive Z Gyro
        gyro_sample = glm.vec3(0, 0, 50.0) # 50 deg/s CCW

        # Mock execute_maneuver return value
        sample = MagicMock(spec=IMUReading)
        sample.gyro_raw = gyro_sample

        self.hw_mock.execute_maneuver.return_value = MeasureResult(
            duration=1.5,
            samples=[sample, sample]
        )

        # Run
        with patch("time.sleep"): # Ignore wait
            self.wc.deduce_left_right_autonomous()

        # Verify "The Arc" Power usage via execute_maneuver
        self.hw_mock.execute_maneuver.assert_called_once()
        steps = self.hw_mock.execute_maneuver.call_args[0][0]
        p_left, p_right, _ = steps[0]

        # We don't know exact values, but one should be ~2x the other
        self.assertTrue(p_left > p_right * 1.5 or p_right > p_left * 1.5, f"Motors not in Arc configuration: {p_left}, {p_right}")

        # Verify Swap
        # Initial: L=0, R=1
        # Result: CCW -> Ch0 is Right. So L should be 1, R should be 0.
        # In code: self._update_hw_config(motor_l=self.hw_config.motor_r, ...)
        # self.hw_config.motor_r is 1. So motor_l becomes 1.
        # Check call to model_copy
        # We need to inspect arguments to model_copy
        found_swap = False
        for call_args in self.wc.hw_config.model_copy.call_args_list:
            kwargs = call_args.kwargs.get('update', {})
            if kwargs.get('motor_l') == 1 and kwargs.get('motor_r') == 0:
                found_swap = True
                break
        self.assertTrue(found_swap, "Did not find motor swap update")
        self.assertTrue(self.wc.learning_state.motor_channels_verified)

    def test_left_right_arc_cw(self):
        """Test 'The Arc' where High Power on Left causes Right (CW) Turn."""

        # Simulate CW Turn (Negative Yaw).
        # Ch0 (High Power) is driving the outer arc.
        # CW Turn -> Left Motor is Outer.
        # So Ch0 is Left.
        # Initial Config: L=0. Correct.
        # Expect No Swap.

        gyro_sample = glm.vec3(0, 0, -50.0) # CW
        self.hw_mock.read_imu_raw.return_value = (glm.vec3(0,0,-1), gyro_sample)

        # Mock execute_maneuver
        sample = MagicMock(spec=IMUReading)
        sample.gyro_raw = gyro_sample

        self.hw_mock.execute_maneuver.return_value = MeasureResult(
            duration=1.5,
            samples=[sample, sample]
        )

        with patch("time.sleep"):
            self.wc.deduce_left_right_autonomous()

        # Check NO swap calls
        # (Actually model_copy might be called for other reasons? No, only for updates)
        # But wait, it might be called to update invert flags or verify flag?
        # Verify flag is on learning_state.
        # Check model_copy calls for motor swap
        found_swap = False
        for call_args in self.wc.hw_config.model_copy.call_args_list:
            kwargs = call_args.kwargs.get('update', {})
            if kwargs.get('motor_l') == 1:
                found_swap = True
        self.assertFalse(found_swap, "Found unexpected motor swap")

        self.assertTrue(self.wc.learning_state.motor_channels_verified)
