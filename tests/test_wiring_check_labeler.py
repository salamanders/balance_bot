import unittest
from unittest.mock import MagicMock, patch
import glm
import sys

# Patch modules before import
with patch.dict(sys.modules, {
    'smbus2': MagicMock(),
    'mpu6050': MagicMock(),
    'balance_bot.hardware.piconzero': MagicMock(),
}):
    from balance_bot.wiring_check import WiringCheck, Axis
    from balance_bot.hardware.robot_hardware import IMUReading

class TestWiringCheckLabeler(unittest.TestCase):
    def setUp(self):
        self.wc = WiringCheck()
        self.wc.hw = MagicMock()
        self.wc.hw_config = MagicMock()
        self.wc.learning_state = MagicMock()
        self.wc.learning_state.min_power_visible = 30.0

        self.wc.hw_config.gyro_pitch_axis = Axis.X
        self.wc.hw_config.accel_forward_axis = Axis.Y
        self.wc.hw_config.accel_vertical_axis = None
        self.wc.hw_config.gyro_yaw_axis = None
        self.wc.hw_config.gyro_roll_axis = None

        self.wc._update_hw_config = MagicMock()
        self.wc._update_learning_state = MagicMock()

        # Mock _toddler_flail_collection instead of sort_resting_vectors
        self.wc._toddler_flail_collection = MagicMock()

    def test_calibrate_static_orientation_success(self):
        p1 = glm.vec3(0, 1.0, 1.0)
        p2 = glm.vec3(0, -1.0, 1.0)

        # Return enough vectors for sorting to work
        # sort_resting_vectors uses the first vector as pivot
        # p1 and p2 are separated by 90 deg (>20), so it should split fine.
        self.wc._toddler_flail_collection.return_value = [p1, p1, p2, p2]

        def mock_get_mapped(vec, name):
            if name == "accel_forward":
                return vec.y
            if name == "accel_vertical":
                return vec.z
            return 0.0
        self.wc.hw.get_mapped_value.side_effect = mock_get_mapped

        self.wc.calibrate_static_orientation()

        self.wc._update_hw_config.assert_called_with(
            accel_vertical_axis=Axis.Z,
            accel_vertical_invert=False,
            gyro_yaw_axis=Axis.Z,
            gyro_yaw_invert=False,
            gyro_roll_axis=Axis.Y
        )

        args, kwargs = self.wc._update_learning_state.call_args
        self.assertAlmostEqual(kwargs['rest_angle_forward'], 45.0, delta=0.1)
        self.assertAlmostEqual(kwargs['rest_angle_backward'], -45.0, delta=0.1)

    def test_calibrate_static_orientation_inverted_vertical(self):
        self.wc.hw_config.gyro_pitch_axis = Axis.X
        self.wc.hw_config.accel_forward_axis = Axis.Y

        p1 = glm.vec3(0, 1.0, -1.0)
        p2 = glm.vec3(0, -1.0, -1.0)

        self.wc._toddler_flail_collection.return_value = [p1, p1, p2, p2]

        def mock_get_mapped(vec, name):
            if name == "accel_forward":
                return vec.y
            if name == "accel_vertical":
                return -vec.z
            return 0.0
        self.wc.hw.get_mapped_value.side_effect = mock_get_mapped

        self.wc.calibrate_static_orientation()

        # Check for invert=True
        self.wc._update_hw_config.assert_called_with(
            accel_vertical_axis=Axis.Z,
            accel_vertical_invert=True,
            gyro_yaw_axis=Axis.Z,
            gyro_yaw_invert=False,
            gyro_roll_axis=Axis.Y
        )

        args, kwargs = self.wc._update_learning_state.call_args
        self.assertAlmostEqual(kwargs['rest_angle_forward'], 45.0, delta=0.1)

if __name__ == '__main__':
    unittest.main()
