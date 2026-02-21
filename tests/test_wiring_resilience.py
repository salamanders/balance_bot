import unittest
from unittest.mock import MagicMock, patch, ANY
import sys
import math

class TestWiringResilience(unittest.TestCase):

    def setUp(self):
        # Patch modules to mock hardware dependencies
        self.modules_patcher = patch.dict(sys.modules, {
            'smbus': MagicMock(),
            'balance_bot.hardware.piconzero': MagicMock(),
            # We might need to mock mpu6050 if RobotHardware imports it
            'mpu6050': MagicMock()
        })
        self.modules_patcher.start()

        # Reload relevant modules to ensure they pick up the mocks
        modules_to_clear = [
            'balance_bot.wiring_check',
            'balance_bot.hardware.robot_hardware',
            'balance_bot.hardware.piconzero',
            'balance_bot.config'
        ]
        for m in modules_to_clear:
            if m in sys.modules:
                del sys.modules[m]

        # Import dependencies (now safe)
        from balance_bot.config import RobotConfig, PIDParams
        self.RobotConfig = RobotConfig
        self.PIDParams = PIDParams

        # Setup mock config
        self.mock_config = self.RobotConfig(pid=self.PIDParams())
        self.mock_config.motor_i2c_bus = 1
        self.mock_config.imu_i2c_bus = 1
        self.mock_config.motor_trim = 0.0
        self.mock_config.min_power_visible = 20

    def tearDown(self):
        self.modules_patcher.stop()

    @patch("balance_bot.wiring_check.RobotHardware")
    @patch("balance_bot.wiring_check.input", side_effect=lambda x: None)
    @patch("balance_bot.wiring_check.time.sleep", side_effect=lambda x: None)
    def test_asymmetric_rest_angles(self, mock_sleep, mock_input, MockHW):
        # Must import inside test to ensure we get the version using mocks
        from balance_bot.wiring_check import WiringCheck
        from balance_bot.utils import Vector3
        from balance_bot.enums import Axis
        from balance_bot.hardware.robot_hardware import IMUReading

        # Setup Mock Hardware Instance
        hw_instance = MockHW.return_value

        vec_back = Vector3(0.0, -0.17, 0.98)
        vec_front = Vector3(0.0, 0.5, 0.86)

        samples = [vec_back] * 110 + [vec_front] * 110
        hw_instance.read_imu_raw.side_effect = lambda: (samples.pop(0), Vector3(0,0,0))

        mock_reading_front = MagicMock(spec=IMUReading)
        mock_reading_front.pitch_angle = 30.0
        hw_instance.read_imu_converted.return_value = mock_reading_front

        wc = WiringCheck()
        wc.config = self.mock_config
        wc.hw = hw_instance

        wc.calibrate_static_orientation()

        self.assertEqual(wc.config.accel_vertical_axis, Axis.Z)
        self.assertEqual(wc.config.gyro_pitch_axis, Axis.X)
        self.assertEqual(wc.config.accel_forward_axis, Axis.Y)

        self.assertEqual(wc.config.rest_angle_forward, 30.0)
        self.assertAlmostEqual(wc.config.rest_angle_backward, -9.84, places=1)

    @patch("balance_bot.wiring_check.RobotHardware")
    @patch("balance_bot.wiring_check.time.sleep", side_effect=lambda x: None)
    def test_adaptive_trim_calibration(self, mock_sleep, MockHW):
        from balance_bot.wiring_check import WiringCheck
        from balance_bot.hardware.robot_hardware import MeasureResult

        hw_instance = MockHW.return_value
        wc = WiringCheck()
        wc.config = self.mock_config
        wc.hw = hw_instance

        results = [
            MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=20.0)]*10),
            MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=5.0)]*10),
            MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=0.0)]*10),
        ]
        hw_instance.drive_and_measure.side_effect = results

        wc.calibrate_motor_trim()

        self.assertAlmostEqual(wc.config.motor_trim, 0.325, places=3)
        self.assertEqual(hw_instance.drive_and_measure.call_count, 3)

if __name__ == "__main__":
    unittest.main()
