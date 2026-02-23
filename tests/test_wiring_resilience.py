import unittest
from unittest.mock import MagicMock, patch, ANY
import sys
import math

class TestWiringResilience(unittest.TestCase):

    def setUp(self):
        # Patch modules to mock hardware dependencies
        self.modules_patcher = patch.dict(sys.modules, {
            'smbus': MagicMock(),
            'smbus2': MagicMock(),
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
            'balance_bot.configuration'
        ]
        for m in modules_to_clear:
            if m in sys.modules:
                del sys.modules[m]

        # Import dependencies (now safe)
        from balance_bot.configuration import HardwareConfig, LearningState, PIDParams
        self.HardwareConfig = HardwareConfig
        self.LearningState = LearningState
        self.PIDParams = PIDParams

        # Setup mock config
        self.mock_hw_config = self.HardwareConfig(motor_i2c_bus=1, imu_i2c_bus=1)
        # Mock save on immutable config
        object.__setattr__(self.mock_hw_config, 'save', MagicMock())

        self.mock_learning_state = self.LearningState(pid=self.PIDParams())
        self.mock_learning_state.min_power_visible = 20
        # Bypass Pydantic __setattr__ to mock method on instance
        object.__setattr__(self.mock_learning_state, 'save', MagicMock())

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
        from balance_bot.hardware.robot_hardware import IMUReading, MeasureResult

        # Setup Mock Hardware Instance
        hw_instance = MockHW.return_value

        vec_back = Vector3(0.0, -0.17, 0.98)
        vec_front = Vector3(0.0, 0.5, 0.86)

        # Mock drive_and_measure for gravity vector measurement
        # We need it to return MeasureResult with samples containing accel_raw
        r_back = MagicMock(spec=IMUReading)
        r_back.accel_raw = vec_back
        res_back = MeasureResult(duration=1.0, samples=[r_back]*50)

        r_front = MagicMock(spec=IMUReading)
        r_front.accel_raw = vec_front
        res_front = MeasureResult(duration=1.0, samples=[r_front]*50)

        hw_instance.drive_and_measure.side_effect = [res_back, res_front]

        mock_reading_front = MagicMock(spec=IMUReading)
        mock_reading_front.pitch_angle = 30.0
        hw_instance.read_imu_converted.return_value = mock_reading_front

        # Configure get_axis_value side effect to perform real calculation
        def get_axis_value_side_effect(vec, axis, invert):
            val = getattr(vec, axis.value)
            return -val if invert else val
        hw_instance.get_axis_value.side_effect = get_axis_value_side_effect

        # Need to patch loads so WiringCheck uses our mocks
        with patch('balance_bot.configuration.HardwareConfig.load', return_value=self.mock_hw_config), \
             patch('balance_bot.configuration.LearningState.load', return_value=self.mock_learning_state):

            wc = WiringCheck()
            wc.hw = hw_instance

            # Since WiringCheck reloads configs in __init__, we might need to re-inject mocks
            # if we didn't patch load. But we patched load.
            # However, calibrate_static_orientation modifies hw_config using model_copy.
            # So we need to ensure model_copy returns something we can verify or intercept save.
            # But the test checks wc.config attributes.
            # wc.hw_config will be replaced.

            # Let's mock _update_hw_config to update our mock object instead of creating new one
            # Or just let it update and check result.
            # Since we pass mock_hw_config which is a real Pydantic model (with mocked save),
            # model_copy will work and return a new Pydantic model.

            wc.calibrate_static_orientation()

            self.assertEqual(wc.hw_config.accel_vertical_axis, Axis.Z)
            self.assertEqual(wc.hw_config.gyro_pitch_axis, Axis.X)
            self.assertEqual(wc.hw_config.accel_forward_axis, Axis.Y)

            self.assertEqual(wc.learning_state.rest_angle_forward, 30.0)
            self.assertAlmostEqual(wc.learning_state.rest_angle_backward, -9.84, places=1)

    @patch("balance_bot.wiring_check.RobotHardware")
    @patch("balance_bot.wiring_check.time.sleep", side_effect=lambda x: None)
    def test_adaptive_trim_calibration(self, mock_sleep, MockHW):
        from balance_bot.wiring_check import WiringCheck
        from balance_bot.hardware.robot_hardware import MeasureResult

        hw_instance = MockHW.return_value

        with patch('balance_bot.configuration.HardwareConfig.load', return_value=self.mock_hw_config), \
             patch('balance_bot.configuration.LearningState.load', return_value=self.mock_learning_state):

            wc = WiringCheck()
            wc.hw = hw_instance

            results = [
                MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=20.0)]*10),
                MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=5.0)]*10),
                MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=0.0)]*10),
            ]
            hw_instance.drive_and_measure.side_effect = results

            wc.calibrate_motor_trim()

            self.assertAlmostEqual(wc.learning_state.motor_trim, 0.325, places=3)
            self.assertEqual(hw_instance.drive_and_measure.call_count, 3)

if __name__ == "__main__":
    unittest.main()
