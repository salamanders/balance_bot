import unittest
from unittest.mock import MagicMock, patch, ANY, call
import sys
import math

# Define mock classes if needed, or rely on MagicMock
# We need Vector3 for type hints or usage in create_sample if we import it

from src.balance_bot.utils import Vector3
# We need IMUReading and MeasureResult for mocking return values
from src.balance_bot.hardware.robot_hardware import IMUReading, MeasureResult

# Mock dependencies before importing wiring_check
with patch.dict(sys.modules, {
    'smbus2': MagicMock(),
    'mpu6050': MagicMock(),
    'src.balance_bot.hardware.piconzero': MagicMock(),
}):
    from src.balance_bot.wiring_check import WiringCheck, HardwareConfig, LearningState

class TestWiringCheckPhase(unittest.TestCase):
    def setUp(self):
        # Patch HardwareConfig and LearningState load
        self.mock_hw_config = MagicMock()
        self.mock_hw_config.motor_r_invert = False
        # When model_copy is called, we return self.mock_hw_config but don't automatically update attributes
        # unless we side_effect it. But here we just want to verify logic flow.
        self.mock_hw_config.model_copy.return_value = self.mock_hw_config

        self.mock_learning_state = MagicMock()
        self.mock_learning_state.min_power_visible = 30.0
        self.mock_learning_state.motor_phasing_verified = False

        # Patch the class methods load
        self.patcher1 = patch('src.balance_bot.wiring_check.HardwareConfig.load', return_value=self.mock_hw_config)
        self.patcher2 = patch('src.balance_bot.wiring_check.LearningState.load', return_value=self.mock_learning_state)
        self.patcher1.start()
        self.patcher2.start()

        self.wc = WiringCheck()

        # Inject mock hardware
        self.wc.hw = MagicMock()
        self.wc.hw.hw_config = self.mock_hw_config
        self.wc.hw.get_mapped_value.return_value = 0.0

    def tearDown(self):
        self.patcher1.stop()
        self.patcher2.stop()

    def create_sample(self, ax, ay, az, gx, gy, gz, pitch=0.0, yaw=0.0):
        return IMUReading(
            pitch_angle=pitch,
            pitch_rate=gx,
            yaw_rate=yaw,
            roll_angle=0.0,
            roll_rate=0.0,
            accel_raw=Vector3(ax, ay, az),
            gyro_raw=Vector3(gx, gy, gz)
        )

    def test_align_motors_phase_stuck(self):
        # Mock _drive_and_wait to return stuck result (no noise)
        samples = [self.create_sample(0, 0, 1.0, 0, 0, 0) for _ in range(10)]
        res = MeasureResult(duration=0.5, samples=samples)
        self.wc._drive_and_wait = MagicMock(return_value=res)

        # Expect SystemExit after retries
        with self.assertRaises(SystemExit):
             self.wc.align_motors_phase()

        # Check that it tried 3 times (0, 1, 2)
        # _drive_and_wait(p, p, 0.5)
        self.assertEqual(self.wc._drive_and_wait.call_count, 3)

    def test_align_motors_phase_spinning(self):
        # Scenario: Spinning.
        # Need some noise on accel to pass Stuck check
        samples = []
        for i in range(10):
            noise = (i % 2) * 0.1
            # Yaw high (40)
            samples.append(self.create_sample(0, 0, 1.0 + noise, 0, 0, 40.0, yaw=40.0))
        res = MeasureResult(duration=0.5, samples=samples)
        self.wc._drive_and_wait = MagicMock(return_value=res)

        # Mock get_mapped_value to be low so it doesn't pass translation check
        self.wc.hw.get_mapped_value.return_value = 0.0

        # Spy on _update_hw_config using a wrapper
        # We can't use patch.object easily on the method itself if we want to call it?
        # Actually verify calls self._update_hw_config.
        # We can mock it.
        self.wc._update_hw_config = MagicMock()

        with self.assertRaises(SystemExit):
            self.wc.align_motors_phase()

        # Should have called update_hw_config to invert motor_r
        # motor_r_invert = not False -> True
        self.wc._update_hw_config.assert_any_call(motor_r_invert=True)

    def test_align_motors_phase_success(self):
        # Scenario: Translating (Forward Accel).
        samples = []
        for i in range(10):
            noise = (i % 2) * 0.1
            # Forward accel 0.2
            samples.append(self.create_sample(0.2, 0, 1.0 + noise, 0, 5, 0, yaw=2.0))
        res = MeasureResult(duration=0.5, samples=samples)
        self.wc._drive_and_wait = MagicMock(return_value=res)

        # We need self.hw.get_mapped_value to return 0.2 for "accel_forward"
        self.wc.hw.get_mapped_value.side_effect = lambda vec, name: vec.x if name == "accel_forward" else 0.0

        # Should pass on first attempt
        try:
            self.wc.align_motors_phase()
        except SystemExit:
            self.fail("align_motors_phase raised SystemExit unexpectedly!")

        self.assertTrue(self.mock_learning_state.motor_phasing_verified)

if __name__ == '__main__':
    unittest.main()
