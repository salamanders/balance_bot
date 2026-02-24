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
    from balance_bot.hardware.robot_hardware import IMUReading, MeasureResult

class TestWiringCheckYank(unittest.TestCase):
    def setUp(self):
        self.wc = WiringCheck()
        self.wc.hw = MagicMock()
        self.wc.hw_config = MagicMock()
        self.wc.learning_state = MagicMock()
        self.wc.learning_state.min_power_visible = 30.0

        # Mock update methods
        self.wc._update_hw_config = MagicMock()
        self.wc._update_learning_state = MagicMock()

    def create_sample(self, ax, ay, az, gx, gy, gz):
        return IMUReading(
            pitch_angle=0, pitch_rate=0, yaw_rate=0, roll_angle=0, roll_rate=0,
            accel_raw=glm.vec3(ax, ay, az),
            gyro_raw=glm.vec3(gx, gy, gz)
        )

    def test_determine_motor_direction_standard(self):
        # 1. Baseline: Stationary (Gravity on Z)
        baseline_accel = glm.vec3(0, 0, 1.0)
        self.wc.hw.read_imu_raw.return_value = (baseline_accel, glm.vec3(0))

        # 2. Drive Response
        # Gyro: Pitching Back (Rotation around X? Let's say X).
        # "inertially pitch backward" -> Nose Up.
        # Let's say rotation is +50 on X.
        # We want this to be NEGATIVE. So Invert should be True.

        # Accel: Moving Forward (Y axis).
        # "IMU mass lags backward".
        # Let's say Delta is +0.2 on Y.
        # We want this to be NEGATIVE. So Invert should be True.

        samples = []
        for _ in range(10):
            # Accel = Baseline + Delta(0, 0.2, 0) -> (0, 0.2, 1.0)
            # Gyro = (50, 0, 0)
            samples.append(self.create_sample(0, 0.2, 1.0, 50, 0, 0))

        res = MeasureResult(duration=0.3, samples=samples)
        self.wc._drive_and_wait = MagicMock(return_value=res)

        self.wc.determine_motor_direction()

        # Check Gyro Pitch
        # Expect Axis.X, Invert=True (because raw > 0)
        # Check Accel Forward
        # Expect Axis.Y, Invert=True (because raw delta > 0)

        # We check the calls to _update_hw_config
        # It might be called multiple times or once.
        # Based on my implementation, called twice.

        # Call 1: Pitch
        self.wc._update_hw_config.assert_any_call(
            gyro_pitch_axis=Axis.X,
            gyro_pitch_invert=True
        )

        # Call 2: Forward
        self.wc._update_hw_config.assert_any_call(
            accel_forward_axis=Axis.Y,
            accel_forward_invert=True
        )

        self.wc._update_learning_state.assert_called_with(motor_direction_verified=True)

    def test_determine_motor_direction_inverted(self):
        # Baseline
        baseline_accel = glm.vec3(0, 1.0, 0) # Gravity on Y this time
        self.wc.hw.read_imu_raw.return_value = (baseline_accel, glm.vec3(0))

        # Drive Response
        # Gyro: -50 on Z. (Raw < 0).
        # We want Negative. So Invert should be False.

        # Accel: Delta -0.2 on X. (Raw < 0).
        # We want Negative. So Invert should be False.

        samples = []
        for _ in range(10):
            # Accel = Baseline(0,1,0) + Delta(-0.2, 0, 0) -> (-0.2, 1, 0)
            # Gyro = (0, 0, -50)
            samples.append(self.create_sample(-0.2, 1.0, 0, 0, 0, -50))

        res = MeasureResult(duration=0.3, samples=samples)
        self.wc._drive_and_wait = MagicMock(return_value=res)

        self.wc.determine_motor_direction()

        self.wc._update_hw_config.assert_any_call(
            gyro_pitch_axis=Axis.Z,
            gyro_pitch_invert=False
        )

        self.wc._update_hw_config.assert_any_call(
            accel_forward_axis=Axis.X,
            accel_forward_invert=False
        )

if __name__ == '__main__':
    unittest.main()
