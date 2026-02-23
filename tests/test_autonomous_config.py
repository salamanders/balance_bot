import sys
import unittest
from unittest.mock import MagicMock, patch

# Mock Hardware Libraries BEFORE importing balance_bot modules
sys.modules["smbus2"] = MagicMock()
sys.modules["mpu6050"] = MagicMock()

# Now we can import safely
from balance_bot.wiring_check import WiringCheck
from balance_bot.config import RobotConfig, PIDParams
from balance_bot.utils import Vector3
from balance_bot.enums import Axis

class TestAutonomousConfig(unittest.TestCase):
    def setUp(self):
        # Patch RobotConfig.load to return a fresh config
        with patch('balance_bot.config.RobotConfig.load') as mock_load:
            mock_load.return_value = RobotConfig(pid=PIDParams())
            self.check = WiringCheck()

        # Mock HW
        self.check.hw = MagicMock()
        # Default side_effect for read_imu_raw (Infinite loop safety)
        self.check.hw.read_imu_raw.return_value = (Vector3(0, 0, -1), Vector3(0, 0, 0))

        # Reset Config
        self.check.config.motor_l = 0
        self.check.config.motor_r = 1
        self.check.config.min_power_visible = 50
        self.check.config.gyro_yaw_axis = Axis.Z
        self.check.config.gyro_yaw_invert = False

        # Mock drive_and_measure to avoid actual sleep
        # self.check.hw.drive_and_measure needs to be mocked per test

        # Mock wait_for_stability to avoid blocking or IMU errors
        self.check.hw.wait_for_stability = MagicMock()

    def test_deduce_left_right_ccw_spin(self):
        """
        Case: Robot spins CCW (Left). Dot Product > 0.
        Deduction: Ch0 is Right, Ch1 is Left.
        Expect: Config updated to L=1, R=0 (Swap from default).
        """
        # 1. Mock Gravity (-Z) -> Up is +Z
        # 2. Mock Gyro (+Z) -> Dot Prod > 0

        # We need a sequence for read_imu_raw:
        # Call 1: Gravity Check (Up Vector)
        # Call 2...N: Spin Loop (Gyro)

        spin_sample = (Vector3(0, 0, -1.0), Vector3(0, 0, 100.0))

        self.check.hw.read_imu_raw.side_effect = [
            (Vector3(0, 0, -1.0), Vector3(0, 0, 0)), # Gravity
        ] + [spin_sample] * 200 # Spin samples

        # Mock execute_maneuver to return the spin samples
        res = MagicMock()
        # Create samples that appear to have gyro_raw
        s = MagicMock()
        s.gyro_raw = spin_sample[1] # The gyro part
        res.samples = [s] * 10
        self.check.hw.execute_maneuver.return_value = res

        with patch('builtins.input', return_value=''):
            self.check.deduce_left_right_autonomous()

        # Verify Swap
        self.assertEqual(self.check.config.motor_l, 1)
        self.assertEqual(self.check.config.motor_r, 0)
        self.assertTrue(self.check.config.motor_channels_verified)

    def test_deduce_left_right_cw_spin(self):
        """
        Case: Robot spins CW (Right). Dot Product < 0.
        Deduction: Ch0 is Left, Ch1 is Right.
        Expect: Config matches Ch0=Left, Ch1=Right (L=0, R=1).
        """
        # Start with Wrong Config (L=1, R=0)
        self.check.config.motor_l = 1
        self.check.config.motor_r = 0

        spin_sample = (Vector3(0, 0, -1.0), Vector3(0, 0, -100.0))

        self.check.hw.read_imu_raw.side_effect = [
            (Vector3(0, 0, -1.0), Vector3(0, 0, 0)), # Gravity
        ] + [spin_sample] * 200 # Spin samples

        # Mock execute_maneuver
        res = MagicMock()
        s = MagicMock()
        s.gyro_raw = spin_sample[1]
        res.samples = [s] * 10
        self.check.hw.execute_maneuver.return_value = res

        with patch('builtins.input', return_value=''):
            self.check.deduce_left_right_autonomous()

        # Verify Swap back to L=0, R=1
        self.assertEqual(self.check.config.motor_l, 0)
        self.assertEqual(self.check.config.motor_r, 1)

    def test_calibrate_motor_trim_left_drift(self):
        """
        Case: Drifting Left (Pos Yaw).
        Right Motor is stronger.
        Expect: Trim increases (Positive).
        """
        self.check.config.motor_trim = 0.0

        # Mock drive_and_measure to return Pos Yaw
        res = MagicMock()
        res.samples = [MagicMock()]
        res.avg_yaw_rate = 10.0
        self.check.hw.drive_and_measure.return_value = res

        # Mock init_hw to do nothing
        self.check.init_hw = MagicMock()

        self.check.calibrate_motor_trim()

        # Expected: 15 attempts (New logic).
        # Each attempt adds 10.0 * 0.005 = 0.05.
        # 15 * 0.05 = 0.75.
        # Clamped to 0.4 (New logic).
        self.assertAlmostEqual(self.check.config.motor_trim, 0.4)

    def test_calibrate_motor_trim_right_drift(self):
        """
        Case: Drifting Right (Neg Yaw).
        Left Motor is stronger.
        Expect: Trim decreases (Negative).
        """
        self.check.config.motor_trim = 0.0

        res = MagicMock()
        res.samples = [MagicMock()]
        res.avg_yaw_rate = -10.0
        self.check.hw.drive_and_measure.return_value = res

        self.check.init_hw = MagicMock()

        self.check.calibrate_motor_trim()

        # -0.75 clamped to -0.4
        self.assertAlmostEqual(self.check.config.motor_trim, -0.4)

    def test_calibrate_motor_trim_converges(self):
        """
        Case: Drift reduces to 0.
        Expect: Trim stops changing.
        """
        self.check.config.motor_trim = 0.0

        # Sequence of return values for drive_and_measure
        # 1. 10.0 (High) -> Trim becomes 0.05
        # 2. 1.0 (Low/Converged) -> Stop

        res1 = MagicMock(); res1.samples = [MagicMock()]; res1.avg_yaw_rate = 10.0
        res2 = MagicMock(); res2.samples = [MagicMock()]; res2.avg_yaw_rate = 1.0

        self.check.hw.drive_and_measure.side_effect = [
            res1,
            res2
        ]

        self.check.init_hw = MagicMock()

        self.check.calibrate_motor_trim()

        # Should have run 2 times.
        # 1st: Trim += 0.05.
        # 2nd: Converged.
        self.assertAlmostEqual(self.check.config.motor_trim, 0.05)

if __name__ == "__main__":
    unittest.main()
