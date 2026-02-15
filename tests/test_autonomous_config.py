import sys
import unittest
from unittest.mock import MagicMock, patch

sys.modules["smbus"] = MagicMock()
sys.modules["mpu6050"] = MagicMock()

from balance_bot.wiring_check import WiringCheck, MeasureResult
from balance_bot.config import RobotConfig
from balance_bot.utils import Vector3
from balance_bot.enums import Axis

class TestAutonomousConfig(unittest.TestCase):
    def setUp(self):
        with patch('balance_bot.config.RobotConfig.load') as mock_load:
            mock_load.return_value = RobotConfig(pid=MagicMock())
            self.check = WiringCheck()

        self.check.hw = MagicMock()
        self.check.config.motor_l = 0
        self.check.config.motor_r = 1
        self.check.config.min_power_visible = 50
        self.check.config.gyro_yaw_axis = Axis.Z
        self.check.config.gyro_yaw_invert = False

        self.check.wait_for_stability = MagicMock()
        self.check.init_hw = MagicMock()

    @patch('balance_bot.wiring_check.WiringCheck.collect_data')
    def test_deduce_left_right_ccw_spin(self, mock_collect):
        # Result 1: Gravity (-Z) -> Up is +Z
        r1 = MagicMock(spec=MeasureResult)
        r1.avg_accel_raw = Vector3(0, 0, -1.0)

        # Result 2: Spin Gyro (+Z) -> Dot Prod > 0
        r2 = MagicMock(spec=MeasureResult)
        r2.avg_gyro_raw = Vector3(0, 0, 100.0)

        mock_collect.side_effect = [r1, r2]

        self.check.deduce_left_right_autonomous()

        # Verify Swap
        self.assertEqual(self.check.config.motor_l, 1)
        self.assertEqual(self.check.config.motor_r, 0)
        self.assertTrue(self.check.config.motor_channels_verified)

    @patch('balance_bot.wiring_check.WiringCheck.collect_data')
    def test_deduce_left_right_cw_spin(self, mock_collect):
        self.check.config.motor_l = 1
        self.check.config.motor_r = 0

        # Result 1: Gravity
        r1 = MagicMock(spec=MeasureResult)
        r1.avg_accel_raw = Vector3(0, 0, -1.0)

        # Result 2: Spin Gyro (-Z) -> Dot Prod < 0
        r2 = MagicMock(spec=MeasureResult)
        r2.avg_gyro_raw = Vector3(0, 0, -100.0)

        mock_collect.side_effect = [r1, r2]

        self.check.deduce_left_right_autonomous()

        # Verify Swap back to L=0, R=1
        self.assertEqual(self.check.config.motor_l, 0)
        self.assertEqual(self.check.config.motor_r, 1)

    @patch('balance_bot.wiring_check.WiringCheck.collect_data')
    def test_calibrate_motor_trim_left_drift(self, mock_collect):
        # Case: Drifting Left (Pos Yaw).
        self.check.config.motor_trim = 0.0

        sample = MagicMock(spec=MeasureResult)
        sample.avg_yaw_rate = 10.0
        mock_collect.return_value = sample

        self.check.calibrate_motor_trim()

        # 10 attempts. Each adds 10*0.005 = 0.05. 10*0.05 = 0.5. Clamped 0.3.
        self.assertAlmostEqual(self.check.config.motor_trim, 0.3)

    @patch('balance_bot.wiring_check.WiringCheck.collect_data')
    def test_calibrate_motor_trim_right_drift(self, mock_collect):
        # Case: Drifting Right (Neg Yaw).
        self.check.config.motor_trim = 0.0

        sample = MagicMock(spec=MeasureResult)
        sample.avg_yaw_rate = -10.0
        mock_collect.return_value = sample

        self.check.calibrate_motor_trim()

        self.assertAlmostEqual(self.check.config.motor_trim, -0.3)

    @patch('balance_bot.wiring_check.WiringCheck.collect_data')
    def test_calibrate_motor_trim_converges(self, mock_collect):
        self.check.config.motor_trim = 0.0

        s1 = MagicMock(spec=MeasureResult); s1.avg_yaw_rate = 10.0
        s2 = MagicMock(spec=MeasureResult); s2.avg_yaw_rate = 1.0

        mock_collect.side_effect = [s1, s2]

        self.check.calibrate_motor_trim()

        # 1st attempt: trim += 0.05.
        # 2nd attempt: converged.
        self.assertAlmostEqual(self.check.config.motor_trim, 0.05)

if __name__ == "__main__":
    unittest.main()
