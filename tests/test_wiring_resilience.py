import unittest
from unittest.mock import MagicMock, patch, ANY
import sys
import math

# Mock smbus/piconzero before importing application code
sys.modules["smbus"] = MagicMock()
sys.modules["piconzero"] = MagicMock()

from src.balance_bot.wiring_check import WiringCheck
from src.balance_bot.utils import Vector3
from src.balance_bot.enums import Axis
from src.balance_bot.config import RobotConfig
from src.balance_bot.hardware.robot_hardware import MeasureResult, IMUReading

class TestWiringResilience(unittest.TestCase):

    def setUp(self):
        # Reset Config singleton-ish behavior by mocking load
        self.mock_config = MagicMock(spec=RobotConfig)
        self.mock_config.motor_i2c_bus = 1
        self.mock_config.imu_i2c_bus = 1
        self.mock_config.motor_trim = 0.0
        # Add the new fields
        self.mock_config.rest_angle_forward = None
        self.mock_config.rest_angle_backward = None
        # Mock other needed fields
        self.mock_config.accel_vertical_axis = None
        self.mock_config.accel_forward_axis = None
        self.mock_config.gyro_pitch_axis = None
        self.mock_config.accel_vertical_invert = False
        self.mock_config.accel_forward_invert = False
        self.mock_config.gyro_pitch_invert = False
        self.mock_config.min_power_visible = 20

    @patch("src.balance_bot.wiring_check.RobotHardware")
    @patch("src.balance_bot.wiring_check.input", side_effect=lambda x: None) # skip input prompts
    @patch("src.balance_bot.wiring_check.time.sleep", side_effect=lambda x: None) # skip sleep
    def test_asymmetric_rest_angles(self, mock_sleep, mock_input, MockHW):
        # Setup Mock Hardware Instance
        hw_instance = MockHW.return_value

        # Scenario:
        # Back Rest: -10 degrees pitch. (Leaning slightly back)
        # Front Rest: +30 degrees pitch. (Leaning heavily forward)
        # We need to simulate Gravity Vector.
        # Assume Robot Frame: Y=Forward, Z=Up (Vertical). Pitch rotates around X.
        # Back (-10 deg): Gravity (Down) is rotated +10 deg relative to Robot Z?
        # If Robot pitches Up (+10), Gravity rotates Back (-10).
        # Wait, usually Pitch Up = Nose Up = Positive.
        # Resting Back = Pitch Up? Or Pitch Down?
        # Usually Back = Negative Pitch (Nose Up).
        # Wait, if I lean back, my nose goes UP.
        # If standard is Pitch=0 is Upright.
        # Pitch > 0 usually means Nose Down (Forward).
        # Let's assume Back = -10, Front = +30.

        # Gravity Vector G = [0, 0, -1] in World.
        # In Robot Frame (Rotated by Pitch P around X):
        # R_x(P) = [[1, 0, 0], [0, cos P, -sin P], [0, sin P, cos P]]
        # G_robot = R_x(P) * G_world
        # But Accelerometer measures Reaction Force = -Gravity (Points Up).
        # A_world = [0, 0, 1].
        # A_robot = R_x(P)^T * A_world ? No, sensor rotates with body.
        # Let's just approximate:
        # Back (-10 deg): Z sees cos(10), Y sees sin(10)?
        # If I lean back (Nose Up), gravity pulls me "Back".
        # So Y component should be negative?
        # Let's construct vectors that WILL result in -10 and +30 given Z=Vert, Y=Fwd.

        # Vector for Back (-10 deg):
        # Z = cos(10) = 0.98, Y = sin(-10)? = -0.17
        vec_back = Vector3(0.0, -0.17, 0.98)

        # Vector for Front (+30 deg):
        # Z = cos(30) = 0.86, Y = sin(30) = 0.5
        vec_front = Vector3(0.0, 0.5, 0.86)

        # read_imu_raw called repeatedly.
        # First 100 calls (Back), then 100 calls (Front).
        # We can use side_effect with an iterator.
        samples = [vec_back] * 110 + [vec_front] * 110
        hw_instance.read_imu_raw.side_effect = lambda: (samples.pop(0), Vector3(0,0,0))

        # Also mock read_imu_converted for the final check
        # It needs to return a reading with pitch_angle set.
        # WiringCheck calls init_hw() again, so we need to mock that logic if it matters.
        # But we just want to verify what it saves to config.
        # The code calls read_imu_converted() at the end to get the angle.
        # We need to ensure read_imu_converted returns -10 then +30?
        # No, it calls it once for Front angle.
        # Then calculates Back angle manually.

        mock_reading_front = MagicMock(spec=IMUReading)
        mock_reading_front.pitch_angle = 30.0
        hw_instance.read_imu_converted.return_value = mock_reading_front

        # Instantiate WiringCheck
        wc = WiringCheck()
        # Inject our mock config
        wc.config = self.mock_config
        wc.hw = hw_instance

        # Run
        wc.calibrate_static_orientation()

        # Verify Axes
        # Dominant in Back is Z (0.98). So Vertical = Z.
        # Pitch Axis = Back x Front.
        # (0, -0.17, 0.98) x (0, 0.5, 0.86)
        # X = (-0.17*0.86) - (0.98*0.5) = -0.14 - 0.49 = -0.63
        # Y = 0
        # Z = 0
        # So Pitch Axis is X.
        # Forward Axis is Y.

        self.assertEqual(wc.config.accel_vertical_axis, Axis.Z)
        self.assertEqual(wc.config.gyro_pitch_axis, Axis.X)
        self.assertEqual(wc.config.accel_forward_axis, Axis.Y)

        # Verify Rest Angles
        # Front should be 30.0 (from mock return)
        self.assertEqual(wc.config.rest_angle_forward, 30.0)

        # Back should be calculated from vec_back using Y and Z.
        # vec_back Y=-0.17, Z=0.98.
        # calculate_pitch(-0.17, 0.98) -> atan2(-0.17, 0.98) -> -9.8 degrees.
        # Close to -10.
        self.assertAlmostEqual(wc.config.rest_angle_backward, -9.84, places=1)

    @patch("src.balance_bot.wiring_check.RobotHardware")
    @patch("src.balance_bot.wiring_check.time.sleep", side_effect=lambda x: None)
    def test_adaptive_trim_calibration(self, mock_sleep, MockHW):
        hw_instance = MockHW.return_value
        wc = WiringCheck()
        wc.config = self.mock_config
        wc.hw = hw_instance

        # Scenario: Huge Motor Mismatch.
        # Driving straight produces +20 deg/s Yaw (Turning Left -> Right Motor Stronger).
        # We need to increase trim.
        # Attempt 1: AvgYaw = 20. Trim += 20 * 0.015 (high gain) = 0.3.
        # Attempt 2: Drift reduces to 5 deg/s. Trim += 5 * 0.005 = 0.025. Total 0.325.
        # Attempt 3: Drift 0.

        results = [
            MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=20.0)]*10),
            MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=5.0)]*10),
            MeasureResult(duration=1.0, samples=[MagicMock(yaw_rate=0.0)]*10),
        ]
        hw_instance.drive_and_measure.side_effect = results

        wc.calibrate_motor_trim()

        # Verify Trim
        # Initial 0.0
        # Step 1: 20 * 0.015 = 0.3. New Trim = 0.3.
        # Step 2: 5 * 0.005 = 0.025. New Trim = 0.325.
        # Step 3: Stop.

        self.assertAlmostEqual(wc.config.motor_trim, 0.325, places=3)
        self.assertEqual(hw_instance.drive_and_measure.call_count, 3)

if __name__ == "__main__":
    unittest.main()
