import unittest
from unittest.mock import MagicMock, patch
from balance_bot.hardware.robot_hardware import RobotHardware, Vector3

class TestRobotHardwareLogic(unittest.TestCase):
    def setUp(self):
        # Patch init_hardware to avoid real imports/drivers
        with patch.object(RobotHardware, '_init_hardware'):
            self.hw = RobotHardware()
            self.hw.sensor = MagicMock()
            self.hw.pz = MagicMock()

    def test_wait_for_stability_success(self):
        # Setup readings: Moving -> Stable -> Stable
        # (accel, gyro)
        moving = (MagicMock(), Vector3(10.0, 10.0, 10.0))
        stable = (MagicMock(), Vector3(0.1, 0.1, 0.1))

        self.hw.sensor.get_accel_data.return_value = {'x':0, 'y':0, 'z':0} # Dummy

        # We mock read_imu_raw logic or side effects on sensor?
        # read_imu_raw calls sensor.get_gyro_data() and wraps in Vector3
        # But wait, read_imu_raw logic is:
        # try: accel=get... gyro=get... return accel, gyro

        # Let's mock read_imu_raw directly on the instance
        self.hw.read_imu_raw = MagicMock()
        self.hw.read_imu_raw.side_effect = [moving, stable, stable, stable, stable, stable]

        # Mock time to control loop
        start_time = 1000.0
        def time_gen():
            nonlocal start_time
            start_time += 0.1
            return start_time

        with patch("time.sleep"), \
             patch("time.time", side_effect=time_gen):

             self.hw.wait_for_stability(duration=0.2, threshold=5.0)

        # Should have called at least 3 times (Moving, Stable(Start), Stable(End))
        self.assertGreaterEqual(self.hw.read_imu_raw.call_count, 3)

    def test_wait_for_stability_interrupted(self):
        # Sequence: Stable, Moving, Stable, Stable...
        moving = (MagicMock(), Vector3(10.0, 10.0, 10.0))
        stable = (MagicMock(), Vector3(0.1, 0.1, 0.1))

        self.hw.read_imu_raw = MagicMock()
        self.hw.read_imu_raw.side_effect = [stable, moving, stable, stable, stable, stable]

        start_time = 1000.0
        def time_gen():
            nonlocal start_time
            start_time += 0.1
            return start_time

        with patch("time.sleep"), \
             patch("time.time", side_effect=time_gen):

             self.hw.wait_for_stability(duration=0.2, threshold=5.0)

        # Should reset on moving
        self.assertGreaterEqual(self.hw.read_imu_raw.call_count, 4)

if __name__ == "__main__":
    unittest.main()
