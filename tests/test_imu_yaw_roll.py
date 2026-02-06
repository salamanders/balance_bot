import math
from unittest.mock import MagicMock
from balance_bot.hardware.robot_hardware import RobotHardware, IMUReading
from balance_bot.enums import Axis

def test_imu_yaw_roll_defaults(monkeypatch):
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")
    # Defaults: Yaw=Z, Roll=Y. AccelRoll=X (since Vert=Z, Fwd=Y).
    hw = RobotHardware(0, 1)
    hw.sensor = MagicMock()

    # Simulate:
    # Yaw (Z) = 10 deg/s
    # Roll (Y) = 20 deg/s
    # Pitch (X) = 0
    hw.sensor.get_gyro_data.return_value = {"x": 0.0, "y": 20.0, "z": 10.0}
    # Accel: Vertical(Z)=1g. Roll(X)=0.5g. Forward(Y)=0.
    # Roll Angle = atan2(X, Z) = atan2(0.5, 1.0) approx 26.5 deg
    hw.sensor.get_accel_data.return_value = {"x": 0.5, "y": 0.0, "z": 1.0}

    reading = hw.read_imu_converted()

    assert math.isclose(reading.yaw_rate, 10.0)
    assert math.isclose(reading.roll_rate, 20.0)
    # math.degrees(math.atan2(0.5, 1.0)) = 26.565
    assert math.isclose(reading.roll_angle, 26.565, abs_tol=0.1)

def test_imu_yaw_roll_custom_axis_invert(monkeypatch):
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")
    # Set Yaw to X (invert), Roll to Z (invert)
    # Gyro Pitch default is X, so we can't use X for Yaw unless we change Pitch.
    # Let's say: Pitch=Y, Yaw=X, Roll=Z.
    hw = RobotHardware(
        0, 1,
        gyro_axis=Axis.Y,
        gyro_yaw_axis=Axis.X, gyro_yaw_invert=True,
        gyro_roll_axis=Axis.Z, gyro_roll_invert=True,
        accel_vertical_axis=Axis.Z,
        accel_forward_axis=Axis.Y
        # Accel Roll Axis will be deduced as X.
    )
    hw.sensor = MagicMock()

    hw.sensor.get_gyro_data.return_value = {"x": 10.0, "y": 0.0, "z": 20.0}
    # Accel: Roll(X) = 0.5. Vert(Z) = 1.0.
    hw.sensor.get_accel_data.return_value = {"x": 0.5, "y": 0.0, "z": 1.0}

    reading = hw.read_imu_converted()

    # Yaw is X (10), inverted -> -10
    assert math.isclose(reading.yaw_rate, -10.0)
    # Roll is Z (20), inverted -> -20
    assert math.isclose(reading.roll_rate, -20.0)
    # Roll Angle: Accel Roll Axis is X (deduced). atan2(0.5, 1.0) -> 26.565
    assert math.isclose(reading.roll_angle, 26.565, abs_tol=0.1)
