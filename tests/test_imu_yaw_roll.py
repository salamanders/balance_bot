from balance_bot.enums import Axis
from balance_bot.hardware.robot_hardware import IMUReading
import glm
import math

def test_imu_yaw_roll_defaults(mock_hardware_factory):
    """Test default Yaw/Roll axis mapping."""
    # Defaults: Yaw=Z, Roll=Y. AccelRoll=X (since Vert=Z, Fwd=Y).
    hw = mock_hardware_factory(
        motor_l=0,
        motor_r=1,
        gyro_pitch_axis=Axis.X,
        accel_vertical_axis=Axis.Z,
        accel_forward_axis=Axis.Y,
        gyro_yaw_axis=Axis.Z,
        gyro_roll_axis=Axis.Y
    )

    # Simulate:
    # Yaw (Z) = 10 deg/s
    # Roll (Y) = 20 deg/s
    # Pitch (X) = 0
    hw.sensor.get_gyro_data.return_value = glm.vec3(0.0, 20.0, 10.0)
    # Accel: Vertical(Z)=1g. Roll(X)=0.5g. Forward(Y)=0.
    # Roll Angle = atan2(X, Z) = atan2(0.5, 1.0) approx 26.5 deg
    hw.sensor.get_accel_data.return_value = glm.vec3(0.5, 0.0, 1.0)

    reading: IMUReading = hw.read_imu_converted()

    assert math.isclose(reading.yaw_rate, 10.0)
    assert math.isclose(reading.roll_rate, 20.0)
    assert math.isclose(reading.roll_angle, math.degrees(math.atan2(0.5, 1.0)), abs_tol=0.1)

def test_imu_yaw_roll_custom_axis_invert(mock_hardware_factory):
    """Test custom axis mapping and inversion."""
    # Set Yaw to X (invert), Roll to Z (invert)
    # Pitch=Y.
    hw = mock_hardware_factory(
        motor_l=0,
        motor_r=1,
        gyro_pitch_axis=Axis.Y,
        gyro_yaw_axis=Axis.X,
        gyro_yaw_invert=True,
        gyro_roll_axis=Axis.Z,
        gyro_roll_invert=True,
        accel_vertical_axis=Axis.Z,
        accel_forward_axis=Axis.Y
    )

    hw.sensor.get_gyro_data.return_value = glm.vec3(10.0, 0.0, 20.0)
    # Accel: Roll(X) = 0.5. Vert(Z) = 1.0.
    hw.sensor.get_accel_data.return_value = glm.vec3(0.5, 0.0, 1.0)

    reading: IMUReading = hw.read_imu_converted()

    # Yaw is X (10), inverted -> -10
    assert math.isclose(reading.yaw_rate, -10.0)
    # Roll is Z (20), inverted -> -20
    assert math.isclose(reading.roll_rate, -20.0)
