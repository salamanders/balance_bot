from typing import Any
from unittest.mock import MagicMock
from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.configuration import HardwareConfig, LearningState, PIDParams
from balance_bot.enums import Axis
import glm
import math

def test_imu_yaw_roll_defaults(monkeypatch: Any) -> None:
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")
    # Defaults: Yaw=Z, Roll=Y. AccelRoll=X (since Vert=Z, Fwd=Y).

    hw_config = HardwareConfig(
        motor_l=0,
        motor_r=1,
        gyro_pitch_axis=Axis.X,
        accel_vertical_axis=Axis.Z,
        accel_forward_axis=Axis.Y,
        gyro_yaw_axis=Axis.Z,
        gyro_roll_axis=Axis.Y
    )
    learning_state = LearningState(pid=PIDParams())

    hw = RobotHardware(hw_config, learning_state)
    hw.stop_sensor_thread()
    hw.sensor = MagicMock()

    # Simulate:
    # Yaw (Z) = 10 deg/s
    # Roll (Y) = 20 deg/s
    # Pitch (X) = 0
    hw.sensor.get_gyro_data.return_value = glm.vec3(0.0, 20.0, 10.0)
    # Accel: Vertical(Z)=1g. Roll(X)=0.5g. Forward(Y)=0.
    # Roll Angle = atan2(X, Z) = atan2(0.5, 1.0) approx 26.5 deg
    hw.sensor.get_accel_data.return_value = glm.vec3(0.5, 0.0, 1.0)

    reading = hw.read_imu_converted()

    assert math.isclose(reading.yaw_rate, 10.0)
    assert math.isclose(reading.roll_rate, 20.0)
    assert math.isclose(reading.roll_angle, math.degrees(math.atan2(0.5, 1.0)), abs_tol=0.1)

def test_imu_yaw_roll_custom_axis_invert(monkeypatch: Any) -> None:
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")
    # Set Yaw to X (invert), Roll to Z (invert)
    # Gyro Pitch default is X, so we can't use X for Yaw unless we change Pitch.
    # Let's say: Pitch=Y, Yaw=X, Roll=Z.

    hw_config = HardwareConfig(
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
    learning_state = LearningState(pid=PIDParams())

    # Accel Roll Axis will be deduced as X.

    hw = RobotHardware(hw_config, learning_state)
    hw.stop_sensor_thread()
    hw.sensor = MagicMock()

    hw.sensor.get_gyro_data.return_value = glm.vec3(10.0, 0.0, 20.0)
    # Accel: Roll(X) = 0.5. Vert(Z) = 1.0.
    hw.sensor.get_accel_data.return_value = glm.vec3(0.5, 0.0, 1.0)

    reading = hw.read_imu_converted()

    # Yaw is X (10), inverted -> -10
    assert math.isclose(reading.yaw_rate, -10.0)
    # Roll is Z (20), inverted -> -20
    assert math.isclose(reading.roll_rate, -20.0)
