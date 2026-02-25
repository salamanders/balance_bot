import math
import glm
from unittest.mock import MagicMock
from balance_bot.hardware.robot_hardware import RobotHardware, IMUReading
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.enums import Axis

def test_imu_processing_default(monkeypatch):
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")

    # Setup Config
    hw_config = HardwareConfig(
        motor_l=0,
        motor_r=1,
        accel_vertical_axis=Axis.Z,
        accel_forward_axis=Axis.Y,
        gyro_pitch_axis=Axis.X,
        gyro_yaw_axis=Axis.Z,
        gyro_roll_axis=Axis.Y
    )
    learning_state = LearningState()

    hw = RobotHardware(hw_config, learning_state)

    # Mock the sensor
    hw.sensor = MagicMock()
    # Accel Z = 1G (vertical), others 0
    hw.sensor.get_accel_data.return_value = glm.vec3(0.0, 0.0, 9.8)
    hw.sensor.get_gyro_data.return_value = glm.vec3(0.0, 0.0, 0.0)

    # Default axis is X. Y is forward.
    # Pitch = atan2(acc_y, acc_z) = atan2(0, 9.8) = 0

    reading: IMUReading = hw.read_imu_converted()
    assert math.isclose(reading.pitch_angle, 0.0)
    assert math.isclose(reading.pitch_rate, 0.0)

def test_imu_processing_tilted(monkeypatch):
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")

    hw_config = HardwareConfig(
        motor_l=0,
        motor_r=1,
        accel_vertical_axis=Axis.Z,
        accel_forward_axis=Axis.Y,
        gyro_pitch_axis=Axis.X,
        gyro_yaw_axis=Axis.Z,
        gyro_roll_axis=Axis.Y
    )
    learning_state = LearningState()

    hw = RobotHardware(hw_config, learning_state)
    hw.sensor = MagicMock()

    # Simulate tilt 45 deg forward
    # Y = sin(45)*9.8, Z = cos(45)*9.8
    val = 9.8 * 0.707
    hw.sensor.get_accel_data.return_value = glm.vec3(0.0, val, val)
    hw.sensor.get_gyro_data.return_value = glm.vec3(10.0, 0.0, 0.0)

    reading = hw.read_imu_converted()

    assert math.isclose(reading.pitch_angle, 45.0, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, 10.0)

def test_imu_processing_axis_y(monkeypatch):
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")
    # Axis Y means we use X accel and Y gyro.
    hw_config = HardwareConfig(
        motor_l=0,
        motor_r=1,
        gyro_pitch_axis=Axis.Y,
        accel_forward_axis=Axis.X,
        accel_vertical_axis=Axis.Z,
        gyro_yaw_axis=Axis.Z,
        gyro_roll_axis=Axis.X
    )
    learning_state = LearningState()

    hw = RobotHardware(hw_config, learning_state)
    hw.sensor = MagicMock()

    # Simulate tilt on X axis (which is now pitch)
    val = 9.8 * 0.707
    hw.sensor.get_accel_data.return_value = glm.vec3(val, 0.0, val)
    hw.sensor.get_gyro_data.return_value = glm.vec3(0.0, 5.0, 0.0)

    reading = hw.read_imu_converted()

    assert math.isclose(reading.pitch_angle, 45.0, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, 5.0) # Uses Y gyro

def test_imu_processing_invert(monkeypatch):
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")
    # Invert both pitch angle and gyro rate
    hw_config = HardwareConfig(
        motor_l=0,
        motor_r=1,
        gyro_pitch_invert=True,
        accel_forward_invert=True,
        accel_vertical_axis=Axis.Z,
        accel_forward_axis=Axis.Y,
        gyro_pitch_axis=Axis.X,
        gyro_yaw_axis=Axis.Z,
        gyro_roll_axis=Axis.Y
    )
    learning_state = LearningState()

    hw = RobotHardware(hw_config, learning_state)
    hw.sensor = MagicMock()

    val = 9.8 * 0.707
    hw.sensor.get_accel_data.return_value = glm.vec3(0.0, val, val)
    hw.sensor.get_gyro_data.return_value = glm.vec3(10.0, 0.0, 0.0)

    reading = hw.read_imu_converted()

    # If forward axis is inverted, +val becomes -val.
    # atan2(-val, val) -> -45 deg.
    assert math.isclose(reading.pitch_angle, -45.0, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, -10.0)

def test_imu_processing_sideways(monkeypatch):
    """Test sideways mounting configuration (Vertical X, Forward Y, Gyro Z)"""
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")
    hw_config = HardwareConfig(
        motor_l=0,
        motor_r=1,
        accel_vertical_axis=Axis.X,
        accel_forward_axis=Axis.Y,
        gyro_pitch_axis=Axis.Z,
        gyro_yaw_axis=Axis.X,
        gyro_roll_axis=Axis.Y
    )
    learning_state = LearningState()

    hw = RobotHardware(hw_config, learning_state)
    hw.sensor = MagicMock()

    # Simulate 45 deg tilt.
    # Vertical (X) decreases to cos(45). Forward (Y) increases to sin(45).
    val = 9.8 * 0.707
    hw.sensor.get_accel_data.return_value = glm.vec3(val, val, 0.0)
    hw.sensor.get_gyro_data.return_value = glm.vec3(0.0, 0.0, 5.0)

    reading = hw.read_imu_converted()

    assert math.isclose(reading.pitch_angle, 45.0, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, 5.0)
