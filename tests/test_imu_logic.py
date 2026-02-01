import math
from unittest.mock import MagicMock
from balance_bot.robot_hardware import RobotHardware, IMUReading

# We need to mock the imports inside RobotHardware
# But RobotHardware mocks them if ImportError.
# We can force mock mode by setting env var if needed, or relying on the fact that MPU6050 isn't installed.
# But we are in a sandbox where dependencies might be installed.
# The code checks for mocks.py if ImportError.
# Let's force MOCK_HARDWARE env var.

def test_imu_processing_default(monkeypatch):
    monkeypatch.setenv("MOCK_HARDWARE", "1")

    hw = RobotHardware(0, 1)

    # Mock the sensor
    hw.sensor = MagicMock()
    # Accel Z = 1G (vertical), others 0
    hw.sensor.get_accel_data.return_value = {"x": 0.0, "y": 0.0, "z": 9.8}
    hw.sensor.get_gyro_data.return_value = {"x": 0.0, "y": 0.0, "z": 0.0}

    # Default axis is X. Y is forward.
    # Pitch = atan2(acc_y, acc_z) = atan2(0, 9.8) = 0

    reading: IMUReading = hw.read_imu_converted()
    assert math.isclose(reading.pitch_angle, 0.0)
    assert math.isclose(reading.pitch_rate, 0.0)

def test_imu_processing_tilted(monkeypatch):
    monkeypatch.setenv("MOCK_HARDWARE", "1")
    hw = RobotHardware(0, 1)
    hw.sensor = MagicMock()

    # Simulate tilt 45 deg forward
    # Y = sin(45)*9.8, Z = cos(45)*9.8
    val = 9.8 * 0.707
    hw.sensor.get_accel_data.return_value = {"x": 0.0, "y": val, "z": val}
    hw.sensor.get_gyro_data.return_value = {"x": 10.0, "y": 0.0, "z": 0.0}

    reading = hw.read_imu_converted()

    assert math.isclose(reading.pitch_angle, 45.0, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, 10.0)

def test_imu_processing_axis_y(monkeypatch):
    monkeypatch.setenv("MOCK_HARDWARE", "1")
    # Axis Y means we use X accel and Y gyro. In new API, we must specify accel_forward_axis.
    hw = RobotHardware(0, 1, gyro_axis="y", accel_forward_axis="x")
    hw.sensor = MagicMock()

    # Simulate tilt on X axis (which is now pitch)
    val = 9.8 * 0.707
    hw.sensor.get_accel_data.return_value = {"x": val, "y": 0.0, "z": val}
    hw.sensor.get_gyro_data.return_value = {"x": 0.0, "y": 5.0, "z": 0.0}

    reading = hw.read_imu_converted()

    assert math.isclose(reading.pitch_angle, 45.0, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, 5.0) # Uses Y gyro

def test_imu_processing_invert(monkeypatch):
    monkeypatch.setenv("MOCK_HARDWARE", "1")
    # Invert both pitch angle and gyro rate
    hw = RobotHardware(0, 1, gyro_invert=True, accel_forward_invert=True)
    hw.sensor = MagicMock()

    val = 9.8 * 0.707
    hw.sensor.get_accel_data.return_value = {"x": 0.0, "y": val, "z": val}
    hw.sensor.get_gyro_data.return_value = {"x": 10.0, "y": 0.0, "z": 0.0}

    reading = hw.read_imu_converted()

    # If forward axis is inverted, +val becomes -val.
    # atan2(-val, val) -> -45 deg.
    assert math.isclose(reading.pitch_angle, -45.0, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, -10.0)

def test_imu_processing_sideways(monkeypatch):
    """Test sideways mounting configuration (Vertical X, Forward Y, Gyro Z)"""
    monkeypatch.setenv("MOCK_HARDWARE", "1")
    hw = RobotHardware(
        0, 1,
        accel_vertical_axis="x",
        accel_forward_axis="y",
        gyro_axis="z"
    )
    hw.sensor = MagicMock()

    # Simulate 45 deg tilt.
    # Vertical (X) decreases to cos(45). Forward (Y) increases to sin(45).
    val = 9.8 * 0.707
    hw.sensor.get_accel_data.return_value = {"x": val, "y": val, "z": 0.0}
    hw.sensor.get_gyro_data.return_value = {"x": 0.0, "y": 0.0, "z": 5.0}

    reading = hw.read_imu_converted()

    assert math.isclose(reading.pitch_angle, 45.0, abs_tol=0.1)
    assert math.isclose(reading.pitch_rate, 5.0)
