import pytest
from unittest.mock import MagicMock
from balance_bot.config import RobotConfig, PIDParams
from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.utils import Vector3

def test_imu_resilience_zero_order_hold(monkeypatch):
    """
    Test that read_imu_raw implements a Zero-Order Hold strategy for transient I2C errors.
    It should catch OSError and return the last known good value.
    """
    # Force mock mode to avoid import errors for hardware libs
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")

    config = RobotConfig(pid=PIDParams(), motor_l=0, motor_r=1)
    hw = RobotHardware(config)
    # Replaces the internal sensor with a MagicMock for control
    hw.sensor = MagicMock()

    # --- Case 1: Initial Failure (No data yet) ---
    # We simulate an immediate I2C error.
    # The handler should return the initialized defaults (zeros).
    hw.sensor.get_accel_data.side_effect = OSError("Input/output error")
    hw.sensor.get_gyro_data.side_effect = OSError("Input/output error")

    # This calls read_imu_raw which should catch the error
    accel, gyro = hw.read_imu_raw()

    # Verify defaults
    assert accel == Vector3(0.0, 0.0, 0.0)
    assert gyro == Vector3(0.0, 0.0, 0.0)

    # --- Case 2: Successful Read ---
    hw.sensor.get_accel_data.side_effect = None
    hw.sensor.get_gyro_data.side_effect = None
    hw.sensor.get_accel_data.return_value = Vector3(1.0, 2.0, 3.0)
    hw.sensor.get_gyro_data.return_value = Vector3(4.0, 5.0, 6.0)

    accel, gyro = hw.read_imu_raw()

    # Verify we got the new values
    assert accel == Vector3(1.0, 2.0, 3.0)
    assert gyro == Vector3(4.0, 5.0, 6.0)

    # --- Case 3: Transient Failure (Zero-Order Hold) ---
    # Simulate another I2C error.
    hw.sensor.get_accel_data.side_effect = OSError("Input/output error")

    # Should return the values from Case 2 (Last Known Good)
    accel, gyro = hw.read_imu_raw()

    assert accel == Vector3(1.0, 2.0, 3.0)
    assert gyro == Vector3(4.0, 5.0, 6.0)
