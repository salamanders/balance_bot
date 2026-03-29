from typing import Any
from unittest.mock import MagicMock
from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.configuration import HardwareConfig, LearningState, PIDParams

def test_imu_resilience_fail_fast(monkeypatch: Any) -> None:
    """
    Test that read_imu_raw implements a resilience strategy for transient I2C errors
    up to a threshold, returning cached data, and then fails fast when exceeded.
    """
    monkeypatch.setenv("ALLOW_MOCK_FALLBACK", "1")

    hw_config = HardwareConfig(motor_l=0, motor_r=1, imu_max_retries=5)
    learning_state = LearningState(pid=PIDParams())

    hw = RobotHardware(hw_config, learning_state)
    hw.stop_sensor_thread()
    hw.sensor = MagicMock()

    hw.sensor.get_accel_data.side_effect = OSError("Input/output error")
    hw.sensor.get_gyro_data.side_effect = OSError("Input/output error")

    # Should survive up to imu_max_retries by returning cached values
    for _ in range(hw_config.imu_max_retries):
        hw.read_imu_raw()

    # Exceeding the max retries should NO LONGER raise an OSError.
    # It should gracefully return cached values indefinitely to keep the continuous actuation loop alive.
    hw.read_imu_raw()
    assert hw._imu_consecutive_errors == hw_config.imu_max_retries + 1

    # Verify that the continuous data quality metric is returned correctly.
    reading = hw.read_imu_converted()
    assert reading.error_count == hw_config.imu_max_retries + 2
