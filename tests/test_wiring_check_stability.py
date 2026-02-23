import sys
import pytest
from unittest.mock import MagicMock, patch, call
import time

# Mock smbus2 before import if missing
if 'smbus2' not in sys.modules:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.hardware.robot_hardware import IMUReading
from balance_bot.configuration import RobotConfig, PIDParams
from balance_bot.utils import Vector3

import os
from balance_bot.hardware.robot_hardware import RobotHardware

@pytest.fixture
def hw_fixture():
    # Use Mock Fallback to avoid import errors and dependency on real libs
    os.environ["ALLOW_MOCK_FALLBACK"] = "1"

    config = RobotConfig(pid=PIDParams())
    config.motor_i2c_bus = 1
    config.imu_i2c_bus = 1

    hw = RobotHardware(config)

    # Mock the method we depend on
    hw.read_imu_raw = MagicMock()
    return hw

def test_wait_for_stability_success(hw_fixture):
    hw = hw_fixture

    moving_gyro = Vector3(10.0, 10.0, 10.0)
    stable_gyro = Vector3(0.1, 0.1, 0.1)
    dummy_accel = Vector3(0.0, 0.0, 1.0)

    hw.read_imu_raw.side_effect = [
        (dummy_accel, moving_gyro),
        (dummy_accel, stable_gyro),
        (dummy_accel, stable_gyro),
        (dummy_accel, stable_gyro),
        (dummy_accel, stable_gyro)
    ]

    start_time = 1000.0
    def time_gen():
        nonlocal start_time
        start_time += 0.1
        return start_time

    with patch("time.sleep") as mock_sleep, \
         patch("time.time", side_effect=time_gen) as mock_time:

        # Duration 0.15s should pass with 2 stable readings at 0.1s interval
        hw.wait_for_stability(duration=0.15, threshold=5.0)

    assert hw.read_imu_raw.call_count >= 3

def test_wait_for_stability_interrupted(hw_fixture):
    hw = hw_fixture

    moving_gyro = Vector3(10.0, 10.0, 10.0)
    stable_gyro = Vector3(0.1, 0.1, 0.1)
    dummy_accel = Vector3(0.0, 0.0, 1.0)

    # Sequence:
    # 1. Stable (Start Timer)
    # 2. Moving (Reset Timer)
    # 3. Stable (Start Timer)
    # 4. Stable (Check Timer - Done)
    hw.read_imu_raw.side_effect = [
        (dummy_accel, stable_gyro),
        (dummy_accel, moving_gyro),
        (dummy_accel, stable_gyro),
        (dummy_accel, stable_gyro),
        (dummy_accel, stable_gyro),
        (dummy_accel, stable_gyro)
    ]

    start_time = 1000.0
    def time_gen():
        nonlocal start_time
        start_time += 0.1
        return start_time

    with patch("time.sleep") as mock_sleep, \
         patch("time.time", side_effect=time_gen) as mock_time:

        hw.wait_for_stability(duration=0.15, threshold=5.0)

    # Should have called read at least 4 times
    assert hw.read_imu_raw.call_count >= 4
