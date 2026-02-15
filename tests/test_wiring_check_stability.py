import sys
import pytest
from unittest.mock import MagicMock, patch, call
import time

# Mock smbus before import if missing
if 'smbus' not in sys.modules:
    sys.modules['smbus'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.hardware.robot_hardware import IMUReading

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.wiring_check.smbus"), \
         patch("balance_bot.wiring_check.RobotHardware"), \
         patch("balance_bot.wiring_check.RobotConfig") as MockConfig:

        wc = WiringCheck()
        wc.hw = MagicMock()
        wc.config = MagicMock()
        yield wc

def test_wait_for_stability_success(wc_fixture):
    wc = wc_fixture

    moving = IMUReading(
        pitch_angle=0.0, pitch_rate=10.0, yaw_rate=10.0, roll_angle=0.0, roll_rate=10.0
    )
    stable = IMUReading(
        pitch_angle=0.0, pitch_rate=0.1, yaw_rate=0.1, roll_angle=0.0, roll_rate=0.1
    )

    wc.hw.read_imu_converted.side_effect = [moving, stable, stable, stable, stable]

    start_time = 1000.0
    def time_gen():
        nonlocal start_time
        start_time += 0.1
        return start_time

    with patch("time.sleep") as mock_sleep, \
         patch("time.time", side_effect=time_gen) as mock_time:

        # Duration 0.15s should pass with 2 stable readings at 0.1s interval
        wc.wait_for_stability(duration=0.15, threshold=5.0)

    assert wc.hw.read_imu_converted.call_count >= 3

def test_wait_for_stability_interrupted(wc_fixture):
    wc = wc_fixture

    moving = IMUReading(
        pitch_angle=0.0, pitch_rate=10.0, yaw_rate=10.0, roll_angle=0.0, roll_rate=10.0
    )
    stable = IMUReading(
        pitch_angle=0.0, pitch_rate=0.1, yaw_rate=0.1, roll_angle=0.0, roll_rate=0.1
    )

    # Sequence:
    # 1. Stable (Start Timer)
    # 2. Moving (Reset Timer)
    # 3. Stable (Start Timer)
    # 4. Stable (Check Timer - Done)
    wc.hw.read_imu_converted.side_effect = [stable, moving, stable, stable, stable, stable]

    start_time = 1000.0
    def time_gen():
        nonlocal start_time
        start_time += 0.1
        return start_time

    with patch("time.sleep") as mock_sleep, \
         patch("time.time", side_effect=time_gen) as mock_time:

        wc.wait_for_stability(duration=0.15, threshold=5.0)

    # Should have called read at least 4 times
    assert wc.hw.read_imu_converted.call_count >= 4
