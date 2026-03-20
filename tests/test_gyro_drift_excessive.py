import pytest
from unittest.mock import MagicMock, patch
import sys
import os

# Mock smbus2 before import if missing
if 'smbus2' not in sys.modules:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.configuration import HardwareConfig, LearningState, PIDParams
import glm

@pytest.fixture
def hw_fixture():
    os.environ["ALLOW_MOCK_FALLBACK"] = "1"
    hw_config = HardwareConfig(motor_i2c_bus=1, imu_i2c_bus=1)
    learning_state = LearningState(pid=PIDParams())
    hw = RobotHardware(hw_config, learning_state)
    hw.sensor = MagicMock()
    return hw

def test_wait_for_stability_excessive_bias(hw_fixture):
    """
    Verifies that wait_for_stability raises an error when bias is excessively large.
    """
    hw = hw_fixture

    # Simulate an excessive bias of 500.0 deg/s on X axis from the SENSOR.
    biased_gyro_raw = glm.vec3(500.0, 0.0, 0.0)
    dummy_accel = glm.vec3(0.0, 0.0, 1.0)

    # Configure the sensor mock to return biased raw data
    hw.sensor.get_accel_data.return_value = dummy_accel
    hw.sensor.get_gyro_data.return_value = biased_gyro_raw

    # Speed up time.sleep
    with patch("time.sleep", return_value=None):
        with pytest.raises(RuntimeError, match="Excessive gyro drift"):
             hw.wait_for_stability(duration=1.0, threshold=2.0)
