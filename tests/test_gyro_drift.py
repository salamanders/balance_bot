from typing import Generator
from typing import Any
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

@pytest.fixture()  # type: ignore[untyped-decorator]
def hw_fixture() -> Generator[Any, None, None]:
    os.environ["ALLOW_MOCK_FALLBACK"] = "1"
    hw_config = HardwareConfig(motor_i2c_bus=1, imu_i2c_bus=1)
    learning_state = LearningState(pid=PIDParams())
    hw = RobotHardware(hw_config, learning_state)

    # We must NOT mock read_imu_raw directly, because the fix is inside read_imu_raw logic (bias application).
    # Instead, we mock the underlying sensor driver.
    hw.sensor = MagicMock()

    yield hw

def test_wait_for_stability_fixes_bias(hw_fixture: Any) -> None:
    """
    Verifies that wait_for_stability detects bias and auto-calibrates.
    """
    hw = hw_fixture

    # Simulate a constant bias of 3.0 deg/s on X axis from the SENSOR.
    biased_gyro_raw = glm.vec3(3.0, 0.0, 0.0)
    dummy_accel = glm.vec3(0.0, 0.0, 1.0)

    # Configure the sensor mock to return biased raw data
    hw.sensor.get_accel_data.return_value = dummy_accel
    hw.sensor.get_gyro_data.return_value = biased_gyro_raw

    # Check initial config bias
    assert hw.learning_state.gyro_bias_x == 0.0

    # Run wait_for_stability.
    # It should:
    # 1. Read 3.0 (which is > threshold 2.0).
    # 2. Wait 1 second (approx 20 loops).
    # 3. Detect stability (variance ~0).
    # 4. Update bias to +3.0.
    # 5. Subsequent reads will be (3.0 - 3.0) = 0.0.
    # 6. Detect stable rate < 2.0.
    # 7. Wait duration (1.0s).
    # 8. Return.

    # Speed up time.sleep
    with patch("time.sleep", return_value=None):
        try:
             hw.wait_for_stability(duration=1.0, threshold=2.0)
        except KeyboardInterrupt:
             pytest.fail("Infinite loop detected! Auto-calibration failed.")

    # Assert Bias was updated
    # It should be exactly 3.0 (or very close)
    assert hw.learning_state.gyro_bias_x == pytest.approx(3.0, abs=0.1)

    # Assert we can now read clean data
    accel, gyro = hw.read_imu_raw()
    assert gyro.x == pytest.approx(0.0, abs=0.1)
