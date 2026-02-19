import sys
import pytest
from unittest.mock import MagicMock, patch

# Mock smbus before import
if 'smbus' not in sys.modules:
    sys.modules['smbus'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.utils import Vector3
from balance_bot.enums import Axis

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.wiring_check.smbus"), \
         patch("balance_bot.wiring_check.RobotHardware"), \
         patch("balance_bot.wiring_check.RobotConfig") as MockConfig:

        wc = WiringCheck()
        wc.hw = MagicMock()
        wc.config = MagicMock()
        # Defaults
        wc.config.accel_vertical_axis = None

        yield wc

def test_calibrate_orientation(wc_fixture):
    wc = wc_fixture

    # 1. Back (Vertical Gravity)
    # Simulate resting on back: Gravity on Z (negative or positive depending on mounting)
    # Say Z = -9.8 (Gravity down, Z up)
    # Forward (Y) = small

    back_vec = Vector3(0.1, 0.2, -9.8)
    front_vec = Vector3(0.1, 9.8, 0.2) # Tipped forward -> Gravity on Y

    # We need to handle 2 loops of 50 samples
    # We'll just make read_imu_raw return back_vec initially, then front_vec
    # call_count increments every call.

    # Reset mock to count from 0
    wc.hw.read_imu_raw.reset_mock()

    def side_effect():
        # First 50 calls are for Back
        # Then some logic runs
        # Then 50 calls for Front
        if wc.hw.read_imu_raw.call_count <= 55:
             return back_vec, Vector3(0,0,0)
        return front_vec, Vector3(0,0,0)

    wc.hw.read_imu_raw.side_effect = side_effect

    # Mock input to avoid waiting
    with patch("builtins.input") as mock_input:
        original_hw_mock = wc.hw
        wc.calibrate_static_orientation()

        # We expect wait_for_stability instead of the first input
        original_hw_mock.wait_for_stability.assert_called_once()
        # We expect input only for the second interaction (tipping forward)
        assert mock_input.call_count == 1

    # Verify Vertical Axis = Z
    # analyze_dominance on back_vec (z=-9.8) -> Winner Z
    assert wc.config.accel_vertical_axis == Axis.Z

    # Verify Pitch Axis
    # Back = {x:0.1, y:0.2, z:-9.8} (Approx Z-)
    # Front = {x:0.1, y:9.8, z:0.2} (Approx Y+)
    # Cross Product: Back x Front
    # x = 0.2*0.2 - (-9.8)*9.8 = 0.04 + 96.04 = 96.08
    # y = -9.8*0.1 - 0.1*0.2 = -0.98 - 0.02 = -1.0
    # z = 0.1*9.8 - 0.2*0.1 = 0.98 - 0.02 = 0.96
    # Dominant is X.

    assert wc.config.gyro_pitch_axis == Axis.X
