import sys
import pytest
from unittest.mock import MagicMock, patch

if 'smbus' not in sys.modules:
    sys.modules['smbus'] = MagicMock()

from balance_bot.wiring_check import WiringCheck, MeasureResult
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
        wc.config.accel_vertical_axis = None
        yield wc

def test_calibrate_orientation(wc_fixture):
    wc = wc_fixture

    back_vec = Vector3(0.1, 0.2, -9.8)
    front_vec = Vector3(0.1, 9.8, 0.2)

    # We mock collect_data to return pre-calculated results
    with patch.object(wc, 'collect_data') as mock_collect, \
         patch("builtins.input") as mock_input:

        wc.wait_for_stability = MagicMock()

        # Result 1: Back
        r1 = MagicMock(spec=MeasureResult)
        r1.avg_accel_raw = back_vec

        # Result 2: Front
        r2 = MagicMock(spec=MeasureResult)
        r2.avg_accel_raw = front_vec

        mock_collect.side_effect = [r1, r2]

        wc.calibrate_static_orientation()

        wc.wait_for_stability.assert_called_once()
        assert mock_input.call_count == 1
        assert mock_collect.call_count == 2

    # Verify Vertical Axis = Z (from back_vec z=-9.8)
    assert wc.config.accel_vertical_axis == Axis.Z

    # Verify Pitch Axis = X (from Cross Product logic)
    assert wc.config.gyro_pitch_axis == Axis.X
