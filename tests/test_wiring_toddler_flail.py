import sys
import pytest
import time
from unittest.mock import MagicMock, patch, ANY

# Mock smbus2 before import
if 'smbus2' not in sys.modules:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.utils import Vector3
from balance_bot.hardware.robot_hardware import MeasureResult, IMUReading

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.utils.smbus"), \
         patch("balance_bot.wiring_check.RobotHardware") as MockHW, \
         patch("balance_bot.wiring_check.HardwareConfig") as MockHWConfig, \
         patch("balance_bot.wiring_check.LearningState") as MockLearningState:

        # Setup Configs
        hw_config = MagicMock()
        MockHWConfig.load.return_value = hw_config

        learning_state = MagicMock()
        learning_state.min_power_visible = 10
        MockLearningState.load.return_value = learning_state

        wc = WiringCheck()
        wc.hw = MockHW.return_value
        wc.hw.wait_for_stability = MagicMock()

        yield wc, wc.hw, learning_state

def test_toddler_flail_collection_loop(wc_fixture):
    wc, mock_hw, _ = wc_fixture

    # Mock time to control loop execution
    # Sequence of time.time() calls:
    # 1. Start time capture
    # 2. Loop condition check 1 (0.0 < 10.0) -> Enter
    # 3. Loop condition check 2 (11.0 > 10.0) -> Exit
    with patch('time.time', side_effect=[100.0, 100.1, 111.0]):

        # Mock measurement return
        # _measure_gravity_with_hardware calls drive_and_measure
        # flail calls drive_and_measure
        mock_hw.drive_and_measure.return_value = MeasureResult(0.1, [])

        # Mock _measure_gravity_with_hardware to return a dummy vector
        # because the real one digs into res.samples which we mocked as empty
        with patch.object(wc, '_measure_gravity_with_hardware', return_value=Vector3(0,0,1)):
            vectors = wc._toddler_flail_collection(duration=10.0)

        assert len(vectors) == 1
        # drive_and_measure called for flail
        # wait_for_stability called
        assert mock_hw.drive_and_measure.called
        assert wc.hw.wait_for_stability.called

def test_measure_gravity_vectors_success(wc_fixture):
    wc, _, _ = wc_fixture

    v1 = Vector3(1, 0, 0)
    v2 = Vector3(0, 1, 0) # 90 degrees apart

    # Mock _toddler_flail_collection
    with patch.object(wc, '_toddler_flail_collection', return_value=[v1, v2]), \
         patch("balance_bot.wiring_check.sort_resting_vectors", return_value=(v1, v2)):

        p1, p2 = wc._measure_gravity_vectors()

        assert p1 == v1
        assert p2 == v2

def test_measure_gravity_vectors_failsafe(wc_fixture):
    wc, _, _ = wc_fixture

    v1 = Vector3(1, 0, 0)
    v2 = Vector3(0.99, 0.1, 0) # Very close (angle < 15)

    with patch.object(wc, '_toddler_flail_collection', return_value=[v1, v2]), \
         patch("balance_bot.wiring_check.sort_resting_vectors", return_value=(v1, v2)):

        with pytest.raises(RuntimeError) as excinfo:
            wc._measure_gravity_vectors()

        assert "Resting vectors are too close" in str(excinfo.value)

def test_measure_gravity_vectors_sort_failure(wc_fixture):
    wc, _, _ = wc_fixture

    with patch.object(wc, '_toddler_flail_collection', return_value=[]), \
         patch("balance_bot.wiring_check.sort_resting_vectors", side_effect=ValueError("Sort failed")):

        with pytest.raises(ValueError):
             wc._measure_gravity_vectors()
