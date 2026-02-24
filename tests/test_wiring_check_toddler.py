import sys
import pytest
from unittest.mock import MagicMock, patch
import glm

# Mock smbus2 before import
if 'smbus2' not in sys.modules:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.hardware.robot_hardware import MeasureResult, IMUReading

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.utils.smbus"), \
         patch("balance_bot.wiring_check.RobotHardware") as MockHW, \
         patch("balance_bot.wiring_check.HardwareConfig") as MockHWConfig, \
         patch("balance_bot.wiring_check.LearningState") as MockLearningState:

        # Setup Configs
        hw_config = MagicMock()
        hw_config.motor_l = 0
        hw_config.motor_r = 1
        # Set default invert flags to False so NOT invert toggles them
        hw_config.motor_r_invert = False

        MockHWConfig.load.return_value = hw_config
        hw_config.model_copy.return_value = hw_config

        learning_state = MagicMock()
        learning_state.min_power_visible = 0
        MockLearningState.load.return_value = learning_state

        wc = WiringCheck()
        wc.hw = MockHW.return_value
        # Mock get_mapped_value to avoid formatting errors
        wc.hw.get_mapped_value.return_value = 0.0
        wc.hw.wait_for_stability = MagicMock()

        yield wc, wc.hw, learning_state, hw_config

def test_find_min_power_raw_success(wc_fixture):
    wc, mock_hw, learning_state, _ = wc_fixture

    # 1. First call (p=10): Small magnitude
    res_low = MeasureResult(duration=0.3, samples=[
        IMUReading(0,0,0,0,0, accel_raw=None, gyro_raw=glm.vec3(1,1,1)) # Mag ~1.7
    ])

    # 2. Second call (p=15): Big magnitude
    res_high = MeasureResult(duration=0.3, samples=[
        IMUReading(0,0,0,0,0, accel_raw=None, gyro_raw=glm.vec3(10,10,10)) # Mag ~17.3
    ])

    mock_hw.drive_and_measure.side_effect = [res_low, res_high]

    # Run
    wc.find_min_power()

    # Expect found value should be 15.
    assert learning_state.min_power_visible == 15
    assert learning_state.save.called

def test_align_motors_phase_raw_straight(wc_fixture):
    """Test case where motors are aligned (perpendicular vectors)."""
    wc, mock_hw, learning_state, _ = wc_fixture
    learning_state.min_power_visible = 20

    # Setup drive_and_measure result
    gravity = glm.vec3(0, 0, 9.8)
    gyro = glm.vec3(10, 0, 0)

    # Need fwd accel > 0.1
    # Mock get_mapped_value to return 0.2 when asking for accel_forward
    # But return 0.0 for baseline (which has z=1 in our manual construction below)
    def get_mapped_val(v, name):
        if name == "accel_forward":
            if v and v.z == 1.0: return 0.0
            return 0.2
        return 0.0
    mock_hw.get_mapped_value.side_effect = get_mapped_val

    res = MeasureResult(duration=0.5, samples=[
        IMUReading(0,0,0,0,0, accel_raw=gravity, gyro_raw=gyro)
    ])

    mock_hw.drive_and_measure.return_value = res

    # Mock verify_with_retries to just run test and verify once
    with patch("balance_bot.wiring_check.verify_with_retries") as mock_verify:
        wc.align_motors_phase()

        # Verify call args
        test_cb = mock_verify.call_args[0][1]
        verify_cb = mock_verify.call_args[0][2]

        # Run callbacks
        test_cb(0) # Should call drive_and_measure with attempt 0
        # Power = 20 + 20 + 0 = 40
        mock_hw.drive_and_measure.assert_called_with(40, 40, 0.5, wait_for_stability=False)

        test_cb(1) # Should call drive_and_measure with attempt 1 (Ramping)
        # Power = 20 + 20 + 10 = 50
        mock_hw.drive_and_measure.assert_called_with(50, 50, 0.5, wait_for_stability=False)

        # Verify expects (baseline, res)
        baseline = IMUReading(0,0,0,0,0, accel_raw=glm.vec3(0,0,1), gyro_raw=glm.vec3(0,0,0))
        result = verify_cb((baseline, res)) # Should check alignment

        assert result is True
        assert learning_state.motor_phasing_verified == True
        assert learning_state.save.called

def test_align_motors_phase_raw_spinning(wc_fixture):
    """Test case where motors are fighting (aligned vectors)."""
    wc, mock_hw, learning_state, hw_config = wc_fixture
    learning_state.min_power_visible = 20

    # Setup drive_and_measure result
    gravity = glm.vec3(0, 0, 9.8)
    gyro = glm.vec3(0, 0, 10)

    # Spinning means Yaw Rate > 30.0
    res = MeasureResult(duration=0.5, samples=[
        IMUReading(0,0, 40.0, 0,0, accel_raw=gravity, gyro_raw=gyro)
    ])

    mock_hw.drive_and_measure.return_value = res

    # Reset side effect for this test
    mock_hw.get_mapped_value.side_effect = None
    mock_hw.get_mapped_value.return_value = 0.0

    with patch("balance_bot.wiring_check.verify_with_retries") as mock_verify:
        wc.align_motors_phase()

        verify_cb = mock_verify.call_args[0][2]
        baseline = IMUReading(0,0,0,0,0, accel_raw=glm.vec3(0,0,1), gyro_raw=glm.vec3(0,0,0))
        result = verify_cb((baseline, res))

        assert result is False

        # Check if config was updated
        assert hw_config.model_copy.called
        found = False
        for call in hw_config.model_copy.call_args_list:
            if 'motor_r_invert' in call.kwargs.get('update', {}):
                found = True
        assert found
