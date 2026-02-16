
import sys
import pytest
from unittest.mock import MagicMock, patch, call

# Mock smbus before import if missing
try:
    import smbus
except ImportError:
    sys.modules['smbus'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.hardware.robot_hardware import IMUReading

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.wiring_check.smbus") as mock_smbus_module, \
         patch("balance_bot.wiring_check.RobotHardware") as mock_rh_cls, \
         patch("balance_bot.wiring_check.RobotConfig") as MockConfig:

        config_inst = MagicMock()
        config_inst.motor_i2c_bus = 1
        config_inst.imu_i2c_bus = 1
        config_inst.min_power_visible = 20
        config_inst.control.kickup_power_forward = 0.0
        config_inst.control.kickup_power_backward = 0.0
        MockConfig.load.return_value = config_inst

        # Setup RobotHardware Mock Instance
        mock_hw = mock_rh_cls.return_value

        wc = WiringCheck()
        wc.hw = mock_hw

        # Mock wait_for_stability to avoid infinite loops and side-effect consumption
        wc.wait_for_stability = MagicMock()

        yield wc, mock_hw, config_inst

def test_find_flop_thresholds_success(wc_fixture):
    wc, mock_hw, config_inst = wc_fixture

    # Patch internal method to avoid dealing with IMU loops
    wc._attempt_dynamic_flop = MagicMock()
    # Also patch start condition to avoid IMU reads there if we want, or just mock IMU for it.
    # Let's patch start condition to be simple.
    wc._wait_for_start_condition = MagicMock()

    # Scenario:
    # 1. Forward Flop (BACK)
    #    - Power 30: FAIL_POWER
    #    - Power 35: SUCCESS
    # 2. Backward Flop (FRONT)
    #    - Power 30: FAIL_POWER
    #    - Power 35: SUCCESS

    wc._attempt_dynamic_flop.side_effect = [
        "FAIL_POWER", # BACK, 30
        "SUCCESS",    # BACK, 35
        "FAIL_POWER", # FRONT, 30
        "SUCCESS"     # FRONT, 35
    ]

    with patch("builtins.input", side_effect=Exception("Input called!")), \
         patch("time.sleep"):

        wc.find_flop_thresholds()

    assert config_inst.control.kickup_power_forward == 35
    assert config_inst.control.kickup_power_backward == 35

    # Verify calls to attempt_dynamic_flop
    assert wc._attempt_dynamic_flop.call_count == 4

def test_find_flop_thresholds_initial_pitch_warning(wc_fixture):
    wc, mock_hw, config_inst = wc_fixture

    # Here we WANT to test the start condition logic, so we do NOT patch _wait_for_start_condition.
    # But we MUST patch _attempt_dynamic_flop to avoid the IMU loop crash.
    wc._attempt_dynamic_flop = MagicMock(return_value="SUCCESS")

    with patch("builtins.input", side_effect=Exception("Input called!")), \
         patch("time.sleep"), \
         patch("builtins.print") as mock_print:

        def reading(p):
            r = MagicMock(spec=IMUReading)
            r.pitch_angle = float(p)
            return r

        # Scenario: Robot is too upright at start of forward flop test
        # 1. Forward Flop Start Check (Pitch < -10)
        #    - Reading 1: 0.0 (Fail) -> Should print warning
        #    - Reading 2: -20.0 (OK)
        #    - (Call _attempt_dynamic_flop -> Success)

        # 2. Backward Flop Start Check (Pitch > 10)
        #    - Reading 3: 20.0 (OK)
        #    - (Call _attempt_dynamic_flop -> Success)

        mock_hw.read_imu_converted.side_effect = [
            reading(0.0),   # Fail
            reading(-20.0), # Pass
            reading(20.0)   # Pass (Backward)
        ]

        wc.find_flop_thresholds()

        # Check that warning was printed
        # "[WAITING] Position incorrect (Pitch=0.0). Please adjust."
        assert any("Position incorrect (Pitch=0.0)" in str(c) for c in mock_print.call_args_list)

        # Verify attempts
        assert wc._attempt_dynamic_flop.call_count == 2
