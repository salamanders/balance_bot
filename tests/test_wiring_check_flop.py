
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

    # Mock user input just in case, but it shouldn't be called
    with patch("builtins.input", side_effect=Exception("Input called!")), \
         patch("time.sleep"), \
         patch.object(wc, 'drive_and_measure', return_value=[]):

        # Helper to create IMU reading
        def reading(p):
            r = MagicMock(spec=IMUReading)
            r.pitch_angle = float(p)
            return r

        # Side effect sequence
        # 1. Forward Flop (Start Check: Pitch < -10)
        #    - Reading 1: -30 (OK)
        #    - Power 30 -> Drive
        #    - Reading 2: -20 (Fail, > 10 required for success)
        #    - Reset Check: Pitch < -10
        #    - Reading 3: -30 (OK)
        #    - Power 35 -> Drive
        #    - Reading 4: 20 (Success, > 10)

        # 2. Backward Flop (Start Check: Pitch > 10)
        #    - Reading 5: 20 (OK)
        #    - Power 30 -> Drive
        #    - Reading 6: 20 (Fail, < -10 required for success)
        #    - Reset Check: Pitch > 10
        #    - Reading 7: 20 (OK)
        #    - Power 35 -> Drive
        #    - Reading 8: -20 (Success, < -10)

        mock_hw.read_imu_converted.side_effect = [
            reading(-30),
            reading(-20),
            reading(-30),
            reading(20),
            reading(20),
            reading(20),
            reading(20),
            reading(-20)
        ]

        wc.find_flop_thresholds()

        assert config_inst.control.kickup_power_forward == 35
        assert config_inst.control.kickup_power_backward == 35

def test_find_flop_thresholds_initial_pitch_warning(wc_fixture):
    wc, mock_hw, config_inst = wc_fixture

    with patch("builtins.input", side_effect=Exception("Input called!")), \
         patch("time.sleep"), \
         patch("builtins.print") as mock_print, \
         patch.object(wc, 'drive_and_measure', return_value=[]):

        def reading(p):
            r = MagicMock(spec=IMUReading)
            r.pitch_angle = float(p)
            return r

        # Scenario: Robot is too upright at start of forward flop test
        # 1. Forward Flop Start Check (Pitch < -10)
        #    - Reading 1: 0.0 (Fail) -> Should print warning
        #    - Reading 2: -20.0 (OK)
        #    - Power 30 -> Drive
        #    - Reading 3: 20.0 (Success)

        # 2. Backward Flop Start Check (Pitch > 10)
        #    - Reading 4: 20.0 (OK)
        #    - Power 30 -> Drive
        #    - Reading 5: -20.0 (Success)

        mock_hw.read_imu_converted.side_effect = [
            reading(0.0),
            reading(-20.0),
            reading(20.0),
            reading(20.0),
            reading(-20.0)
        ]

        wc.find_flop_thresholds()

        # Check that warning was printed
        # "[WAITING] Position incorrect (Pitch=0.0). Please adjust."
        assert any("Position incorrect (Pitch=0.0)" in str(c) for c in mock_print.call_args_list)
