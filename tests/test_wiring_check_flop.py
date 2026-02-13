import sys
import pytest
from unittest.mock import MagicMock, patch, call

# Mock smbus before import if missing
try:
    import smbus
except ImportError:
    sys.modules['smbus'] = MagicMock()

from balance_bot.wiring_check import WiringCheck

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

        yield wc, mock_hw, config_inst

def test_find_flop_thresholds_success(wc_fixture):
    wc, mock_hw, config_inst = wc_fixture

    # Mock user input to always be "enter"
    with patch("builtins.input", return_value=""), \
         patch("time.sleep"), \
         patch.object(wc, 'drive_and_measure', return_value=[]):

        # We need to simulate the sequence of IMU readings.
        # Flow:
        # 1. Forward Flop:
        #    - Initial read (check if back enough) -> pitch = -30 (ok)
        #    - Power 30 loop:
        #      - drive (mocked)
        #      - read imu -> pitch = -20 (fail, not > 10)
        #    - Power 35 loop:
        #      - drive (mocked)
        #      - read imu -> pitch = 20 (success)
        # 2. Backward Flop:
        #    - Power 30 loop:
        #      - drive (mocked)
        #      - read imu -> pitch = 20 (fail, not < -10)
        #    - Power 35 loop:
        #      - drive (mocked)
        #      - read imu -> pitch = -20 (success)

        # Let's setup side_effect for read_imu_converted

        reading_back_ok = MagicMock(pitch_angle=-30.0)
        reading_fwd_fail = MagicMock(pitch_angle=-20.0)
        reading_fwd_success = MagicMock(pitch_angle=20.0)

        reading_bwd_fail = MagicMock(pitch_angle=20.0)
        reading_bwd_success = MagicMock(pitch_angle=-20.0)

        # Sequence of calls:
        # 1. Forward Flop Initial Check
        # 2. Forward Flop Power 30 Check (read AFTER drive)
        # 3. Forward Flop Power 35 Check (read AFTER drive)
        # 4. Backward Flop Power 30 Check (read AFTER drive)
        # 5. Backward Flop Power 35 Check (read AFTER drive)

        mock_hw.read_imu_converted.side_effect = [
            reading_back_ok,      # Initial check
            reading_fwd_fail,     # Power 30 result
            reading_fwd_success,  # Power 35 result
            reading_bwd_fail,     # Power 30 result
            reading_bwd_success,  # Power 35 result
        ]

        wc.find_flop_thresholds()

        assert config_inst.control.kickup_power_forward == 35
        assert config_inst.control.kickup_power_backward == 35

def test_find_flop_thresholds_initial_pitch_warning(wc_fixture):
    wc, mock_hw, config_inst = wc_fixture

    with patch("builtins.input", return_value=""), \
         patch("time.sleep"), \
         patch("builtins.print") as mock_print, \
         patch.object(wc, 'drive_and_measure', return_value=[]):

        # Scenario: Robot is too upright at start of forward flop test
        reading_bad_start = MagicMock(pitch_angle=0.0) # > -10
        reading_success = MagicMock(pitch_angle=20.0) # Success immediately
        reading_bwd_success = MagicMock(pitch_angle=-20.0)

        mock_hw.read_imu_converted.side_effect = [
            reading_bad_start,
            reading_success,
            reading_bwd_success
        ]

        wc.find_flop_thresholds()

        # Check that warning was printed
        # "Warning: Pitch 0.0 is not Back enough."
        assert any("Warning: Pitch 0.0 is not Back enough" in str(c) for c in mock_print.call_args_list)
