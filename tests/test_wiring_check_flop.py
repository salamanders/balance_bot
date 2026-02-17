
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
         patch("balance_bot.wiring_check.RobotConfig") as MockConfig:

        config_inst = MagicMock()
        config_inst.motor_i2c_bus = 1
        config_inst.imu_i2c_bus = 1
        config_inst.min_power_visible = 20
        config_inst.control.kickup_power_forward = 0.0
        config_inst.control.kickup_power_backward = 0.0
        MockConfig.load.return_value = config_inst

        # Setup RobotHardware Mock Instance
        mock_hw = MagicMock()
        # Make config return this mock hw
        config_inst.to_hardware.return_value = mock_hw

        # Ensure read_imu_converted returns a dummy reading by default
        mock_hw.read_imu_converted.return_value = MagicMock(spec=IMUReading, pitch_angle=0.0, pitch_rate=0.0, roll_angle=0.0, roll_rate=0.0, yaw_angle=0.0, yaw_rate=0.0)

        # Ensure drive_and_measure returns a list of dummy readings (so loops run at least once logic-wise if they iterate)
        mock_hw.drive_and_measure.return_value = [mock_hw.read_imu_converted.return_value]

        wc = WiringCheck()
        # Inject HW directly as well, though init_hw will overwrite it via config.to_hardware
        wc.hw = mock_hw

        # Mock wait_for_stability
        wc.wait_for_stability = MagicMock()

        # Also mock _wait_for_start_condition
        wc._wait_for_start_condition = MagicMock()

        yield wc, mock_hw, config_inst

def test_find_flop_thresholds_success(wc_fixture):
    wc, mock_hw, config_inst = wc_fixture

    # We patch _attempt_dynamic_flop to simulate success/failure
    with patch("builtins.input", side_effect=Exception("Input called!")), \
         patch("time.sleep"), \
         patch.object(wc, '_attempt_dynamic_flop') as mock_attempt:

        mock_attempt.side_effect = [
            "FAIL_POWER", "SUCCESS",
            "FAIL_POWER", "SUCCESS"
        ]

        wc.find_flop_thresholds()

        assert config_inst.control.kickup_power_forward == 35
        assert config_inst.control.kickup_power_backward == 35

        calls = mock_attempt.call_args_list
        assert calls[0] == call("BACK", 30)
        assert calls[1] == call("BACK", 35)
        assert calls[2] == call("FRONT", 30)
        assert calls[3] == call("FRONT", 35)

def test_find_flop_thresholds_initial_pitch_warning(wc_fixture):
    pass

def test_attempt_dynamic_flop_physics(wc_fixture):
    """
    Verify that _attempt_dynamic_flop uses correct motor signs.
    """
    wc, mock_hw, config_inst = wc_fixture

    # _attempt_dynamic_flop uses self.hw.drive_and_measure.
    # We want to verify the calls to drive_and_measure.

    # We mock time only to avoid sleep delays, logic doesn't depend on time anymore for the loop
    # because drive_and_measure encapsulates the loop.
    with patch("time.sleep"):

        # Test BACK kick (Should use Negative Power for kick)
        mock_hw.drive_and_measure.reset_mock()
        wc._attempt_dynamic_flop("BACK", 50.0)

        calls = mock_hw.drive_and_measure.call_args_list
        assert len(calls) >= 2

        # Call 0: Setup (Roll Forward -> Positive)
        # drive_and_measure(left, right, duration, ...)
        setup_args = calls[0][0] # (left, right, duration)
        assert setup_args[0] > 0, "Back setup should be positive roll"
        assert setup_args[0] == setup_args[1] # Both motors same

        # Call 1: Kick (Kick Up -> Backward Wheels -> Negative)
        kick_args = calls[1][0]
        assert kick_args[0] < 0, "Back kick should be negative power"
        assert kick_args[0] == -50.0

    # Re-run for FRONT with fresh mocks
    with patch("time.sleep"):

        mock_hw.drive_and_measure.reset_mock()
        wc._attempt_dynamic_flop("FRONT", 50.0)

        calls = mock_hw.drive_and_measure.call_args_list
        assert len(calls) >= 2

        # Call 0: Setup (Roll Backward -> Negative)
        setup_args = calls[0][0]
        assert setup_args[0] < 0, "Front setup should be negative roll"

        # Call 1: Kick (Kick Up -> Forward Wheels -> Positive)
        kick_args = calls[1][0]
        assert kick_args[0] > 0, "Front kick should be positive power"
        assert kick_args[0] == 50.0
