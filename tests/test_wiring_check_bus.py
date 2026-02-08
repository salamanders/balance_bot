import sys
import pytest
from unittest.mock import MagicMock, patch, call

# Mock smbus before import
sys.modules['smbus'] = MagicMock()

from balance_bot.wiring_check import WiringCheck

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.wiring_check.smbus") as mock_smbus_module, \
         patch("balance_bot.wiring_check.RobotHardware") as mock_rh_cls, \
         patch("balance_bot.wiring_check.RobotConfig") as MockConfig:

        config_inst = MagicMock()
        config_inst.motor_i2c_bus = 99
        config_inst.imu_i2c_bus = 88
        config_inst.min_power_visible = 60
        MockConfig.load.return_value = config_inst

        # Setup RobotHardware Mock Instance
        mock_hw = mock_rh_cls.return_value

        wc = WiringCheck()
        yield wc, mock_hw, mock_smbus_module

def test_detect_i2c_buses_found(wc_fixture):
    wc, mock_hw, mock_smbus = wc_fixture

    # Setup SMBus Mocks
    def smbus_side_effect(bus_id):
        bus = MagicMock()
        # PiconZero (0x22) on Bus 1
        # MPU6050 (0x68) on Bus 3

        def read_word_se(addr, reg):
            if bus_id == 1 and addr == 0x22: return 0
            raise OSError
        bus.read_word_data.side_effect = read_word_se

        def read_byte_se(addr, reg):
            if bus_id == 3 and addr == 0x68: return 0x68
            raise OSError
        bus.read_byte_data.side_effect = read_byte_se
        return bus

    mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_buses()

    assert wc.config.motor_i2c_bus == 1
    assert wc.config.imu_i2c_bus == 3

def test_setup_motors_ramping_success(wc_fixture):
    wc, mock_hw, _ = wc_fixture

    # Inputs:
    # 1. 'n' (20 power) -> Try next
    # 2. 'y' (40 power) -> Found
    # 3. 'a' (Left Fwd) -> Motor A is Left
    # 4. 'y' (Right Fwd) -> Motor B is Right Fwd
    inputs = ['n', 'y', 'a', 'y']

    with patch("builtins.input", side_effect=inputs):
        wc.setup_motors()

    assert wc.config.min_power_visible == 40

    # Verify Calls
    # 1. (20, 20)
    # 2. (40, 40)
    # 3. (40, 0)
    calls = mock_hw.set_motors.call_args_list
    assert call(20, 20) in calls
    assert call(40, 40) in calls
    assert call(40, 0) in calls # Check A with detected power

def test_setup_motors_ramping_failure(wc_fixture):
    wc, mock_hw, _ = wc_fixture
    # Fail 20, 40, 60, 80, 100, 120
    inputs = ['n'] * 6

    with patch("builtins.input", side_effect=inputs), \
         patch("sys.exit", side_effect=SystemExit) as mock_exit:

        with pytest.raises(SystemExit):
            wc.setup_motors()

        mock_exit.assert_called_with(1)
