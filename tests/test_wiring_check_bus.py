import sys
import pytest
from unittest.mock import MagicMock, patch, call

# Mock smbus2 before import if missing
try:
    import smbus2 as smbus
except ImportError:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.wiring_check import WiringCheck

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.wiring_check.smbus") as mock_smbus_module, \
         patch("balance_bot.wiring_check.RobotHardware") as mock_rh_cls, \
         patch("balance_bot.wiring_check.RobotConfig") as MockConfig:

        config_inst = MagicMock()
        config_inst.motor_i2c_bus = None # Start unknown
        config_inst.imu_i2c_bus = None
        config_inst.min_power_visible = 0
        MockConfig.load.return_value = config_inst

        # Setup RobotHardware Mock Instance
        mock_hw = mock_rh_cls.return_value

        # Mock read_imu_raw default
        mock_hw.read_imu_raw.return_value = ({'x':0, 'y':0, 'z':0}, {'x':0, 'y':0, 'z':0})

        # Mock read_imu_converted default
        zero_reading = MagicMock()
        zero_reading.pitch_rate = 0.0
        zero_reading.yaw_rate = 0.0
        zero_reading.roll_rate = 0.0
        mock_hw.read_imu_converted.return_value = zero_reading

        wc = WiringCheck()
        # Inject mock hw directly because init_hw might fail without buses
        wc.hw = mock_hw

        yield wc, mock_hw, mock_smbus_module

def test_discover_buses_found(wc_fixture):
    wc, mock_hw, mock_smbus = wc_fixture

    # Setup SMBus Mocks
    # Bus 1 has PiconZero (0x22)
    # Bus 3 has MPU6050 (0x68)

    # We need separate Mock objects for each bus call
    bus1 = MagicMock()
    bus3 = MagicMock()
    bus0 = MagicMock()
    bus2 = MagicMock()

    # Configure Bus 1: Has 0x22 (read_byte_data succeeds), No 0x68 (read_byte_data fails)
    # Note: side_effect must return value or raise Exception
    def bus1_read(addr, reg):
        if addr == 0x22: return 0
        raise OSError("Not Found")
    bus1.read_byte_data.side_effect = bus1_read

    # Configure Bus 3: Has 0x68 (read_byte_data succeeds for 0x75), No 0x22
    def bus3_read(addr, reg):
        if addr == 0x68 and reg == 0x75: return 0x68
        raise OSError("Not Found")
    bus3.read_byte_data.side_effect = bus3_read

    # Bus 0, 2: Nothing
    bus0.read_byte_data.side_effect = OSError("Not Found")
    bus2.read_byte_data.side_effect = OSError("Not Found")

    def smbus_side_effect(bus_id):
        if bus_id == 1: return bus1
        if bus_id == 3: return bus3
        if bus_id == 0: return bus0
        if bus_id == 2: return bus2
        raise OSError("Bus Error")

    # smbus2.SMBus is the constructor
    mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.discover_buses()

    assert wc.config.motor_i2c_bus == 1
    assert wc.config.imu_i2c_bus == 3

def test_find_min_power_success(wc_fixture):
    wc, mock_hw, _ = wc_fixture

    # We need drive_and_measure to return a MeasureResult mock
    # The loop starts at pwm=10, step=5.
    # 10: Fail
    # 15: Fail
    # 20: Success

    def drive_side_effect(l, r, duration):
        res = MagicMock()
        if l >= 20:
             res.max_rate = 20.0 # Success
        else:
             res.max_rate = 0.0 # Failure
        return res

    mock_hw.drive_and_measure.side_effect = drive_side_effect

    wc.find_min_power()

    assert wc.config.min_power_visible == 20

    # Verify sequence
    # Should have called 10, 15, 20
    calls = [c[0][0] for c in mock_hw.drive_and_measure.call_args_list]
    assert 10 in calls
    assert 15 in calls
    assert 20 in calls
    assert 25 not in calls

def test_find_min_power_failure(wc_fixture):
    wc, mock_hw, _ = wc_fixture

    # Always return static
    mock_res = MagicMock()
    mock_res.max_rate = 0.0
    mock_hw.drive_and_measure.return_value = mock_res

    with patch("sys.exit", side_effect=SystemExit) as mock_exit:
        with pytest.raises(SystemExit):
            wc.find_min_power()

        mock_exit.assert_called_with(1)
        # Should have tried up to 100
        calls = [c[0][0] for c in mock_hw.drive_and_measure.call_args_list]
        assert 100 in calls
