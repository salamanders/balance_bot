import sys
import pytest
from unittest.mock import MagicMock, patch

from balance_bot.wiring_check import WiringCheck

@pytest.fixture
def wc():
    # Patch smbus specifically for WiringCheck module
    with patch("balance_bot.wiring_check.smbus") as mock_smbus_module, \
         patch("balance_bot.wiring_check.RobotHardware") as mock_rh, \
         patch("balance_bot.wiring_check.RobotConfig") as MockConfig:

        config_inst = MagicMock()
        config_inst.motor_i2c_bus = 99 # Default/Pre-existing
        config_inst.imu_i2c_bus = 88
        config_inst.min_power_visible = 60
        MockConfig.load.return_value = config_inst

        wc = WiringCheck()
        wc.mock_smbus = mock_smbus_module
        yield wc

@patch("builtins.input", return_value="y")
def test_detect_i2c_buses_found(mock_input, wc):
    """Test successful detection on specific buses."""
    # Candidates are [1, 3, 0, 2]
    # Simulate: PiconZero (0x22) on Bus 1
    # Simulate: MPU6050 (0x68) on Bus 3

    def smbus_side_effect(bus_id):
        # Return a bus mock directly
        bus = MagicMock()
        bus.close = MagicMock()

        # PiconZero Check (read_word_data 0x22)
        def read_word_side_effect(addr, reg):
            if bus_id == 1 and addr == 0x22:
                return 0 # Success
            raise OSError("Not found")

        bus.read_word_data.side_effect = read_word_side_effect

        # MPU6050 Check (read_byte_data 0x68)
        def read_byte_side_effect(addr, reg):
            if bus_id == 3 and addr == 0x68:
                return 0x68 # Success
            raise OSError("Not found")

        bus.read_byte_data.side_effect = read_byte_side_effect

        # MPU6050 Live Data Check (read_i2c_block_data)
        def read_block_side_effect(addr, reg, length):
            if bus_id == 3 and addr == 0x68:
                # Return valid 1g Z-axis: [0,0, 0,0, 0x40,0x00] -> Z=16384 (Mag ~16384 > 500)
                return [0, 0, 0, 0, 0x40, 0x00]
            raise OSError("Not found")
        bus.read_i2c_block_data.side_effect = read_block_side_effect

        return bus

    wc.mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_buses()

    assert wc.config.motor_i2c_bus == 1
    assert wc.config.imu_i2c_bus == 3

@patch("builtins.input", return_value="y")
def test_detect_i2c_buses_not_found(mock_input, wc):
    """Test when no bus has the device."""
    def smbus_side_effect(bus_id):
        bus = MagicMock()
        bus.close = MagicMock()
        bus.read_word_data.side_effect = OSError("Device not found")
        bus.read_byte_data.side_effect = OSError("Device not found")
        return bus

    wc.mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_buses()

    # Should remain unchanged
    assert wc.config.motor_i2c_bus == 99
    assert wc.config.imu_i2c_bus == 88

@patch("builtins.input", return_value="y")
def test_detect_i2c_bus_os_error_on_open(mock_input, wc):
    """Test when opening the bus raises OSError (bus doesn't exist)."""
    # Simulate Bus 1 failing to open completely

    def smbus_side_effect(bus_id):
        if bus_id == 1:
            raise OSError("Bus not found")

        bus = MagicMock()
        bus.close = MagicMock()
        bus.read_word_data.side_effect = OSError("Device not found")
        bus.read_byte_data.side_effect = OSError("Device not found")
        return bus

    wc.mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_buses()

    assert wc.config.motor_i2c_bus == 99
    assert wc.config.imu_i2c_bus == 88

def test_ramping_success_first_try(wc):
    """Test user says 'y' immediately at power 20."""
    with patch("builtins.input", side_effect=['y']):
        def smbus_side_effect(bus_id):
            bus = MagicMock()
            bus.read_word_data.return_value = 0 # Found 0x22
            bus.read_byte_data.return_value = 0x68 # Found 0x68
            bus.read_i2c_block_data.return_value = [0]*6 # accel
            return bus

        wc.mock_smbus.SMBus.side_effect = smbus_side_effect

        wc.detect_i2c_buses()

        assert wc.config.min_power_visible == 20
        assert wc.config.motor_i2c_bus == 1

def test_ramping_success_after_increase(wc):
    """Test user says 'm' (more power) then 'y'."""
    with patch("builtins.input", side_effect=['m', 'y']):
        def smbus_side_effect(bus_id):
            bus = MagicMock()
            bus.read_word_data.return_value = 0
            bus.read_byte_data.return_value = 0x68
            bus.read_i2c_block_data.return_value = [0]*6
            return bus

        wc.mock_smbus.SMBus.side_effect = smbus_side_effect

        wc.detect_i2c_buses()

        assert wc.config.min_power_visible == 40
        assert wc.config.motor_i2c_bus == 1

def test_ramping_failure_max_power(wc):
    """Test user says 'm' until 120, then 'n' (fail). Should exit."""
    inputs = ['m', 'm', 'm', 'm', 'm', 'n']

    with patch("builtins.input", side_effect=inputs),          patch("sys.exit", side_effect=SystemExit) as mock_exit:

        def smbus_side_effect(bus_id):
            bus = MagicMock()
            bus.read_word_data.return_value = 0
            return bus

        wc.mock_smbus.SMBus.side_effect = smbus_side_effect

        with pytest.raises(SystemExit):
            wc.detect_i2c_buses()

        mock_exit.assert_called_with(1)

def test_ramping_skip_bus(wc):
    """Test user says 'n' (wrong bus) at 20. Should try next bus."""
    with patch("builtins.input", side_effect=['n', 'y']):

        def smbus_side_effect(bus_id):
            bus = MagicMock()
            bus.read_word_data.return_value = 0
            bus.read_byte_data.return_value = 0x68
            bus.read_i2c_block_data.return_value = [0]*6
            return bus

        wc.mock_smbus.SMBus.side_effect = smbus_side_effect

        wc.detect_i2c_buses()

        assert wc.config.motor_i2c_bus == 3 # Detected on Bus 3
        assert wc.config.min_power_visible == 20

def test_detect_i2c_buses_uses_write_byte_data(wc):
    """
    Verify that detect_i2c_buses uses write_byte_data for motor control
    and NOT write_i2c_block_data for the motor address (0x22).
    """
    with patch("builtins.input", return_value='y'):
        # Setup mock bus
        mock_bus = MagicMock()
        mock_bus.read_word_data.return_value = 0 # Found 0x22
        mock_bus.read_byte_data.return_value = 0x68 # Found 0x68
        mock_bus.read_i2c_block_data.return_value = [0]*6 # accel

        wc.mock_smbus.SMBus.return_value = mock_bus

        wc.detect_i2c_buses()

        # Verify calls to write_byte_data for PiconZero (0x22)
        # We expect calls to set motors 0 and 1 to power (20) and then stop (0)
        # Sequence:
        # 1. Pulse: Motor 0 -> 20, Motor 1 -> 20
        # 2. Stop:  Motor 0 -> 0,  Motor 1 -> 0

        # Filter calls for 0x22
        write_byte_calls = [
            c for c in mock_bus.write_byte_data.call_args_list
            if c.args[0] == 0x22
        ]

        # We expect at least 4 calls (2 start, 2 stop)
        assert len(write_byte_calls) >= 4

        # Check specific calls
        # args: (addr, reg, value)
        has_motor0_start = any(c.args[1] == 0 and c.args[2] == 20 for c in write_byte_calls)
        has_motor1_start = any(c.args[1] == 1 and c.args[2] == 20 for c in write_byte_calls)
        has_motor0_stop = any(c.args[1] == 0 and c.args[2] == 0 for c in write_byte_calls)
        has_motor1_stop = any(c.args[1] == 1 and c.args[2] == 0 for c in write_byte_calls)

        assert has_motor0_start, "Motor 0 start command not found"
        assert has_motor1_start, "Motor 1 start command not found"
        assert has_motor0_stop, "Motor 0 stop command not found"
        assert has_motor1_stop, "Motor 1 stop command not found"

        # Verify NO calls to write_i2c_block_data for 0x22
        block_calls = [
            c for c in mock_bus.write_i2c_block_data.call_args_list
            if c.args[0] == 0x22
        ]
        assert len(block_calls) == 0, f"Found block write calls for PiconZero: {block_calls}"
