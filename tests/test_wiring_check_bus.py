import sys
import pytest
from unittest.mock import MagicMock, patch

# Mock smbus2
mock_smbus = MagicMock()
sys.modules["smbus2"] = mock_smbus

# Mock RobotHardware and Config to avoid file I/O and hardware init
sys.modules["balance_bot.hardware.robot_hardware"] = MagicMock()

from balance_bot.wiring_check import WiringCheck

@pytest.fixture
def wc():
    with patch("balance_bot.wiring_check.RobotConfig") as MockConfig:
        config_inst = MagicMock()
        config_inst.motor_i2c_bus = 99 # Default/Pre-existing
        config_inst.imu_i2c_bus = 88
        MockConfig.load.return_value = config_inst

        wc = WiringCheck()
        return wc

@patch("builtins.input", return_value="y")
def test_detect_i2c_buses_found(mock_input, wc):
    """Test successful detection on specific buses."""
    # Candidates are [1, 3, 0, 2]
    # Simulate: PiconZero (0x22) on Bus 1
    # Simulate: MPU6050 (0x68) on Bus 3

    def smbus_side_effect(bus_id):
        cm = MagicMock()
        bus = MagicMock()
        cm.__enter__.return_value = bus

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

        return cm

    mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_buses()

    assert wc.config.motor_i2c_bus == 1
    assert wc.config.imu_i2c_bus == 3

@patch("builtins.input", return_value="y")
def test_detect_i2c_buses_not_found(mock_input, wc):
    """Test when no bus has the device."""
    def smbus_side_effect(bus_id):
        cm = MagicMock()
        bus = MagicMock()
        cm.__enter__.return_value = bus
        bus.read_word_data.side_effect = OSError("Device not found")
        bus.read_byte_data.side_effect = OSError("Device not found")
        return cm

    mock_smbus.SMBus.side_effect = smbus_side_effect

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

        cm = MagicMock()
        bus = MagicMock()
        cm.__enter__.return_value = bus
        bus.read_word_data.side_effect = OSError("Device not found")
        bus.read_byte_data.side_effect = OSError("Device not found")
        return cm

    mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_buses()

    assert wc.config.motor_i2c_bus == 99
    assert wc.config.imu_i2c_bus == 88
