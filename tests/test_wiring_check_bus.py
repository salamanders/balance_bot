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
        config_inst.i2c_bus = 99 # Default/Pre-existing
        MockConfig.load.return_value = config_inst

        wc = WiringCheck()
        return wc

def test_detect_i2c_bus_found(wc):
    """Test successful detection on a specific bus."""
    # Candidates are [1, 2, 3, 0]
    # We want to simulate finding it on Bus 3.

    # We need to mock smbus2.SMBus(bus_id)
    # It returns a context manager.

    def smbus_side_effect(bus_id):
        cm = MagicMock()
        bus = MagicMock()
        cm.__enter__.return_value = bus

        if bus_id == 3:
            # Success
            bus.read_word_data.return_value = 0
        else:
            # Failure
            bus.read_word_data.side_effect = OSError("Device not found")

        return cm

    mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_bus()

    assert wc.config.i2c_bus == 3

def test_detect_i2c_bus_not_found(wc):
    """Test when no bus has the device."""
    # Always fail
    def smbus_side_effect(bus_id):
        cm = MagicMock()
        bus = MagicMock()
        cm.__enter__.return_value = bus
        bus.read_word_data.side_effect = OSError("Device not found")
        return cm

    mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_bus()

    # Should remain unchanged
    assert wc.config.i2c_bus == 99

def test_detect_i2c_bus_os_error_on_open(wc):
    """Test when opening the bus raises OSError (bus doesn't exist)."""

    def smbus_side_effect(bus_id):
        if bus_id == 1:
            raise OSError("Bus not found")

        # Others open but device not found
        cm = MagicMock()
        bus = MagicMock()
        cm.__enter__.return_value = bus
        bus.read_word_data.side_effect = OSError("Device not found")
        return cm

    mock_smbus.SMBus.side_effect = smbus_side_effect

    wc.detect_i2c_bus()

    assert wc.config.i2c_bus == 99
