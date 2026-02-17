import sys
import pytest
from unittest.mock import MagicMock, patch, call
import time

# Mock smbus before import if missing
if 'smbus' not in sys.modules:
    sys.modules['smbus'] = MagicMock()

from balance_bot.wiring_check import WiringCheck

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.wiring_check.smbus"), \
         patch("balance_bot.wiring_check.RobotConfig") as MockConfig:

        config_inst = MagicMock()
        MockConfig.load.return_value = config_inst

        wc = WiringCheck()
        wc.hw = MagicMock()
        config_inst.to_hardware.return_value = wc.hw

        yield wc

def test_wait_for_stability_delegation(wc_fixture):
    """
    Verify WiringCheck.wait_for_stability delegates to RobotHardware.
    """
    wc = wc_fixture

    wc.wait_for_stability(duration=0.5, threshold=3.0)

    wc.hw.wait_for_stability.assert_called_once_with(0.5, 3.0)

def test_wait_for_stability_inits_hw(wc_fixture):
    """
    Verify WiringCheck.wait_for_stability initializes HW if missing.
    """
    wc = wc_fixture
    wc.hw = None # Simulate missing HW

    # We need to mock to_hardware to return a mock we can check
    mock_hw = MagicMock()
    wc.config.to_hardware.return_value = mock_hw

    # Run
    wc.wait_for_stability(duration=1.0, threshold=2.0)

    # Verify Init
    wc.config.to_hardware.assert_called()
    mock_hw.init.assert_called()

    # Verify Delegation
    mock_hw.wait_for_stability.assert_called_once_with(1.0, 2.0)
