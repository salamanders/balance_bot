from unittest.mock import MagicMock
import pytest
from balance_bot.discovery.steps import DiscoverBusesStep
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.discovery.step import StepStatus

def test_discover_buses_success():
    """Test that discovery step finds buses correctly."""
    step = DiscoverBusesStep()
    hw = MagicMock() # Not used by this step mostly, it scans via smbus
    config = HardwareConfig()
    state = LearningState()

    # We need to mock scan_i2c (imported in steps.py)
    # The step calls: scan_i2c("PiconZero...", check_fn)

    with pytest.MonkeyPatch.context() as m:
        # Mock scan_i2c to return specific bus IDs
        def mock_scan(name, check_fn):
            if "PiconZero" in name:
                return 1
            if "MPU6050" in name:
                return 1
            return None

        m.setattr("balance_bot.discovery.steps.scan_i2c", mock_scan)

        status, cfg_upd, state_upd = step.run(hw, config, state)

        assert status == StepStatus.SUCCESS
        assert cfg_upd['motor_i2c_bus'] == 1
        assert cfg_upd['imu_i2c_bus'] == 1
        assert state_upd['i2c_buses_verified'] is True

def test_discover_buses_failure():
    """Test that discovery step handles missing hardware."""
    step = DiscoverBusesStep()
    hw = MagicMock()
    config = HardwareConfig()
    state = LearningState()

    with pytest.MonkeyPatch.context() as m:
        m.setattr("balance_bot.discovery.steps.scan_i2c", lambda n, c: None)

        status, _, _ = step.run(hw, config, state)
        assert status == StepStatus.FATAL
