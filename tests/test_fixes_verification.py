import pytest
from unittest.mock import MagicMock, patch, PropertyMock
import glm
from balance_bot.enums import BotState, Direction
from balance_bot.wiring_check import WiringCheck
from balance_bot.behavior.agent import Agent
from balance_bot.hardware.robot_hardware import IMUReading, MeasureResult, RobotHardware
from balance_bot.reflex.balance_core import BalanceCore

# --- Fixtures ---
@pytest.fixture
def mock_learning_state():
    state = MagicMock()
    state.min_power_visible = 20.0
    state.pid.target_angle = 0.0
    state.control.kickup_power_forward = 30.0
    state.control.kickup_power_backward = 30.0
    # Mock save to avoid disk writes
    state.save = MagicMock()
    return state

@pytest.fixture
def mock_hw_config():
    config = MagicMock()
    config.motor_l = 0
    config.motor_r = 1
    config.loop_time = 0.01
    return config

# --- Tests ---

def test_align_motors_phase_dag_violation_fix():
    """
    Verify align_motors_phase uses raw data and does not crash when axes are None.
    """
    wc = WiringCheck()
    wc.hw = MagicMock(spec=RobotHardware)
    wc.learning_state = MagicMock()
    wc.learning_state.min_power_visible = 20.0
    wc.hw_config = MagicMock()
    # Simulate uninitialized axes
    wc.hw_config.accel_forward_axis = None
    wc.hw_config.gyro_pitch_axis = None

    # Mock wait_for_stability
    wc.hw.wait_for_stability = MagicMock()

    # Mock read_imu_converted for baseline
    # Returns raw vectors (valid) but 0.0 angles
    baseline_reading = IMUReading(
        pitch_angle=0.0, pitch_rate=0.0, yaw_rate=0.0, roll_angle=0.0, roll_rate=0.0,
        accel_raw=glm.vec3(0, 0, 1), gyro_raw=glm.vec3(0, 0, 0)
    )
    wc.hw.read_imu_converted.return_value = baseline_reading

    # Mock verify_with_retries to capture and run the test/verify logic
    with patch('balance_bot.wiring_check.verify_with_retries') as mock_verify:
        wc.align_motors_phase()

        args, _ = mock_verify.call_args
        test_fn = args[1]
        verify_fn = args[2]

        # 1. Execute test function
        # This calls drive_and_measure
        wc.hw.drive_and_measure.return_value = MeasureResult(
            duration=0.5,
            samples=[
                IMUReading(0,0,0,0,0, glm.vec3(0,0.1,1), glm.vec3(0,0,0)),
                IMUReading(0,0,0,0,0, glm.vec3(0,0.5,0.8), glm.vec3(0,0,0)) # Moved (Accel Change)
            ]
        )

        test_data = test_fn(0)

        # 2. Execute verify function
        # Expect Success because accel changed (Translation) and gyro is low (No Spin)
        result = verify_fn(test_data)

        assert result is True, "Should detect translation via raw accel change"

        # Verify get_mapped_value was NOT called (checked via mock)
        # wc.hw.get_mapped_value is a MagicMock on the instance, if it exists.
        # But we mocked the whole class instance.
        # Actually, RobotHardware instance has get_mapped_value method.
        # We can check if it was called.
        wc.hw.get_mapped_value.assert_not_called()


def test_incremental_kickup_fatal_error(mock_learning_state, mock_hw_config):
    """
    Verify _incremental_kickup transitions to FATAL_ERROR at max power failure.
    """
    with patch('balance_bot.behavior.agent.HardwareConfig.load', return_value=mock_hw_config), \
         patch('balance_bot.behavior.agent.LearningState.load', return_value=mock_learning_state), \
         patch('balance_bot.behavior.agent.BalanceCore') as MockCoreClass:

        # Setup Core Mock
        mock_core = MockCoreClass.return_value
        mock_core.pitch = -50.0 # Start Back
        mock_core.hw = MagicMock()

        agent = Agent()
        # Ensure agent uses our mock state
        agent.learning_state = mock_learning_state

        # Mock _wait_for_settle and _sleep_with_update to speed up test
        agent._wait_for_settle = MagicMock()
        agent._sleep_with_update = MagicMock()

        # Run kickup with start power near max to fail quickly
        # Max power is 100. Start 95. Step 5.
        # Loop 1: 95. Fail. Power -> 100.
        # Loop 2: 100. Fail. Power -> 105. Loop End.
        # Should set FATAL_ERROR.

        # We need update() to return telemetry with high error
        mock_core.update.return_value = MagicMock(pitch_angle=-50.0, pitch_rate=0.0)

        result = agent._incremental_kickup(target_angle=0.0, start_power=95.0)

        assert result is False
        assert agent.state == BotState.FATAL_ERROR
        # Verify logging of max power failure (optional)


def test_fix_power_value(mock_learning_state, mock_hw_config):
    """
    Verify fix_power calculation uses min_power_visible + 15.
    """
    with patch('balance_bot.behavior.agent.HardwareConfig.load', return_value=mock_hw_config), \
         patch('balance_bot.behavior.agent.LearningState.load', return_value=mock_learning_state), \
         patch('balance_bot.behavior.agent.BalanceCore') as MockCoreClass:

        mock_core = MockCoreClass.return_value
        mock_core.hw = MagicMock()

        agent = Agent()
        agent.learning_state = mock_learning_state
        agent._wait_for_settle = MagicMock()
        agent._sleep_with_update = MagicMock()

        # Scenario: Start BACK (-50).
        # Inside loop, detect FRONT (+20) -> Wrong Position -> Fix.

        # We need core.pitch to change.
        # 1. Init: -50 (Determine direction = BACKWARD)
        # 2. Loop check: +20 (Wrong Position)
        # 3. Fix Logic: Check again +20 (Fail/Abort)

        # Using PropertyMock on the instance attribute
        type(mock_core).pitch = PropertyMock(side_effect=[-50.0, 20.0, 20.0, 20.0])

        result = agent._incremental_kickup(target_angle=0.0, start_power=30.0)

        # Expected Fix Power:
        # min_power (20) + 15 = 35.
        # kick_direction = BACKWARD (-1).
        # fix_power = 35 * -(-1) = 35 * 1 = 35.0.

        mock_core.hw.set_motors.assert_any_call(35.0, 35.0)

        # Ensure we returned False (Aborted due to reposition failure)
        assert result is False
