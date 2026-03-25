from typing import Any
from unittest.mock import MagicMock

from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.discovery.step import StepStatus

from balance_bot.discovery.discover_buses import DiscoverBusesStep
from balance_bot.discovery.hardware_init import HardwareInitStep
from balance_bot.discovery.manual_lean_calibration import ManualLeanCalibrationStep
from balance_bot.discovery.broken_wire_check import BrokenWireCheckStep
from balance_bot.discovery.friction_threshold import FrictionThresholdStep
from balance_bot.discovery.derive_kinematics import DeriveKinematicsStep
from balance_bot.discovery.mechanical_backlash import MechanicalBacklashStep
from balance_bot.discovery.motor_trim import MotorTrimStep
from balance_bot.discovery.kickup_dynamics import KickupDynamicsStep

# Mocks and helper structures to simulate the steps

def assert_updates_valid(config_updates: dict, state_updates: dict) -> None:
    # Ensure keys match model schema
    hw_fields = set(HardwareConfig.model_fields.keys())
    ls_fields = set(LearningState.model_fields.keys())

    for key in config_updates:
        assert key in hw_fields, f"Config update key '{key}' not found in HardwareConfig fields!"

    for key in state_updates:
        assert key in ls_fields, f"State update key '{key}' not found in LearningState fields!"

    # Also verify that pydantic doesn't throw on update
    HardwareConfig.model_validate(HardwareConfig().model_dump() | config_updates)

    dummy_state = LearningState()
    for k, v in state_updates.items():
        setattr(dummy_state, k, v)


def test_discover_buses_step(monkeypatch: Any) -> None:
    monkeypatch.setattr("balance_bot.discovery.discover_buses.scan_i2c", lambda _name, _check: 1)

    step = DiscoverBusesStep()
    status, config_updates, state_updates = step.run(MagicMock(), HardwareConfig(), LearningState())
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)

def test_hardware_init_step() -> None:
    hw_mock = MagicMock()
    hw_mock.pz = MagicMock()
    hw_mock.sensor = MagicMock()

    step = HardwareInitStep()
    status, config_updates, state_updates = step.run(hw_mock, HardwareConfig(), LearningState())
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)

def test_manual_lean_calibration_step(monkeypatch: Any) -> None:
    def mock_analyze_dominance(_readings: Any, label: Any) -> tuple[str, float, bool]:
        # analyze_dominance returns (str, float, bool)
        if "Vertical" in label:
            return "y", 9.8, True
        else:
            return "z", 0.5, True

    monkeypatch.setattr("balance_bot.utils.analyze_dominance", mock_analyze_dominance)

    # We must patch the nested functions since they are defined inside run
    # For simplicity, we can just mock hw.sensor.read_imu() and time.sleep()
    # It might be easier to just mock the analysis of dominance and wait routines if they were not nested
    # Since they are nested and use sleep, let's mock the hw.sensor.read_imu to return stable values instantly

    hw_mock = MagicMock()
    hw_mock.watchdog = MagicMock()

    # We'll mock the whole time.sleep to avoid hanging
    monkeypatch.setattr("time.sleep", lambda _x: None)

    import glm

    accel_back = glm.vec3(0.0, 9.8, 0.5)
    gyro_back = glm.vec3(0.0, 0.0, 0.0)

    accel_flop = glm.vec3(0.0, -9.8, -0.5)
    gyro_flop = glm.vec3(0.0, 0.0, 0.0)

    reading_index = [0]
    def read_imu_raw_mock() -> tuple[glm.vec3, glm.vec3]:
        reading_index[0] += 1
        if reading_index[0] <= 10:
            return accel_back, gyro_back
        elif reading_index[0] <= 11:
            # First flop reading that triggers the wait
            return accel_flop, gyro_flop
        else:
            return accel_flop, gyro_flop

    hw_mock.read_imu_raw = read_imu_raw_mock

    step = ManualLeanCalibrationStep()
    status, config_updates, state_updates = step.run(hw_mock, HardwareConfig(), LearningState())

    # If the mock logic works, we check updates
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)

def test_broken_wire_check_step(monkeypatch: Any) -> None:
    import glm

    # Mock execute_maneuver to return samples that have max_mag > 10.0
    hw_mock = MagicMock()

    mock_sample = MagicMock()
    mock_sample.error_count = 0
    mock_sample.gyro_raw = glm.vec3(15.0, 0, 0)

    mock_res = MagicMock()
    mock_res.samples = [mock_sample]

    hw_mock.execute_maneuver.return_value = mock_res

    # Mock sleep to run fast
    monkeypatch.setattr("time.sleep", lambda _x: None)

    step = BrokenWireCheckStep()
    status, config_updates, state_updates = step.run(hw_mock, HardwareConfig(), LearningState())
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)

def test_friction_threshold_step(monkeypatch: Any) -> None:
    monkeypatch.setattr("balance_bot.discovery.friction_threshold.find_threshold", lambda *_args, **_kwargs: 25)

    step = FrictionThresholdStep()
    status, config_updates, state_updates = step.run(MagicMock(), HardwareConfig(), LearningState())
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)

def test_derive_kinematics_step(monkeypatch: Any) -> None:
    # Mocking DeriveKinematics requires overriding _pulse_and_measure and baseline
    import glm

    step = DeriveKinematicsStep()

    hw_mock = MagicMock()
    hw_mock.read_imu_raw.return_value = (glm.vec3(0, 0, 9.8), glm.vec3(0, 0, 0))

    def pulse_mock(_hw: Any, left_power: Any, _r: Any, _name: Any) -> tuple[glm.vec3, glm.vec3]:
        if left_power > 0:
            return glm.vec3(0, -10, 0), glm.vec3(-5, 0, 9.8) # L turns left, tilts back
        else:
            return glm.vec3(0, 10, 0), glm.vec3(5, 0, 9.8)  # R turns right, tilts back

    monkeypatch.setattr(step, "_pulse_and_measure", pulse_mock)
    monkeypatch.setattr("balance_bot.discovery.derive_kinematics.analyze_dominance", lambda vec, name: ("y", 1, 1) if name == "Pitch Axis" else ("x", 1, 1) if name == "Forward Axis" else ("z", 1, 1))

    status, config_updates, state_updates = step.run(hw_mock, HardwareConfig(), LearningState())
    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)

def test_mechanical_backlash_step(monkeypatch: Any) -> None:
    # Mock sleep to run fast
    monkeypatch.setattr("time.sleep", lambda _x: None)


    class TimeMock:
        def __init__(self) -> None:
            self.t = 0.0
        def __call__(self) -> float:
            self.t += 0.05
            return self.t

    monkeypatch.setattr("time.time", TimeMock())

    hw_mock = MagicMock()
    mock_reading = MagicMock()
    mock_reading.pitch_rate = 10.0 # Force move
    hw_mock.read_imu_converted.return_value = mock_reading

    step = MechanicalBacklashStep()
    status, config_updates, state_updates = step.run(hw_mock, HardwareConfig(), LearningState())

    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)

def test_motor_trim_step() -> None:
    hw_mock = MagicMock()
    mock_res = MagicMock()
    mock_res.samples = [MagicMock()]
    mock_res.avg_yaw_rate = 0.5 # Small enough to pass
    mock_res.abs_avg_yaw_rate = 0.5
    hw_mock.execute_maneuver.return_value = mock_res

    # Mock IMU reading for the while loop in _run_square_validation
    mock_reading = MagicMock()
    mock_reading.yaw_rate = 10000.0  # High yaw rate to make the turn loop finish instantly in tests
    hw_mock.read_imu_converted.return_value = mock_reading

    step = MotorTrimStep()
    status, config_updates, state_updates = step.run(hw_mock, HardwareConfig(), LearningState())

    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)

def test_kickup_dynamics_step(monkeypatch: Any) -> None:
    # Return 50 for fwd and 60 for bwd
    monkeypatch.setattr("balance_bot.discovery.kickup_dynamics.KickupDynamicsStep._run_kickup_test", lambda self, hw, state, sign: 50.0 if sign > 0 else 60.0)

    step = KickupDynamicsStep()
    status, config_updates, state_updates = step.run(MagicMock(), HardwareConfig(), LearningState())

    assert status == StepStatus.SUCCESS
    assert_updates_valid(config_updates, state_updates)
