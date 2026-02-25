import pytest
from unittest.mock import MagicMock, patch
import glm
from balance_bot.hardware.robot_hardware import RobotHardware, IMUReading
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.enums import Axis
from balance_bot.discovery.steps import DiscoverBusesStep, StepStatus

class TestFailLoud:

    @pytest.fixture
    def mock_hardware_deps(self):
        # We only patch MPU6050Adapter because it is defined at module level.
        # PiconZero is imported locally so we don't patch it here,
        # we rely on mocking initialize_drivers and injecting hw.pz directly.
        with patch('balance_bot.hardware.robot_hardware.MPU6050Adapter') as mock_imu:
            yield mock_imu

    def test_set_motors_raises_if_unmapped(self, mock_hardware_deps):
        """Test that set_motors raises RuntimeError if motor channels are None."""
        # Config with unmapped motors
        hw_config = HardwareConfig(motor_l=None, motor_r=None)
        state = LearningState()

        # We need to bypass __init__ driver init or mock it
        with patch.object(RobotHardware, 'initialize_drivers'):
            hw = RobotHardware(hw_config, state)
            # Manually inject mocks so set_motors doesn't fail on missing pz
            hw.pz = MagicMock()

            with pytest.raises(RuntimeError, match="CRITICAL: Attempted to actuate motors"):
                hw.set_motors(50, 50)

    def test_set_motors_raises_if_one_unmapped(self, mock_hardware_deps):
        """Test that set_motors raises RuntimeError if even one motor is None."""
        hw_config = HardwareConfig(motor_l=0, motor_r=None)
        state = LearningState()

        with patch.object(RobotHardware, 'initialize_drivers'):
            hw = RobotHardware(hw_config, state)
            hw.pz = MagicMock()

            with pytest.raises(RuntimeError, match="CRITICAL: Attempted to actuate motors"):
                hw.set_motors(50, 50)

    def test_set_motors_succeeds_if_mapped(self, mock_hardware_deps):
        """Test that set_motors works if mapped."""
        hw_config = HardwareConfig(motor_l=0, motor_r=1)
        state = LearningState()

        with patch.object(RobotHardware, 'initialize_drivers'):
            hw = RobotHardware(hw_config, state)
            hw.pz = MagicMock()

            # Should not raise
            hw.set_motors(50, 50)
            hw.pz.set_motors.assert_called()

    def test_read_imu_converted_raises_if_yaw_missing_in_adult_mode(self, mock_hardware_deps):
        """
        Test that read_imu_converted raises RuntimeError if in Adult Mode (Pitch/Vert/Fwd set)
        but Yaw is missing.
        """
        hw_config = HardwareConfig(
            accel_vertical_axis=Axis.Z,
            accel_forward_axis=Axis.Y,
            gyro_pitch_axis=Axis.X,
            gyro_yaw_axis=None  # Missing Yaw
        )
        state = LearningState()

        with patch.object(RobotHardware, 'initialize_drivers'):
            hw = RobotHardware(hw_config, state)
            hw.sensor = MagicMock()
            hw.sensor.get_accel_data.return_value = glm.vec3(0, 0, 9.8)
            hw.sensor.get_gyro_data.return_value = glm.vec3(0, 0, 0)

            # Should raise because it tries to read Yaw which is None
            with pytest.raises(RuntimeError, match="CRITICAL: Attempted to read unmapped axis"):
                hw.read_imu_converted()

    def test_read_imu_converted_toddler_mode_safe(self, mock_hardware_deps):
        """
        Test that read_imu_converted returns safe defaults in Toddler Mode
        (Critical axes missing).
        """
        hw_config = HardwareConfig(
            accel_vertical_axis=None, # Trigger Toddler Mode
            gyro_yaw_axis=None
        )
        state = LearningState()

        with patch.object(RobotHardware, 'initialize_drivers'):
            hw = RobotHardware(hw_config, state)
            hw.sensor = MagicMock()
            hw.sensor.get_accel_data.return_value = glm.vec3(0, 0, 9.8)
            hw.sensor.get_gyro_data.return_value = glm.vec3(0, 0, 0)

            reading = hw.read_imu_converted()
            assert reading.pitch_angle == 0.0
            assert reading.yaw_rate == 0.0

    def test_discovery_step_bootstrap(self):
        """Test that DiscoverBusesStep injects bootstrap configuration."""
        step = DiscoverBusesStep()
        hw = MagicMock()
        config = HardwareConfig()
        state = LearningState()

        # Mock scan_i2c to return buses
        with patch('balance_bot.discovery.steps.scan_i2c', side_effect=[1, 1]):
            status, config_updates, state_updates = step.run(hw, config, state)

            assert status == StepStatus.SUCCESS
            assert config_updates['motor_l'] == 0
            assert config_updates['motor_r'] == 1
