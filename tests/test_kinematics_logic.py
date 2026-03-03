import sys
import os
import unittest
from unittest.mock import MagicMock
import glm

# Adjust path to import src
sys.path.insert(0, os.path.abspath("src"))

from balance_bot.discovery.steps import DeriveKinematicsStep
from balance_bot.discovery.step import StepStatus
from balance_bot.configuration import HardwareConfig, LearningState, ControlConfig
from balance_bot.hardware.robot_hardware import RobotHardware, IMUReading

class TestDeriveKinematics(unittest.TestCase):

    def test_kinematics_happy_path(self):
        """Test the DeriveKinematicsStep with a standard correct setup."""

        # --- Setup ---
        step = DeriveKinematicsStep()

        # Config & State
        config = HardwareConfig()
        state = LearningState()
        state.min_power_visible = 50.0 # Just a value
        state.control = ControlConfig()
        state.spatial_orientation_verified = False # Force run
        state.motor_direction_verified = False
        state.motor_phasing_verified = False
        state.motor_channels_verified = False

        # Hardware Mock
        hw = MagicMock(spec=RobotHardware)
        # Baseline Accel (Gravity +Z)
        hw.read_imu_raw.return_value = (glm.vec3(0, 0, 1.0), glm.vec3(0, 0, 0))

        # Drive and Measure Mock
        # We need to return specific responses for Left Pulse and Right Pulse.

        # Scenario: Correct Setup (Not Swapped)
        # Forward -> +Y (Body Frame)
        # Pitch Back -> +X (Body Frame) (Nose Up)
        # Left Pulse -> Right Turn -> -Z (Body Frame)
        # Right Pulse -> Left Turn -> +Z (Body Frame)

        # Accel Lag (Inertial Force):
        # Forward Accel -> Backward Lag -> -Y

        # Left Pulse:
        # Expected: Right Turn (-Z) + Pitch Back (+X) + Forward Lag (-Y)
        l_gyro = glm.vec3(20, 0, -50) # Pitch Back (+X), Right Turn (-Z)
        l_accel_delta = glm.vec3(0, -0.5, 0) # Backward Lag (-Y)

        # Right Pulse:
        # Expected: Left Turn (+Z) + Pitch Back (+X) + Forward Lag (-Y)
        r_gyro = glm.vec3(20, 0, 50) # Pitch Back (+X), Left Turn (+Z)
        r_accel_delta = glm.vec3(0, -0.5, 0) # Backward Lag (-Y)

        def make_result(gyro, accel_delta):
            res = MagicMock()
            s = MagicMock(spec=IMUReading)
            s.gyro_raw = gyro
            s.accel_raw = accel_delta + glm.vec3(0, 0, 1.0) # Add baseline
            res.samples = [s] * 10
            return res

        hw.drive_and_measure.side_effect = [
            make_result(l_gyro, l_accel_delta),
            make_result(r_gyro, r_accel_delta)
        ]

        # --- Run ---
        status, config_updates, state_updates = step.run(hw, config, state)

        # --- Assert ---
        self.assertEqual(status, StepStatus.SUCCESS)

        # Verify Axes
        # Pitch Axis: Dominant of Sum Gyro (40, 0, 0) -> X
        self.assertEqual(config_updates['gyro_pitch_axis'].value, 'x')
        # Pitch Invert: +PWM -> Pitch Back (+X). Current code checks if pitch_val > 0.
        self.assertTrue(config_updates['gyro_pitch_invert'])

        # Forward Axis: Dominant of Sum Accel (0, -1.0, 0) -> Y
        self.assertEqual(config_updates['accel_forward_axis'].value, 'y')
        # Forward Invert: +PWM -> Lag (-Y). `accel_forward_invert = fwd_val > 0`.
        # -1.0 > 0 is False. So False.
        self.assertFalse(config_updates['accel_forward_invert'])

        # Vertical Axis: Baseline (0, 0, 1.0) -> Z
        self.assertEqual(config_updates['accel_vertical_axis'].value, 'z')
        # Vertical Invert: `vert_val < 0`. 1.0 < 0 is False. So False.
        self.assertFalse(config_updates['accel_vertical_invert'])

        # Yaw Axis: Z
        self.assertEqual(config_updates['gyro_yaw_axis'].value, 'z')

        # Motor Swap Check:
        # Result: swap_motors = False.
        self.assertNotIn('motor_l', config_updates)

if __name__ == "__main__":
    unittest.main()
