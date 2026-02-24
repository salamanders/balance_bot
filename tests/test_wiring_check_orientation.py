import sys
import pytest
import glm
from unittest.mock import MagicMock, patch

# Mock smbus2 before import
if 'smbus2' not in sys.modules:
    sys.modules['smbus2'] = MagicMock()

from balance_bot.wiring_check import WiringCheck
from balance_bot.enums import Axis
from balance_bot.configuration import HardwareConfig, LearningState

@pytest.fixture
def wc_fixture():
    with patch("balance_bot.utils.smbus"), \
         patch("balance_bot.wiring_check.RobotHardware") as MockHW, \
         patch("balance_bot.wiring_check.HardwareConfig") as MockHWConfig, \
         patch("balance_bot.wiring_check.LearningState") as MockLearningState:

        # Setup Configs with PRE-EXISTING axes (from Step 2 - Yank Test)
        hw_config = MagicMock(spec=HardwareConfig)
        hw_config.motor_l = 0
        hw_config.motor_r = 1

        # Assume Yank Test found:
        # Pitch Axis = X (Inverted=True -> Raw Rot +X = Neg Pitch Rate)
        # Forward Axis = Y (Inverted=True -> Raw Delta +Y = Neg Forward Accel)
        hw_config.gyro_pitch_axis = Axis.X
        hw_config.gyro_pitch_invert = True
        hw_config.accel_forward_axis = Axis.Y
        hw_config.accel_forward_invert = True

        # Vertical is NOT YET known (will be found here)
        hw_config.accel_vertical_axis = None

        def model_copy_side_effect(update=None, deep=False):
            new_config = MagicMock(spec=HardwareConfig)
            for k, v in hw_config.__dict__.items():
                setattr(new_config, k, v)
            if update:
                for k, v in update.items():
                    setattr(new_config, k, v)
            return new_config

        hw_config.model_copy.side_effect = model_copy_side_effect
        MockHWConfig.load.return_value = hw_config

        learning_state = MagicMock(spec=LearningState)
        learning_state.min_power_visible = 20
        MockLearningState.load.return_value = learning_state

        wc = WiringCheck()
        wc.hw = MockHW.return_value
        wc.hw_config = hw_config # Set instance config
        wc.learning_state = learning_state

        yield wc, hw_config, learning_state

def test_calibrate_static_orientation_labeler(wc_fixture):
    wc, hw_config, learning_state = wc_fixture

    # 1. Mock _toddler_flail_collection to return vectors that cluster
    # Vector P1: Resting "Back". Pitch should be Negative.
    # Vector P2: Resting "Front". Pitch should be Positive.

    # Physics:
    # Pitch Axis = X. Forward Axis = Y. Vertical Axis = Z (to be found).
    # Forward Invert = True. So +Y reads as NEGATIVE Forward Accel.
    # Vertical Invert = ? (Let's say Z is Gravity Down = -9.8. So Invert=False? Wait.
    # If Accel measures proper acceleration (1g Up), Gravity Down reads as +1g on Z axis.
    # If sensor Z points UP, it reads +1g. If sensor Z points DOWN, it reads -1g.
    # Let's assume sensor Z points UP. So Gravity reads +1g (9.8).

    # Robot "Back" Rest: Leaning backward.
    # Pitch Angle (nose up) is positive?
    # Usually "Forward" lean is Positive Pitch? No, standard aviation: Nose Up is Positive Pitch.
    # But for a balancing robot, "Forward" usually means falling forward.
    # User Requirement: "The vector resulting in a Positive pitch must be assigned to rest_angle_forward."
    # So Forward Lean = Positive Pitch.
    # Backward Lean = Negative Pitch.

    # So P1 (Back Rest) -> Negative Pitch.
    # P2 (Front Rest) -> Positive Pitch.

    # Construct Vectors:
    # X (Pitch Axis) is negligible for static gravity.
    # Y (Forward Axis). Z (Vertical Axis).

    # Forward Lean (Positive Pitch): Gravity has component on Forward Axis.
    # If robot tips forward (nose down?), usually Pitch is negative in aviation.
    # But here: "Positive pitch must be assigned to rest_angle_forward".
    # So Forward Lean -> Positive Pitch.
    # Gravity vector rotates relative to sensor.
    # If sensor X is Right, Y is Forward, Z is Up.
    # Tipping Forward (+Pitch): Z axis tilts forward, Y axis tilts down.
    # Gravity (Down) now has -Y component and +Z component (reduced).
    # Wait, if Y tilts down, Gravity (Vertical) projects onto Y as Positive?
    # Gravity Vector is [0, 0, -g] in World Frame.
    # Sensor Frame rotated by +Theta around X.
    # R_x(theta) = [[1, 0, 0], [0, c, -s], [0, s, c]]
    # g_sensor = R * g_world = [0, g*sin(theta), -g*cos(theta)] ?
    # Let's just create vectors that give desired results when passed to calculate_pitch.
    # calculate_pitch(accel_forward, accel_vertical) = atan2(fwd, vert).

    # We want P2 -> Positive Pitch.
    # So atan2(fwd, vert) > 0.
    # Means fwd > 0 (assuming vert > 0).

    # We want P1 -> Negative Pitch.
    # So atan2(fwd, vert) < 0.
    # Means fwd < 0.

    # Known Config:
    # Forward Axis = Y. Invert = True.
    # So Mapped Forward = -Raw_Y.

    # Vertical Axis = Z (Dominant in P1). Invert = ?
    # Let's say P1 Raw = [0, +1, +9.8].
    # Vertical is Z. +9.8 is dominant.
    # If gravity is +9.8, that means sensor is accelerating UP? No, sitting on table, normal force is UP.
    # Accelerometer reads +1g UP.
    # So Z axis points UP.
    # Vertical Invert logic in old code: `update_dict['accel_vertical_invert'] = vert_val < 0`
    # If vert_val is +9.8, invert is False. Mapped Vertical = +Raw_Z = +9.8.

    # Back Rest (P1): Raw Y = +2.0.
    # Mapped Forward = -2.0.
    # Mapped Vertical = +9.8.
    # Pitch = atan2(-2, 9.8) = Negative. Correct (Back Rest).

    # Front Rest (P2): Raw Y = -2.0.
    # Mapped Forward = -(-2.0) = +2.0.
    # Mapped Vertical = +9.8.
    # Pitch = atan2(2, 9.8) = Positive. Correct (Front Rest).

    # So we need vectors:
    p1 = glm.vec3(0.0, 2.0, 9.8) # Back
    p2 = glm.vec3(0.0, -2.0, 9.8) # Front

    # Mock toddler flail to return these
    with patch.object(wc, '_toddler_flail_collection', return_value=[p1, p1, p2, p2]):

        # Mock hw.get_mapped_value to behave correctly based on config
        # But wait, hw is a mock. We can't rely on it using the config logic unless we implement side_effect.
        # Or we can just mock the return values if we know what order they are called.
        # It's better to implement a simple side_effect for get_mapped_value to verify the logic.

        def get_mapped_value_side_effect(vec, axis_name):
            # axis_name e.g. "accel_forward"
            # Look up in mocked config
            if axis_name == "accel_forward":
                axis = hw_config.accel_forward_axis
                invert = hw_config.accel_forward_invert
            elif axis_name == "accel_vertical":
                # This might be called before or after config update?
                # The code updates config first, then calculates rest angles.
                # So we should use the updated values.
                # But hw_config mock object might not update in real-time unless we track it.
                # In the test fixture, model_copy returns a NEW mock, but we don't update `wc.hw_config` automatically?
                # The code does: self.hw_config = self.hw_config.model_copy...
                # So wc.hw_config will point to the NEW mock.
                # We need to make sure our side_effect looks at `wc.hw_config`.
                axis = wc.hw_config.accel_vertical_axis
                invert = wc.hw_config.accel_vertical_invert
            else:
                return 0.0

            val = getattr(vec, axis.value)
            if invert:
                val = -val
            return val

        wc.hw.get_mapped_value.side_effect = get_mapped_value_side_effect

        # Also need get_axis_value for some calls if legacy code uses it?
        # New code uses `get_mapped_value`? I need to verify what I write.
        # User prompt says: "Map p1 and p2 using self.hw.get_mapped_value"

        # Run
        wc.calibrate_static_orientation()

    # Assertions

    # 1. Vertical Axis Discovery (from P1=[0, 2, 9.8]) -> Z
    # Check that Z was set as Vertical
    # We need to find the config update.
    # Note: wc.hw_config has been replaced by the result of model_copy.
    final_config = wc.hw_config

    # In my fixture, model_copy returns a new mock.
    # I need to verify the attributes on that final mock.
    assert final_config.accel_vertical_axis == Axis.Z
    # Invert should be False (since 9.8 > 0)
    assert final_config.accel_vertical_invert is False

    # 2. Rest Angles
    # Back Angle (P1) -> Negative Pitch
    # Front Angle (P2) -> Positive Pitch
    # Check that they are assigned correctly
    assert learning_state.rest_angle_forward > 0
    assert learning_state.rest_angle_backward < 0

    assert learning_state.save.called
