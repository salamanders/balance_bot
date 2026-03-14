import time
import logging
from typing import Tuple, Dict, Any, List, Optional
import glm

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..enums import Axis
from ..utils import analyze_dominance

logger = logging.getLogger(__name__)

# --- Step 4: Derive Kinematics (The Single Wiggle) ---
class DeriveKinematicsStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Derive Kinematics (Single Wiggle)"

    def is_verified(self, state: LearningState) -> bool:
        # Check all related flags. If any missing, run.
        return (state.spatial_orientation_verified and
                state.motor_direction_verified and
                state.motor_phasing_verified and
                state.motor_channels_verified)

    def _pulse_and_measure(self, hw: RobotHardware, l_p: float, r_p: float, name: str) -> Tuple[glm.vec3, glm.vec3]:
        """Pulse motors and return average gyro and accel vectors."""
        logger.info(f"  Pulsing {name}...")

        # Ramp up power
        steps = []
        ramp_duration = 0.1
        ramp_steps = 5
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps.append((l_p * factor, r_p * factor, ramp_duration / ramp_steps))

        # Hold
        steps.append((l_p, r_p, 0.4 - ramp_duration))

        # Ramp down
        for i in range(ramp_steps - 1, -1, -1):
            factor = i / ramp_steps
            steps.append((l_p * factor, r_p * factor, ramp_duration / ramp_steps))

        res = hw.execute_maneuver(steps)
        time.sleep(1.0) # Settle

        if not res.samples:
             logger.error(f"  [FAILURE] No samples collected for {name} Pulse. Ignored command / System Glitch.")
             return None, None

        error_count = sum(s.error_count > 0 for s in res.samples)
        if error_count > 0:
            logger.warning(f"  [GLITCH] Encountered {error_count} IMU errors during {name} pulse.")

        gyro = self._avg_vec([s.gyro_raw for s in res.samples if s.gyro_raw])
        accel = self._avg_vec([s.accel_raw for s in res.samples if s.accel_raw])

        # Verify if the motor actually moved the robot
        if glm.length(gyro) < 5.0:
            logger.warning(f"  [IGNORED] Command to {name} resulted in <5.0 deg/s rotation. Not enough power or glitch.")

        return gyro, accel

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info("\n>>> Deriving Kinematics (Single Wiggle) <<<")
        hw.wait_for_stability()

        # 0. Baseline (Resting)
        baseline_accel, _ = hw.read_imu_raw()

        test_power = state.min_power_visible + 30.0

        # 1. Pulse Left Only (+PWM)
        l_gyro, l_accel = self._pulse_and_measure(hw, test_power, 0, "LEFT Motor")
        if l_gyro is None:
            return StepStatus.NEEDS_RETRY, {}, {}
        l_accel_delta = l_accel - baseline_accel

        # Wait for complete physical stability before Right motor test
        logger.info("  Waiting for stability before pulsing RIGHT Motor...")
        hw.wait_for_stability()

        # 2. Pulse Right Only (+PWM)
        r_gyro, r_accel = self._pulse_and_measure(hw, 0, test_power, "RIGHT Motor")
        if r_gyro is None:
            return StepStatus.NEEDS_RETRY, {}, {}
        r_accel_delta = r_accel - baseline_accel

        # --- Phase Alignment ---
        logger.info("  [Analysis] Checking Motor Phase...")
        up_dir = glm.normalize(baseline_accel)
        yaw_mag_sum = abs(glm.dot(l_gyro + r_gyro, up_dir))
        yaw_mag_diff = abs(glm.dot(l_gyro - r_gyro, up_dir))

        motor_r_invert = False

        if yaw_mag_sum > yaw_mag_diff:
            logger.info("  -> Phase Mismatch Detected. Inverting Right Motor logic.")
            motor_r_invert = True
            # Flip R vectors to match L for math
            r_gyro = -r_gyro
            r_accel_delta = -r_accel_delta
        else:
            logger.info("  -> Phase Matched.")

        # --- Pitch & Forward Axis ---
        sum_gyro = l_gyro + r_gyro
        sum_accel = l_accel_delta + r_accel_delta

        # Pitch: Dominant axis of Sum Gyro
        pitch_axis_name, _, _ = analyze_dominance(sum_gyro, "Pitch Axis")
        pitch_val = getattr(sum_gyro, pitch_axis_name)
        # We expect +PWM -> Negative Pitch Rate. If Positive, Invert.
        gyro_pitch_invert = pitch_val > 0

        # Forward: Dominant axis of Sum Accel
        # Exclude Pitch Axis?
        # Ideally Forward is distinct from Pitch axis (which is Gyro).
        # Accel Forward is usually distinct.
        fwd_axis_name, _, _ = analyze_dominance(sum_accel, "Forward Axis")
        fwd_val = getattr(sum_accel, fwd_axis_name)
        # We expect +PWM -> Negative Accel Delta (Lag). If Positive, Invert.
        accel_forward_invert = fwd_val > 0

        # --- Vertical Axis ---
        # The remaining axis.
        axes = {'x', 'y', 'z'}
        # Note: Pitch Axis is Gyro axis (e.g. 'y'). Forward is Accel axis (e.g. 'x').
        # Vertical should be 'z'.
        # We need to pick the one that is NOT Forward. (Pitch is rotation, doesn't consume a translation axis, but usually aligns with Y).
        # Let's verify dominance on Baseline Accel excluding Forward Axis.
        baseline_candidates = {k: abs(getattr(baseline_accel, k)) for k in axes if k != fwd_axis_name}
        vert_axis_name, _, _ = analyze_dominance(baseline_candidates, "Vertical Axis")

        vert_val = getattr(baseline_accel, vert_axis_name)
        # We want Baseline to be POSITIVE (1g). If Negative, Invert.
        accel_vertical_invert = vert_val < 0

        # --- Yaw & L/R Identity ---
        logger.info("  [Analysis] Checking L/R Identity...")
        # Yaw Axis: Dominant axis of (L_gyro_raw - R_gyro_raw) ?
        # Or just use Vertical Axis (Z)? Usually Yaw is around Vertical.
        # Let's use Vertical Axis as Yaw Axis.
        gyro_yaw_axis = vert_axis_name

        # Physical Up Vector = Lag x Pitch Backward
        raw_up = glm.cross(sum_accel, sum_gyro)

        # Yaw Diff
        yaw_diff = l_gyro - r_gyro

        check_val = glm.dot(yaw_diff, raw_up)
        logger.info(f"    Yaw Check Dot Product: {check_val:.2f}")

        swap_motors = False
        if check_val > 0:
            logger.info("  -> Yaw Direction Opposite. Motors are physically swapped.")
            swap_motors = True
        else:
            logger.info("  -> Yaw Direction Correct.")

        # --- Calculate Yaw Invert ---
        raw_yaw_val = getattr(l_gyro, gyro_yaw_axis)
        if swap_motors:
            # l_gyro was physically a LEFT turn. We expect the mapped Yaw to be Positive.
            gyro_yaw_invert = raw_yaw_val < 0
        else:
            # l_gyro was physically a RIGHT turn. We expect the mapped Yaw to be Negative.
            gyro_yaw_invert = raw_yaw_val > 0

        logger.info("\n--- Discovery Confidence Summary ---")
        logger.info("Motor Phase Match Confidence: High (Based on Sum > Diff)")
        logger.info(f"Pitch Axis Guess: {pitch_axis_name.upper()} Confidence: High (Based on Gyro Dominance)")
        logger.info(f"Forward Axis Guess: {fwd_axis_name.upper()} Confidence: Medium (Based on Accel Change)")
        logger.info("L/R Identity Match Confidence: High (Yaw cross product)")

        # --- Updates ---
        config_updates = {
            'gyro_pitch_axis': Axis(pitch_axis_name),
            'gyro_pitch_invert': gyro_pitch_invert,
            'accel_forward_axis': Axis(fwd_axis_name),
            'accel_forward_invert': accel_forward_invert,
            'accel_vertical_axis': Axis(vert_axis_name),
            'accel_vertical_invert': accel_vertical_invert,
            'gyro_yaw_axis': Axis(gyro_yaw_axis),
            'gyro_yaw_invert': gyro_yaw_invert,
            'motor_r_invert': motor_r_invert,
        }

        if swap_motors:
            # Swap channels
            # Also swap Inverts?
            # If we swap channels, the physical wiring is swapped.
            # If L was + and R was - (inverted), and we swap cables,
            # Now "Logical L" goes to "Physical R". "Logical R" goes to "Physical L".
            # We need to check if the *polarity* follows the motor or the port.
            # Usually polarity is a property of the Motor+Wire+Gearbox.
            # So if we swap ports, the polarity *of the motor* stays with the motor.
            # So we should swap the invert flags too.
            config_updates['motor_l'] = config.motor_r
            config_updates['motor_r'] = config.motor_l
            config_updates['motor_l_invert'] = config.motor_r_invert
            config_updates['motor_r_invert'] = config.motor_l_invert

            # But wait! We just calculated `motor_r_invert` (local var) based on the *current* configuration (before swap).
            # The `motor_r_invert` we found applies to the motor connected to the *current* Right Channel.
            # If we swap channels, that motor becomes the Left Motor.
            # So `config.motor_l_invert` (new) should become `motor_r_invert` (calculated)?
            # And `config.motor_r_invert` (new) should become `config.motor_l_invert` (old)?
            # Let's trace carefully.
            # Current Config: L=0, R=1.
            # We detected R (Channel 1) needs Invert. `motor_r_invert = True`.
            # We detected Swap Needed. (Channel 0 is actually Right, Channel 1 is actually Left).
            # So we want Logical Left -> Channel 1. Logical Right -> Channel 0.
            # Channel 1 (now Left) needed Invert. So `motor_l_invert` = True.
            # Channel 0 (now Right) uses whatever L used? (We assumed L was correct phase because we didn't check L phase, we aligned R to L).
            # This implies `motor_l_invert` (original) was correct for Channel 0.
            # So:
            # New L Channel = Old R Channel. New L Invert = Old R Invert (Calculated).
            # New R Channel = Old L Channel. New R Invert = Old L Invert (Original).

            config_updates['motor_l'] = config.motor_r
            config_updates['motor_r'] = config.motor_l
            config_updates['motor_l_invert'] = motor_r_invert # The one we just calculated for the old R channel
            config_updates['motor_r_invert'] = config.motor_l_invert # The existing L invert

        state_updates = {
            'spatial_orientation_verified': True,
            'motor_direction_verified': True,
            'motor_phasing_verified': True,
            'motor_channels_verified': True
        }

        return StepStatus.SUCCESS, config_updates, state_updates

    def _avg_vec(self, vecs: List[glm.vec3]) -> glm.vec3:
        if not vecs:
            return glm.vec3(0)
        s = glm.vec3(0)
        for v in vecs:
            s += v
        return s / len(vecs)
