import time
import logging
import glm
from typing import Tuple, Dict, Any, List, Optional

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..enums import Axis
from ..utils import (
    analyze_dominance,
    scan_i2c,
    make_i2c_check_fn,
    find_threshold
)

logger = logging.getLogger(__name__)

# --- Step 1: Discover Buses ---
class DiscoverBusesStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Discover I2C Buses"

    def is_verified(self, state: LearningState) -> bool:
        return state.i2c_buses_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        print("Scanning I2C Buses...")

        # 1. Find Motors (0x22)
        check_motor = make_i2c_check_fn(0x22, register=0)
        found_motor_bus = scan_i2c("PiconZero (Motors)", check_motor)
        if found_motor_bus is None:
            return StepStatus.FATAL, {}, {}

        # 2. Find IMU (0x68)
        check_imu = make_i2c_check_fn(0x68, register=0x75, expected_value=0x68)
        found_imu_bus = scan_i2c("MPU6050 (IMU)", check_imu)
        if found_imu_bus is None:
            return StepStatus.FATAL, {}, {}

        return StepStatus.SUCCESS, {
            'motor_i2c_bus': found_motor_bus,
            'imu_i2c_bus': found_imu_bus,
            'motor_l': 0,  # Bootstrap guess
            'motor_r': 1   # Bootstrap guess
        }, {'i2c_buses_verified': True}

# --- Step 2: Hardware Init ---
class HardwareInitStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Initialize Hardware Drivers"

    def is_verified(self, state: LearningState) -> bool:
        return state.hardware_init_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        print("Initializing Hardware Drivers...")
        # Force driver initialization now that buses are known
        try:
            hw.initialize_drivers()
        except Exception as e:
            print(f"  [FAILURE] Driver init raised exception: {e}")
            return StepStatus.FATAL, {}, {}

        # Verify they are alive
        if hw.pz is None or hw.sensor is None:
            print("  [FAILURE] Drivers failed to initialize despite known buses.")
            return StepStatus.FATAL, {}, {}

        try:
            hw.init() # Init the motor driver specifically
        except Exception as e:
            print(f"  [FAILURE] Motor driver init failed: {e}")
            return StepStatus.FATAL, {}, {}

        return StepStatus.SUCCESS, {}, {'hardware_init_verified': True}

# --- Step 3: Friction Threshold ---
class FrictionThresholdStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Friction Threshold (Min Power)"

    def is_verified(self, state: LearningState) -> bool:
        return state.friction_threshold_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        print(">>> Finding Minimum Power (Raw) <<<")
        print("Ensuring robot is on the floor...")

        def action(p):
            res = hw.drive_and_measure(p, p, 0.3, wait_for_stability=False)
            time.sleep(0.5)
            return res

        def check(res):
            # Calculate Max Raw Gyro Magnitude
            max_mag = 0.0
            for s in res.samples:
                if s.gyro_raw:
                    mag = glm.length(s.gyro_raw)
                    if mag > max_mag:
                        max_mag = mag

            print(f"    Max Raw Gyro Magnitude: {max_mag:.1f} deg/s")
            return max_mag > 15.0

        heartbeat_fn = hw.watchdog.heartbeat if hw.watchdog else None
        found = find_threshold("Minimum Power", 0, 5, 100, action, check, heartbeat_fn=heartbeat_fn)

        if found is None:
            return StepStatus.FATAL, {}, {}

        return StepStatus.SUCCESS, {}, {
            'min_power_visible': found,
            'friction_threshold_verified': True
        }

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
        print(f"  Pulsing {name}...")

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
             print(f"  [FAILURE] No samples collected for {name} Pulse.")
             return None, None

        gyro = self._avg_vec([s.gyro_raw for s in res.samples if s.gyro_raw])
        accel = self._avg_vec([s.accel_raw for s in res.samples if s.accel_raw])
        return gyro, accel

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        print(">>> Deriving Kinematics (Single Wiggle) <<<")
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
        print("  Waiting for stability before pulsing RIGHT Motor...")
        hw.wait_for_stability()

        # 2. Pulse Right Only (+PWM)
        r_gyro, r_accel = self._pulse_and_measure(hw, 0, test_power, "RIGHT Motor")
        if r_gyro is None:
            return StepStatus.NEEDS_RETRY, {}, {}
        r_accel_delta = r_accel - baseline_accel

        # --- Phase Alignment ---
        print("  [Analysis] Checking Motor Phase...")
        up_dir = glm.normalize(baseline_accel)
        yaw_mag_sum = abs(glm.dot(l_gyro + r_gyro, up_dir))
        yaw_mag_diff = abs(glm.dot(l_gyro - r_gyro, up_dir))

        motor_r_invert = False

        if yaw_mag_sum > yaw_mag_diff:
            print("  -> Phase Mismatch Detected. Inverting Right Motor logic.")
            motor_r_invert = True
            # Flip R vectors to match L for math
            r_gyro = -r_gyro
            r_accel_delta = -r_accel_delta
        else:
            print("  -> Phase Matched.")

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
        print("  [Analysis] Checking L/R Identity...")
        # Yaw Axis: Dominant axis of (L_gyro_raw - R_gyro_raw) ?
        # Or just use Vertical Axis (Z)? Usually Yaw is around Vertical.
        # Let's use Vertical Axis as Yaw Axis.
        gyro_yaw_axis = vert_axis_name

        # Physical Up Vector = Lag x Pitch Backward
        raw_up = glm.cross(sum_accel, sum_gyro)

        # Yaw Diff
        yaw_diff = l_gyro - r_gyro

        check_val = glm.dot(yaw_diff, raw_up)
        print(f"    Yaw Check Dot Product: {check_val:.2f}")

        swap_motors = False
        if check_val > 0:
            print("  -> Yaw Direction Opposite. Motors are physically swapped.")
            swap_motors = True
        else:
            print("  -> Yaw Direction Correct.")

        # --- Calculate Yaw Invert ---
        raw_yaw_val = getattr(l_gyro, gyro_yaw_axis)
        if swap_motors:
            # l_gyro was physically a LEFT turn. We expect the mapped Yaw to be Positive.
            gyro_yaw_invert = raw_yaw_val < 0
        else:
            # l_gyro was physically a RIGHT turn. We expect the mapped Yaw to be Negative.
            gyro_yaw_invert = raw_yaw_val > 0

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

# --- Step 5: Motor Trim ---
class MotorTrimStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Motor Trim Calibration"

    def is_verified(self, state: LearningState) -> bool:
        return state.motor_trim_verified

    def _build_smooth_ramp(self, l_p: float, r_p: float, total_duration: float) -> list[tuple[float, float, float]]:
        steps = []
        ramp_duration = 0.1
        ramp_steps = 5
        hold_duration = total_duration - (2 * ramp_duration)

        # Ramp up
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps.append((l_p * factor, r_p * factor, ramp_duration / ramp_steps))

        # Hold
        if hold_duration > 0:
            steps.append((l_p, r_p, hold_duration))

        # Ramp down
        for i in range(ramp_steps - 1, -1, -1):
            factor = i / ramp_steps
            steps.append((l_p * factor, r_p * factor, ramp_duration / ramp_steps))

        return steps

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        print(">>> Motor Trim Calibration <<<")
        hw.wait_for_stability()

        best_trim = state.motor_trim

        for i in range(15):
            p = state.min_power_visible + 15

            # Drive forward with smooth ramps
            fwd_steps = self._build_smooth_ramp(p, p, 1.0)
            res = hw.execute_maneuver(fwd_steps, trim_override=best_trim)
            hw.wait_for_stability()

            # Reverse to roughly the starting position with smooth ramps
            rev_steps = self._build_smooth_ramp(-p, -p, 1.0)
            hw.execute_maneuver(rev_steps, trim_override=best_trim)
            hw.wait_for_stability()

            if not res.samples:
                continue
            avg_yaw = res.avg_yaw_rate
            print(f"    Trim: {best_trim:.3f}, Drift: {avg_yaw:.2f} d/s")

            if abs(avg_yaw) < 2.0:
                print("  [SUCCESS] Drift is negligible.")
                return StepStatus.SUCCESS, {}, {'motor_trim': best_trim, 'motor_trim_verified': True}

            # Adaptive Correction
            gain = 0.015 if abs(avg_yaw) > 10.0 else 0.005
            correction = avg_yaw * gain
            best_trim = max(-0.4, min(0.4, best_trim + correction))

        print("  [WARNING] Could not perfectly trim. Saving best effort.")
        return StepStatus.SUCCESS, {}, {'motor_trim': best_trim, 'motor_trim_verified': True}

# --- Step 6: Mechanical Backlash ---
class MechanicalBacklashStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Mechanical Backlash"

    def is_verified(self, state: LearningState) -> bool:
        return state.backlash_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        print(">>> Measuring Mechanical Backlash <<<")
        hw.wait_for_stability(duration=1.0)

        test_power = state.min_power_visible + 10

        # Forward Ramp
        steps_fwd = []
        ramp_steps = 5
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps_fwd.append((test_power * factor, test_power * factor, 0.05))
        steps_fwd.append((test_power, test_power, 0.3))

        # Ramp down
        for i in range(ramp_steps - 1, -1, -1):
            factor = i / ramp_steps
            steps_fwd.append((test_power * factor, test_power * factor, 0.05))

        hw.execute_maneuver(steps_fwd)
        hw.stop()
        hw.wait_for_stability(duration=1.0)

        # Start timing reverse maneuver
        start_time = time.time()

        # Execute the ramp while checking for movement to capture slop properly
        slop_time = 0.2
        reading = None
        moved = False

        for i in range(1, 4):
            factor = i / 3.0
            hw.set_motors(-test_power * factor, -test_power * factor)

            # Wait 0.05s while checking for movement
            step_start = time.time()
            while time.time() - step_start < 0.05:
                reading = hw.read_imu_converted()
                if abs(reading.pitch_rate) > 5.0:
                    slop_time = time.time() - start_time
                    moved = True
                    break
                time.sleep(0.005)

            if moved:
                break

        # If hasn't moved yet during ramp, hold full power and keep timing
        if not moved:
            hw.set_motors(-test_power, -test_power)
            while time.time() - start_time < 1.0:
                reading = hw.read_imu_converted()
                if abs(reading.pitch_rate) > 5.0:
                    slop_time = time.time() - start_time
                    break
                time.sleep(0.005)

        # Ramp down from reverse
        steps_rev_down = []
        for i in range(2, -1, -1):  # Quick 3-step ramp down
            factor = i / 3.0
            steps_rev_down.append((-test_power * factor, -test_power * factor, 0.05))
        hw.execute_maneuver(steps_rev_down)

        hw.stop()
        compensated = max(0.0, slop_time - 0.02)
        print(f"  Backlash: {compensated:.3f}s")

        return StepStatus.SUCCESS, {}, {
            'control': state.control.model_copy(update={'backlash_pulse_time': compensated}),
            # Note: updating nested pydantic model requires full replacement or smart merge.
            # LearningState.control is ControlConfig.
            # We should provide the updated ControlConfig object.
            # But `state.control` is mutable? No, ControlConfig is a Pydantic model.
            # If `LearningState` defines `control: ControlConfig`, we need to update the field.
            # The dictionary approach requires the pipeline to handle nested updates?
            # Usually `model_copy(update=dict)` is shallow.
            # So I should pass `control` key with the NEW ControlConfig object.
            'backlash_verified': True
        }

# --- Step 7: Kickup Dynamics ---
class KickupDynamicsStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Kick-Up Dynamics"

    def is_verified(self, state: LearningState) -> bool:
        return state.kickup_dynamics_verified

    def _force_posture(self, hw: RobotHardware, target_sign: float, base_power: float) -> None:
        """Helper to force the robot into a specific posture (+1 for BACK, -1 for FRONT)."""
        target_name = "BACK" if target_sign > 0 else "FRONT"

        print(f"  [Posture Check] Verifying initial physical state before trying to flop {target_name}...")
        hw.wait_for_stability(1.0)

        # Take multiple readings to be absolutely sure we're not just bouncing
        samples = []
        for _ in range(5):
            samples.append(hw.read_imu_converted().pitch_angle)
            time.sleep(0.05)

        avg_pitch = sum(samples) / len(samples)
        pitch_variance = max(samples) - min(samples)

        print(f"  [Posture Check] Measured average pitch: {avg_pitch:.1f}° (Variance: {pitch_variance:.1f}°)")

        # Define a high-confidence resting threshold (e.g. resting on bumpers usually means > 30 degrees)
        RESTING_THRESHOLD = 25.0

        # Check if we are already securely flopped in the correct direction
        if target_sign > 0 and avg_pitch < -RESTING_THRESHOLD and pitch_variance < 5.0:
            print(f"  [Posture Check] I am REALLY sure I'm on my BACK. I know this because stable pitch is {avg_pitch:.1f}° (Threshold is < -{RESTING_THRESHOLD}°). No flop needed.")
            return
        elif target_sign < 0 and avg_pitch > RESTING_THRESHOLD and pitch_variance < 5.0:
            print(f"  [Posture Check] I am REALLY sure I'm on my FRONT. I know this because stable pitch is {avg_pitch:.1f}° (Threshold is > {RESTING_THRESHOLD}°). No flop needed.")
            return

        print(f"  [Posture Check] Current pitch is {avg_pitch:.1f}°. Need to actively flop to {target_name}. Initiating maneuver...")

        # Increment power until we reach max_power
        p = base_power
        while p <= 100:
            print(f"  Flop to {target_name} (Power {p:.0f}%)...")
            motor_p = p * target_sign

            # Ramp
            steps = []
            ramp_steps = 5
            for i in range(1, ramp_steps + 1):
                factor = i / ramp_steps
                steps.append((motor_p * factor, motor_p * factor, 0.05))
            steps.append((motor_p, motor_p, 0.4 - 0.25))

            # Ramp down
            for i in range(ramp_steps - 1, -1, -1):
                factor = i / ramp_steps
                steps.append((motor_p * factor, motor_p * factor, 0.05))

            hw.execute_maneuver(steps)

            hw.wait_for_stability(1.0)

            # Post-maneuver verification
            samples_post = []
            for _ in range(3):
                samples_post.append(hw.read_imu_converted().pitch_angle)
                time.sleep(0.05)

            avg_pitch_post = sum(samples_post) / len(samples_post)

            # Keep the old, less strict bounds for the actual flop attempt just to detect if we successfully passed the 0 degree point
            if target_sign > 0 and avg_pitch_post < -10:
                print(f"  [Posture Check] Flop successful. Settled at {avg_pitch_post:.1f}°")
                return
            if target_sign < 0 and avg_pitch_post > 10:
                print(f"  [Posture Check] Flop successful. Settled at {avg_pitch_post:.1f}°")
                return

            # Revert movement to avoid hitting walls
            print("  [Posture Check] Flop failed. Reversing to start position...")
            reverse_steps = [(-l, -r, d) for l, r, d in steps]
            hw.execute_maneuver(reverse_steps)
            hw.wait_for_stability(1.0)

            p += 10.0

        raise RuntimeError("Failed to posture")

    def _attempt_kick(self, hw: RobotHardware, start_sign: float, p: float) -> str:
        """Helper to attempt a kick-up maneuver. start_sign: +1 for BACK, -1 for FRONT."""
        kick_sign = -start_sign
        setup_sign = -kick_sign

        setup_p = p * setup_sign * 0.7
        kick_p = p * kick_sign * 1.0

        # Ramp setup power
        steps = []
        ramp_steps = 5
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps.append((setup_p * factor, setup_p * factor, 0.05))

        # Setup hold
        steps.append((setup_p, setup_p, 0.3 - 0.25))

        # Ramp kick power (fast)
        for i in range(1, 3):
            factor = i / 2.0
            steps.append((kick_p * factor, kick_p * factor, 0.02))

        # Kick hold
        steps.append((kick_p, kick_p, 0.4 - 0.04))

        # Ramp kick power down (fast)
        for i in range(1, -1, -1):
            factor = i / 2.0
            steps.append((kick_p * factor, kick_p * factor, 0.02))

        hw.execute_maneuver(steps)

        time.sleep(1.0)
        pitch = hw.read_imu_converted().pitch_angle

        # Check Success
        if start_sign > 0 and -10 < pitch < 20:
            return "SUCCESS"
        if start_sign < 0 and -20 < pitch < 10:
            return "SUCCESS"
        return "FAIL"

    def _run_kickup_test(self, hw: RobotHardware, state: LearningState, direction_sign: float) -> Optional[float]:
        """Runs the threshold search for a specific kick-up direction. direction_sign: +1 for BACK, -1 for FRONT."""
        dir_name = "BACK" if direction_sign > 0 else "FRONT"

        def action(p):
            try:
                self._force_posture(hw, direction_sign, state.min_power_visible + 10)
            except RuntimeError:
                return "POSTURE_FAIL"
            return self._attempt_kick(hw, direction_sign, p)

        return find_threshold(f"KickUp {dir_name}",
                               0,
                               5, 100,
                               action,
                               lambda r: r == "SUCCESS",
                               heartbeat_fn=hw.watchdog.heartbeat if hw.watchdog else None)

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        print(">>> Dynamic Kick-Up Calibration <<<")

        fwd = self._run_kickup_test(hw, state, 1.0) # Kickup Forward (from Back, +1)
        if fwd is None:
            return StepStatus.NEEDS_RETRY, {}, {}

        bwd = self._run_kickup_test(hw, state, -1.0) # Kickup Backward (from Front, -1)
        if bwd is None:
            return StepStatus.NEEDS_RETRY, {}, {}

        # Construct new ControlConfig
        new_control = state.control.model_copy(update={
            'kickup_power_forward': fwd,
            'kickup_power_backward': bwd
        })

        return StepStatus.SUCCESS, {}, {
            'control': new_control,
            'kickup_dynamics_verified': True
        }
