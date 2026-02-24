import sys
import time
import math
import random
import logging
import glm
from typing import Tuple

from .step import CalibrationStep
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..watchdog import SurvivalWatchdog
from ..enums import Axis
from ..utils import (
    analyze_dominance,
    scan_i2c_or_die,
    verify_with_retries,
    find_threshold,
    sort_resting_vectors,
    vector_angle,
    clamp,
    calculate_pitch
)

logger = logging.getLogger(__name__)

# --- Step 1: Discover Buses ---
class DiscoverBusesStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Discover I2C Buses"

    def is_verified(self, state: LearningState) -> bool:
        return state.i2c_buses_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print("Scanning I2C Buses...")

        # 1. Find Motors (0x22)
        def check_motor(bus):
            try:
                bus.read_byte_data(0x22, 0)
                return True
            except:
                return False

        found_motor_bus = scan_i2c_or_die("PiconZero (Motors)", check_motor)

        # 2. Find IMU (0x68)
        def check_imu(bus):
            try:
                return bus.read_byte_data(0x68, 0x75) == 0x68
            except:
                return False

        found_imu_bus = scan_i2c_or_die("MPU6050 (IMU)", check_imu)

        # Update State
        state.i2c_buses_verified = True

        new_config = config.model_copy(update={
            'motor_i2c_bus': found_motor_bus,
            'imu_i2c_bus': found_imu_bus
        })

        return new_config, True

# --- Step 2: Hardware Init ---
class HardwareInitStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Initialize Hardware Drivers"

    def is_verified(self, state: LearningState) -> bool:
        return state.hardware_init_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print("Initializing Hardware Drivers...")
        # Force driver initialization now that buses are known
        hw.initialize_drivers()

        # Verify they are alive
        if hw.pz is None or hw.sensor is None:
            print("  [FAILURE] Drivers failed to initialize despite known buses.")
            return config, False

        hw.init() # Init the motor driver specifically

        state.hardware_init_verified = True
        return config, True

# --- Step 3: Friction Threshold ---
class FrictionThresholdStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Friction Threshold (Min Power)"

    def is_verified(self, state: LearningState) -> bool:
        return state.friction_threshold_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
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

        heartbeat_fn = watchdog.heartbeat if watchdog else None
        found = find_threshold("Minimum Power", 10, 5, 100, action, check, heartbeat_fn=heartbeat_fn)

        state.min_power_visible = found
        state.friction_threshold_verified = True

        return config, True

# --- Step 4: Calibrate Gravity (Toddler Flail) ---
class CalibrateGravityStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Toddler Flail (Gravity Calibration)"

    def is_verified(self, state: LearningState) -> bool:
        return state.spatial_orientation_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print(">>> Calibrating Orientation (The Labeler) <<<")

        # 1. Flail Routine
        duration = 24.0
        print(f"  [Toddler Flail] Starting chaos routine for {duration}s...")
        collected = []
        start_time = time.time()

        while time.time() - start_time < duration:
            if watchdog:
                watchdog.heartbeat()

            base = state.min_power_visible + 30
            extra = random.uniform(0, 40)
            power = clamp(base + extra, 0, 100)
            sign = 1.0 if random.random() > 0.5 else -1.0
            power_signed = power * sign

            # Burst
            hw.drive_and_measure(power_signed, power_signed, 0.4)
            hw.wait_for_stability(duration=1.0)

            # Collect
            vec = hw.measure_gravity()
            collected.append(vec)

        # 2. Sort into two buckets
        print(f"  [Analysis] Sorting {len(collected)} vectors...")
        try:
            p1, p2 = sort_resting_vectors(collected)
        except ValueError as e:
             print(f"  [FAILURE] Sorting failed: {e}")
             return config, False

        print(f"  [DEBUG] Resting Position A: {p1}")
        print(f"  [DEBUG] Resting Position B: {p2}")

        spread = vector_angle(p1, p2)
        print(f"  [DEBUG] Angle Spread: {spread:.1f} degrees")

        if spread < 15.0:
            print(f"  [CRITICAL] Resting vectors too close ({spread:.1f} deg).")
            return config, False

        # 3. Determine Vertical Axis
        # Since we don't know Forward/Pitch yet, we pick the most dominant axis overall?
        # No, Toddler Flail usually happens AFTER Yank in the old code.
        # But here it is BEFORE.
        # If it is BEFORE, we don't know 'accel_forward_axis'.
        # However, Gravity is usually the STRONGEST acceleration (1g).
        # In a resting robot (leaning), gravity is distributed.
        # But p1 and p2 should have the SAME vertical component (relative to the robot chassis)?
        # No, if the robot flips, the vertical axis relative to world is constant, but relative to robot?
        # If the robot is leaning forward vs backward.
        # The axis that changes LEAST is the pitch axis (rotation).
        # The axis that changes SIGN is the Forward/Vertical mix.
        # Actually, let's look at the logic.
        # "Look at the raw gravity vector of p1. Excluding the now-known accel_forward_axis..."
        # If we don't know them, we must search for the axis that is consistently large?
        # Or maybe the axis that is NOT the Pitch Axis?
        # Pitch Axis (Gyro) corresponds to the axis around which p1 rotates to p2.
        # We can find the Pitch Axis by Cross Product of p1 and p2?
        # p1 and p2 are gravity vectors. The rotation between them is around the Pitch Axis.
        # So Cross(p1, p2) gives the Pitch Axis!

        pitch_axis_vec = glm.cross(p1, p2)
        pitch_axis_name, _, _ = analyze_dominance(pitch_axis_vec, "Derived Pitch Axis")

        # Now we know Pitch Axis.
        # The Vertical Axis is one of the other two.
        # Which one?
        # In a balancing robot, "Vertical" usually aligns with Gravity when balanced.
        # When resting (leaning), Gravity has components in Vertical and Forward.
        # But usually Vertical component is dominant if lean is small (<45 deg).
        # Let's assume the component with largest magnitude in p1 (excluding pitch axis) is Vertical.

        candidates = {k: abs(getattr(p1, k)) for k in ['x', 'y', 'z'] if k != pitch_axis_name}
        vert_axis_name, _, _ = analyze_dominance(candidates, "Vertical Axis")

        vert_val = getattr(p1, vert_axis_name)
        invert_vertical = vert_val < 0

        # Yaw is typically the same axis as Vertical for Gyro?
        # Yes.

        state.spatial_orientation_verified = True

        new_config = config.model_copy(update={
            'accel_vertical_axis': Axis(vert_axis_name),
            'accel_vertical_invert': invert_vertical,
            'gyro_yaw_axis': Axis(vert_axis_name)
            # We don't know gyro_yaw_invert yet.
        })

        return new_config, True

# --- Step 5: Determine Motor Direction (The Yank) ---
class DetermineMotorDirectionStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Motor Direction (Yank Test)"

    def is_verified(self, state: LearningState) -> bool:
        return state.motor_direction_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print(">>> Determining Motor Direction (The Yank Test) <<<")

        # 1. Baseline
        hw.wait_for_stability()
        baseline_accel, _ = hw.read_imu_raw()

        # 2. Pulse Forward (+PWM)
        power = state.min_power_visible + 30
        res = hw.drive_and_measure(power, power, 0.3, wait_for_stability=False)
        time.sleep(0.5)

        if not res.samples:
            return config, False

        # 3. Find Pitch Axis (Gyro)
        gyro_sum = glm.vec3(0.0)
        for s in res.samples:
            if s.gyro_raw:
                gyro_sum += s.gyro_raw
        avg_gyro = gyro_sum / len(res.samples)

        pitch_axis_name, _, _ = analyze_dominance(avg_gyro, "Pitch Axis")

        # Invert: We want +PWM -> Forward Motion -> Backward Pitch (Inertial) -> Negative Pitch Rate.
        # So if detected > 0, we must invert.
        detected_val = getattr(avg_gyro, pitch_axis_name)
        invert_pitch = detected_val > 0

        # 4. Find Forward Axis (Accel)
        # Delta = Moving - Baseline
        accel_sum = glm.vec3(0.0)
        for s in res.samples:
            if s.accel_raw:
                accel_sum += s.accel_raw
        avg_accel = accel_sum / len(res.samples)
        delta_accel = avg_accel - baseline_accel

        # Exclude Pitch Axis (and Vertical Axis from step 4?)
        # We know Pitch Axis.
        candidates = {k: abs(getattr(delta_accel, k)) for k in ['x', 'y', 'z'] if k != pitch_axis_name}

        # We might check if it conflicts with Vertical Axis from previous step?
        # But let's trust the Yank data for Forward.
        fwd_axis_name, _, _ = analyze_dominance(candidates, "Forward Axis")

        # Invert: +PWM -> Forward Accel -> Delta should be NEGATIVE (Lag)?
        # "set accel_forward_invert so this delta reads as NEGATIVE."
        delta_val = getattr(delta_accel, fwd_axis_name)
        invert_forward = delta_val > 0

        state.motor_direction_verified = True

        new_config = config.model_copy(update={
            'gyro_pitch_axis': Axis(pitch_axis_name),
            'gyro_pitch_invert': invert_pitch,
            'accel_forward_axis': Axis(fwd_axis_name),
            'accel_forward_invert': invert_forward
        })

        return new_config, True

# --- Step 6: Verify Motor Phase ---
class VerifyMotorPhaseStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Verify Motor Phase"

    def is_verified(self, state: LearningState) -> bool:
        return state.motor_phasing_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print(">>> Aligning Motors Phase <<<")

        def test(attempt: int):
            p = state.min_power_visible + 20 + (attempt * 10)
            hw.wait_for_stability()
            return hw.drive_and_measure(p, p, 0.5, wait_for_stability=False)

        def verify(res):
            if not res or not res.samples: return False

            # Use MAPPED values since we have axes now
            # Check 1: Movement?
            accel_mags = [glm.length(s.accel_raw) for s in res.samples if s.accel_raw]
            if not accel_mags: return False
            accel_range = max(accel_mags) - min(accel_mags)

            # Check 2: Spinning? (Motors fighting)
            # Use avg_yaw_rate (mapped)
            avg_yaw = res.abs_avg_yaw_rate
            print(f"    Accel Range: {accel_range:.3f}g, Avg Yaw: {avg_yaw:.1f} d/s")

            if avg_yaw > 30.0:
                print("  -> Spinning detected. Inverting Right Motor.")
                # We need to return False and modify config OUTSIDE?
                # The step protocol says we can return new_config.
                # But verify_with_retries expects a boolean.
                # We can hack it by modifying a local variable?
                return "INVERT_RIGHT"

            if accel_range > 0.1:
                return True

            return False

        # Custom retry loop since verify_with_retries doesn't support config updates nicely
        for attempt in range(5):
            res = test(attempt)
            outcome = verify(res)

            if outcome == True:
                state.motor_phasing_verified = True
                return config, True
            elif outcome == "INVERT_RIGHT":
                new_config = config.model_copy(update={'motor_r_invert': not config.motor_r_invert})
                # Must restart verification with new config
                # But we can't easily restart inside run() without recursion or loop.
                # We return (new_config, False) to force the pipeline to retry this step?
                # "MUST return (new_config, True) on success, or (config, False) to force a retry."
                # If we return (new_config, False), the pipeline halts?
                # "if success: ... else: raise RuntimeError"
                # The pipeline halts on False!
                # Wait, existing pipeline code:
                # if success: save... else: raise error.
                # So we CANNOT retry from outside. We must handle retries INSIDE.
                # So we must update config inside this loop and apply it to hw manually for testing?
                # "NEVER mutate config ... until the very last line"
                # This makes it hard to test the new config.
                # But we can update `config` local variable and `hw.apply_config` temporary?
                # But step signature: `run(..., config, ...)`
                # I should update local `config` and call `hw.apply_config(config)`.
                config = config.model_copy(update={'motor_r_invert': not config.motor_r_invert})
                hw.apply_config(config)
                print("  -> Retrying with new phase...")
                time.sleep(1.0)
                continue
            else:
                print("  -> No movement/Unclear. Retrying...")

        return config, False

# --- Step 7: Left/Right Identity ---
class LeftRightIdentityStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Left/Right Identity"

    def is_verified(self, state: LearningState) -> bool:
        return state.motor_channels_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print(">>> Autonomous Left/Right Verification <<<")

        def test(attempt: int):
            hw.wait_for_stability()
            # Drive Arc: Left High, Right Low (Logical)
            p_high = state.min_power_visible + 20 + (attempt * 10)
            p_low = p_high * 0.5
            return hw.execute_maneuver([(p_high, p_low, 1.5)])

        def verify(res):
            if not res or not res.samples: return None
            # Use avg_yaw_rate (mapped)
            return res.avg_yaw_rate

        final_config = config

        for attempt in range(5):
            avg_yaw = test(attempt)
            if avg_yaw is None or abs(avg_yaw) < 10.0:
                print("  [WARNING] Spin too low.")
                continue

            print(f"  Avg Yaw Rate: {avg_yaw:.1f} deg/s")

            # Convention:
            # We drove Left > Right.
            # Standard Differential Drive: Turn Right (Clockwise).
            # Standard Gyro (Z Up): CW is Negative.
            # So we expect Negative Yaw.

            if avg_yaw > 0: # Positive -> CCW -> Turn Left
                print("  -> Detected Left Turn (CCW) when expecting Right (CW).")
                print("  -> ACTION: Swapping Motor Channels.")
                final_config = config.model_copy(update={
                    'motor_l': config.motor_r,
                    'motor_r': config.motor_l,
                    'motor_l_invert': config.motor_r_invert,
                    'motor_r_invert': config.motor_l_invert
                })
            else:
                print("  -> Detected Right Turn (CW) as expected.")

            state.motor_channels_verified = True
            return final_config, True

        return config, False

# --- Step 8: Motor Trim ---
class MotorTrimStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Motor Trim Calibration"

    def is_verified(self, state: LearningState) -> bool:
        return state.motor_trim_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print(">>> Motor Trim Calibration <<<")
        hw.wait_for_stability()

        best_trim = state.motor_trim

        for i in range(15):
            p = state.min_power_visible + 15
            res = hw.drive_and_measure(p, p, 1.0)

            if not res.samples: continue
            avg_yaw = res.avg_yaw_rate
            print(f"    Trim: {best_trim:.3f}, Drift: {avg_yaw:.2f} d/s")

            if abs(avg_yaw) < 2.0:
                print("  [SUCCESS] Drift is negligible.")
                state.motor_trim = best_trim
                state.motor_trim_verified = True
                return config, True

            # Adaptive Correction
            gain = 0.015 if abs(avg_yaw) > 10.0 else 0.005
            correction = avg_yaw * gain
            best_trim = max(-0.4, min(0.4, best_trim + correction))

            # Update state temporarily for next iteration?
            # State mutation is discouraged but 'state.motor_trim' is used by hw.set_motors immediately.
            # We must update it.
            state.motor_trim = best_trim

        print("  [WARNING] Could not perfectly trim. Saving best effort.")
        state.motor_trim_verified = True
        return config, True

# --- Step 9: Mechanical Backlash ---
class MechanicalBacklashStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Mechanical Backlash"

    def is_verified(self, state: LearningState) -> bool:
        return state.backlash_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print(">>> Measuring Mechanical Backlash <<<")
        hw.wait_for_stability(duration=1.0)

        test_power = state.min_power_visible + 10

        # Forward
        hw.set_motors(test_power, test_power)
        time.sleep(0.3)
        hw.stop()
        hw.wait_for_stability(duration=1.0)

        # Reverse & Time
        start_time = time.time()
        hw.set_motors(-test_power, -test_power)

        slop_time = 0.2
        while time.time() - start_time < 1.0:
            reading = hw.read_imu_converted()
            if abs(reading.pitch_rate) > 5.0:
                slop_time = time.time() - start_time
                break
            time.sleep(0.005)

        hw.stop()
        compensated = max(0.0, slop_time - 0.02)
        print(f"  Backlash: {compensated:.3f}s")

        state.control.backlash_pulse_time = compensated
        state.backlash_verified = True
        return config, True

# --- Step 10: Kickup Dynamics ---
class KickupDynamicsStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Kick-Up Dynamics"

    def is_verified(self, state: LearningState) -> bool:
        return state.kickup_dynamics_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        print(">>> Dynamic Kick-Up Calibration <<<")

        # Helpers
        def force_posture(target: str):
            base = state.min_power_visible + 10
            for i in range(5):
                hw.wait_for_stability(1.0)
                pitch = hw.read_imu_converted().pitch_angle
                if target == "FRONT" and pitch > 10: return
                if target == "BACK" and pitch < -10: return

                print(f"  Flop to {target}...")
                p = base + (i*10)
                if target == "FRONT": hw.drive_and_measure(-p, -p, 0.4)
                else: hw.drive_and_measure(p, p, 0.4)
            sys.exit(1) # Watchdog

        def attempt_kick(start: str, p: float):
            kick_sign = -1.0 if start == "BACK" else 1.0
            setup_sign = -kick_sign

            setup_p = p * setup_sign * 0.7
            kick_p = p * kick_sign * 1.0

            res = hw.execute_maneuver([
                (setup_p, setup_p, 0.3),
                (kick_p, kick_p, 0.4)
            ])

            time.sleep(1.0)
            pitch = hw.read_imu_converted().pitch_angle

            # Check Success
            if start == "BACK" and -10 < pitch < 20: return "SUCCESS"
            if start == "FRONT" and -20 < pitch < 10: return "SUCCESS"
            return "FAIL"

        def run_test(direction: str, attr: str):
            def action(p):
                force_posture(direction)
                return attempt_kick(direction, p)

            found = find_threshold(f"KickUp {direction}",
                                   max(20, state.min_power_visible + 10),
                                   5, 100,
                                   action,
                                   lambda r: r == "SUCCESS",
                                   heartbeat_fn=watchdog.heartbeat if watchdog else None)
            if found:
                setattr(state.control, attr, found)

        run_test("BACK", "kickup_power_forward")
        run_test("FRONT", "kickup_power_backward")

        state.kickup_dynamics_verified = True
        return config, True
