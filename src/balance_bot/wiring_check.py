import time
import sys
import math

from typing import Callable, Any, Optional
from .configuration import HardwareConfig, LearningState
from .hardware.robot_hardware import RobotHardware, IMUReading
from .watchdog import SurvivalWatchdog
import random
import glm
from .enums import Axis
from .utils import (
    analyze_dominance,
    get_i2c_failure_report,
    run_diagnostics,
    scan_i2c_candidates,
    scan_i2c_or_die,
    verify_with_retries,
    find_threshold,
    sort_resting_vectors,
    vector_angle,
    clamp
)


class WiringCheck:
    """
    Zero-Knowledge "Self-Discovery" Wiring Check.
    Implements a state-machine that incrementally discovers robot configuration.
    See LEARN.md for specification.
    """

    def __init__(self, watchdog: Optional[SurvivalWatchdog] = None):
        self.hw_config = HardwareConfig.load()
        self.learning_state = LearningState.load()
        self.hw = None
        self.watchdog = watchdog

    def init_hw(self):
        """
        Initialize hardware with current known config.
        If config is incomplete, fills in safe defaults for discovery.
        """
        if self.hw:
            return

        # We need buses to be discovered first
        if self.hw_config.motor_i2c_bus is None or self.hw_config.imu_i2c_bus is None:
            # Cannot init HW without buses.
            return

        self.hw = RobotHardware(self.hw_config, self.learning_state, watchdog=self.watchdog)
        self.hw.init()

    def cleanup(self):
        if self.hw:
            self.hw.stop()
            self.hw.cleanup()

    def _update_hw_config(self, **kwargs):
        """Helper to update immutable HardwareConfig."""
        self.hw_config = self.hw_config.model_copy(update=kwargs)
        self.hw_config.save()
        if self.hw:
            self.hw.hw_config = self.hw_config

    def _update_learning_state(self, **kwargs):
        """Helper to update mutable LearningState."""
        for k, v in kwargs.items():
            setattr(self.learning_state, k, v)
        self.learning_state.save()

    def _set_default_motors(self):
        print("-> [INFO] Setting default motors (0,1).")
        self._update_hw_config(motor_l=0, motor_r=1)
        # Re-init hardware to pick up new config if needed,
        # though RobotHardware doesn't cache motor channels, but let's be safe.
        # Actually init_hw checks if self.hw is set. If we want to force update config,
        # we might need to recreate self.hw?
        # RobotHardware holds a reference to hw_config. But hw_config is immutable and replaced on update.
        # So we MUST re-create RobotHardware.
        if self.hw:
            self.hw.cleanup()
            self.hw = None
        self.init_hw()

    def run(self):
        """
        Main Knowledge Dependency Loop.
        Iterates until all configuration requirements are met.
        """
        print("Beginning Self-Discovery Protocol...")
        run_diagnostics()

        steps = [
            ("I2C Bus Assignments",
             lambda: self.hw_config.motor_i2c_bus is None or self.hw_config.imu_i2c_bus is None,
             self.discover_buses),

            ("Hardware Initialization",
             lambda: self.hw is None,
             self.init_hw),

            ("Motor Candidates",
             lambda: self.hw_config.motor_l is None or self.hw_config.motor_r is None,
             self._set_default_motors),

            ("Friction Threshold",
             lambda: self.learning_state.min_power_visible == 0,
             self.find_min_power),

            ("Spatial Orientation",
             lambda: self.hw_config.accel_vertical_axis is None,
             lambda: (self.init_hw(), self.calibrate_static_orientation())),

            ("Motor Phasing",
             lambda: not self.learning_state.motor_phasing_verified,
             self.align_motors_phase),

            ("Motor Direction",
             lambda: not self.learning_state.motor_direction_verified,
             self.determine_motor_direction),

            ("Left/Right Identity",
             lambda: not self.learning_state.motor_channels_verified,
             self.deduce_left_right_autonomous),

            ("Motor Trim",
             lambda: not self.learning_state.motor_trim_verified,
             lambda: (self.calibrate_motor_trim(), self._update_learning_state(motor_trim_verified=True))),

            ("Mechanical Backlash",
             lambda: not self.learning_state.backlash_verified,
             self.measure_backlash),

            ("Kick-Up Dynamics",
             lambda: self.learning_state.control.kickup_power_forward == 0.0,
             self.find_flop_thresholds)
        ]

        while True:
            if self.watchdog:
                self.watchdog.heartbeat()

            print("\n---------------------------------------------------")
            print("Checking Knowledge Base...")

            action_taken = False
            for name, condition, action in steps:
                if condition():
                    print(f"-> [MISSING] {name}.")
                    action()
                    # Configs are saved within actions
                    action_taken = True
                    break # Restart loop

            if action_taken:
                continue

            # All steps passed
            print("-> [VERIFYING] Final Configuration Check.")
            self.verify_final_configuration()
            self._print_summary()
            break

        self.cleanup()

    def _print_summary(self):
        h = self.hw_config
        l = self.learning_state
        print("\n[SUCCESS] Hardware Verified. Ready for Agent (Main Brain).")
        print("Summary:")
        print(f"  Buses: Motor={h.motor_i2c_bus}, IMU={h.imu_i2c_bus}")
        print(f"  Axes: Vert={h.accel_vertical_axis}, Fwd={h.accel_forward_axis}, Pitch={h.gyro_pitch_axis}, Yaw={h.gyro_yaw_axis}")
        print(f"  Motors: MinPower={l.min_power_visible}, L={h.motor_l}(Inv={h.motor_l_invert}), R={h.motor_r}(Inv={h.motor_r_invert})")
        print(f"  Trim: {l.motor_trim:.3f}")
        print(f"  KickUp: Fwd={l.control.kickup_power_forward:.1f}, Bwd={l.control.kickup_power_backward:.1f}")

    def _drive_and_wait(self, left: float, right: float, duration: float,
                        wait_stable: bool = True) -> Any:
        """Helper to drive, measure, and wait."""
        if wait_stable:
            self.hw.wait_for_stability()
        result = self.hw.drive_and_measure(left, right, duration)
        time.sleep(0.5)
        return result

    def _measure_gravity_with_hardware(self) -> glm.vec3:
        """Measure gravity using hardware abstraction."""
        # Drive 0,0 for 1.0 second
        res = self.hw.drive_and_measure(0, 0, 1.0)
        if not res.samples:
            return glm.vec3(0.0)

        avg = glm.vec3(0.0)
        count = 0
        for s in res.samples:
            if s.accel_raw:
                avg += s.accel_raw
                count += 1
        if count > 0:
            return avg / count
        return glm.vec3(0.0)

    def discover_buses(self):
        """
        Scan I2C buses [1, 3, 0, 2] for PiconZero (0x22) and MPU6050 (0x68).
        """
        print("Scanning I2C Buses...")

        # 1. Find Motors (0x22)
        def check_motor(bus):
            bus.read_byte_data(0x22, 0)
            return True

        found_motor_bus = scan_i2c_or_die("PiconZero (Motors)", check_motor)

        # 2. Find IMU (0x68)
        def check_imu(bus):
            return bus.read_byte_data(0x68, 0x75) == 0x68

        found_imu_bus = scan_i2c_or_die("MPU6050 (IMU)", check_imu)

        self._update_hw_config(motor_i2c_bus=found_motor_bus, imu_i2c_bus=found_imu_bus)

    # --- Phase 2: The Physical World (Sensors) ---
    def _toddler_flail_collection(self, duration=10.0) -> list[glm.vec3]:
        """
        Flail around randomly to collect gravity vectors in various resting states.
        """
        print(f"  [Toddler Flail] Starting chaos routine for {duration}s...")
        collected = []
        start_time = time.time()

        while time.time() - start_time < duration:
            if self.watchdog:
                self.watchdog.heartbeat()

            # Random power: min_power + 30 + random(0..40). Clamped to 100 max.
            base = self.learning_state.min_power_visible + 30
            extra = random.uniform(0, 40)
            power = clamp(base + extra, 0, 100)

            # Random direction
            sign = 1.0 if random.random() > 0.5 else -1.0
            power_signed = power * sign

            # Burst
            print(f"    Flailing: {power_signed:.1f} (0.4s)...")
            self.hw.drive_and_measure(power_signed, power_signed, 0.4)

            # Wait for gravity to settle
            self.hw.wait_for_stability(duration=1.0)

            # Collect
            vec = self._measure_gravity_with_hardware()
            collected.append(vec)
            # print(f"    Collected: {vec}")

        return collected

    def _measure_gravity_vectors(self) -> tuple[glm.vec3, glm.vec3]:
        """
        Measure gravity vector at Back and Front resting positions.
        Uses autonomous 'Toddler Flail' to discover physical limits.
        """
        print(">>> Calibrating Orientation (Autonomous) <<<")

        # 1. Flail & Collect
        vectors = self._toddler_flail_collection(duration=24.0)

        # 2. Sort into two buckets
        print(f"  [Analysis] Sorting {len(vectors)} vectors...")
        try:
            p1, p2 = sort_resting_vectors(vectors)
        except ValueError as e:
             print(f"  [FAILURE] Sorting failed: {e}")
             raise

        print(f"  [DEBUG] Resting Position A: {p1}")
        print(f"  [DEBUG] Resting Position B: {p2}")

        # 3. Failsafe Check
        spread = vector_angle(p1, p2)
        print(f"  [DEBUG] Angle Spread: {spread:.1f} degrees")

        if spread < 15.0:
            msg = f"Resting vectors are too close ({spread:.1f} deg). Is the robot flat on its side?"
            print(f"  [CRITICAL] {msg}")
            raise RuntimeError(msg)

        # 4. Assign Arbitrarily (Direction fixed in later step)
        p_back, p_front = p1, p2

        print("  [SUCCESS] Orientation Calibrated.")
        return p_back, p_front

    def _deduce_axes(self, avg_back: glm.vec3, avg_front: glm.vec3):
        """Deduce Pitch, Vertical, and Forward axes from gravity vectors."""
        # 3. Determine Pitch Axis via Cross Product (Normal to the motion plane)
        pitch_vec = glm.cross(avg_back, avg_front)

        pitch_axis_name, pitch_magnitude, _ = analyze_dominance(
            pitch_vec,
            "Pitch Axis (CrossProd)"
        )

        if pitch_magnitude < 1.1:
            print("  [WARNING] Pitch axis unclear. Did the robot actually move?")

        update_dict = {}
        update_dict['gyro_pitch_axis'] = Axis(pitch_axis_name)
        # Fix for glm access
        pitch_val = getattr(pitch_vec, pitch_axis_name)
        update_dict['gyro_pitch_invert'] = pitch_val < 0
        print(f"  -> Pitch Axis: {pitch_axis_name.upper()} (Invert: {update_dict['gyro_pitch_invert']})")

        # 4. Determine Vertical Axis
        candidates = {k: abs(getattr(avg_back, k)) for k in ['x', 'y', 'z'] if k != pitch_axis_name}
        vert_axis_name, _, _ = analyze_dominance(candidates, "Vertical Axis (Gravity)")

        update_dict['accel_vertical_axis'] = Axis(vert_axis_name)
        # Invert if gravity component is negative
        vert_val = getattr(avg_back, vert_axis_name)
        update_dict['accel_vertical_invert'] = vert_val < 0
        print(f"  -> Vertical Axis: {vert_axis_name.upper()} (Invert: {update_dict['accel_vertical_invert']})")

        # 5. Determine Forward Axis
        all_axes = {'x', 'y', 'z'}
        used = {pitch_axis_name, vert_axis_name}
        remaining = list(all_axes - used)

        if not remaining:
            print("  [CRITICAL ERROR] Axis deduction failed. Overlapping axes.")
            sys.exit(1)

        forward_axis_name = remaining[0]
        update_dict['accel_forward_axis'] = Axis(forward_axis_name)

        # Deduce Forward Inversion
        val_front = getattr(avg_front, forward_axis_name)
        val_back = getattr(avg_back, forward_axis_name)
        delta_fwd = val_front - val_back
        update_dict['accel_forward_invert'] = delta_fwd < 0
        print(f"  -> Forward Axis: {forward_axis_name.upper()} (Invert: {update_dict['accel_forward_invert']})")

        # Deduce Gyro Yaw/Roll
        update_dict['gyro_yaw_axis'] = Axis(vert_axis_name)
        update_dict['gyro_roll_axis'] = Axis(forward_axis_name)

        self._update_hw_config(**update_dict)

    def _calculate_rest_angles(self, avg_back: glm.vec3):
        """Calculate and store rest angles based on deduced axes."""
        print("  Measuring Rest Angles...")
        # Currently at FRONT position
        curr_front_reading = self.hw.read_imu_converted()
        angle_front = curr_front_reading.pitch_angle

        # Calculate Back Angle from avg_back using known axes
        fwd_val_back = self.hw.get_axis_value(avg_back, self.hw_config.accel_forward_axis, self.hw_config.accel_forward_invert)
        vert_val_back = self.hw.get_axis_value(avg_back, self.hw_config.accel_vertical_axis, self.hw_config.accel_vertical_invert)

        from .utils import calculate_pitch
        angle_back = calculate_pitch(fwd_val_back, vert_val_back)

        self._update_learning_state(rest_angle_forward=angle_front, rest_angle_backward=angle_back)

        print(f"  -> Calibrated Rest Angles: Back={angle_back:.1f}, Front={angle_front:.1f}")

        spread = angle_front - angle_back
        if spread < 10.0:
            print(f"  [WARNING] Rest angle spread is very small ({spread:.1f}). Is the robot balanced on a point?")

    def calibrate_static_orientation(self):
        """
        Map Physical Axes (X, Y, Z) to Logical Axes.
        """
        avg_back, avg_front = self._measure_gravity_vectors()
        self._deduce_axes(avg_back, avg_front)
        self._calculate_rest_angles(avg_back)


    # --- Phase 3a: Friction Threshold ---
    def find_min_power(self):
        """
        Find minimum PWM to overcome friction.
        Uses raw gyro magnitude to detect movement without axis calibration.
        """
        print(">>> Finding Minimum Power (Raw) <<<")
        print("Ensuring robot is on the floor...")

        def action(p):
            return self._drive_and_wait(p, p, 0.3, wait_stable=False)

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

        heartbeat_fn = self.watchdog.heartbeat if self.watchdog else None
        found = find_threshold("Minimum Power", 10, 5, 100, action, check, heartbeat_fn=heartbeat_fn)
        self._update_learning_state(min_power_visible=found)

    # --- Phase 3b: Phasing ---
    def align_motors_phase(self):
        """
        Ensure motors spin together (Straight), not opposite (Spin).
        Uses accelerometer for translation check and gyro for spin check.
        """
        print(">>> Aligning Motors Phase (New Protocol) <<<")

        def test(attempt: int):
            # Increased power by +20 (was +10) to overcome static friction
            p = self.learning_state.min_power_visible + 20 + (attempt * 10)
            return self._drive_and_wait(p, p, 0.5)

        def verify(res):
            if not res.samples: return False

            # Check 1 (Did we move?):
            # Calculate Accel Magnitude Range
            accel_mags = [glm.length(s.accel_raw) for s in res.samples if s.accel_raw]
            if not accel_mags: return False
            accel_range = max(accel_mags) - min(accel_mags)

            # Calculate Max Raw Gyro Magnitude
            gyro_mags = [glm.length(s.gyro_raw) for s in res.samples if s.gyro_raw]
            if not gyro_mags: return False
            max_gyro = max(gyro_mags)

            print(f"    Accel Range: {accel_range:.3f}g, Max Gyro: {max_gyro:.1f} deg/s")

            # If both are near zero, we are stuck.
            if accel_range < 0.05 and max_gyro < 10.0:
                 print("  [WARNING] No movement detected (Stuck). Retrying with more power...")
                 return False

            # Check 2 (Are we spinning?):
            # Calculate average yaw rate magnitude
            # Note: We rely on calibrate_static_orientation having run before this, so yaw_rate is valid.
            avg_yaw = sum(abs(s.yaw_rate) for s in res.samples) / len(res.samples)
            print(f"    Avg Yaw Rate: {avg_yaw:.1f} deg/s")

            if avg_yaw > 30.0:
                print("  -> Spinning detected. Motors are fighting.")
                print("  -> ACTION: Inverting Right Motor logic.")
                self._update_hw_config(motor_r_invert=not self.hw_config.motor_r_invert)
                return False

            # Check 3 (Are we translating?):
            # We moved (Check 1) and we are NOT spinning (Check 2).
            # Look for Forward Acceleration or Pitch Change.

            # Calculate Average Forward Accel (Mapped)
            fwd_accels = [self.hw.get_mapped_value(s.accel_raw, "accel_forward") for s in res.samples]
            avg_fwd_accel = sum(fwd_accels) / len(res.samples)

            delta_pitch = abs(res.samples[-1].pitch_angle - res.samples[0].pitch_angle)

            print(f"    Avg Fwd Accel: {avg_fwd_accel:.2f}g, Delta Pitch: {delta_pitch:.1f} deg")

            if avg_fwd_accel > 0.1 or delta_pitch > 5.0:
                 print("  -> Translation detected. Motors are aligned (Straight).")
                 self._update_learning_state(motor_phasing_verified=True)
                 return True

            print("  [WARNING] Movement detected but not clearly translation. Retrying...")
            return False

        verify_with_retries("Motor Phasing", test, verify)

    # --- Phase 4: Sense of Forward (Pitch Axis & Polarity) ---
    def determine_motor_direction(self):
        """
        Ensure Positive Power = "Stand Up" (Reduce Lean).
        "Kick Up" Test.
        Requirement: Robot must be leaning FORWARD (Positive Pitch).
        """
        def test(attempt: int):
            # Check Start Pitch
            imu = self.hw.read_imu_converted()
            start_pitch = imu.pitch_angle
            print(f"  Start Pitch: {start_pitch:.1f}")

            # 1. Ensure we are at FRONT (Positive Pitch)
            if start_pitch < 10.0:
                print(f"  [INFO] Not at FRONT (Pitch={start_pitch:.1f}). Attempting to flop...")
                p = self.learning_state.min_power_visible + 30 + (attempt * 10)
                # Try Positive
                self.hw.drive_and_measure(p, p, 0.5)
                self.hw.wait_for_stability()

                # Check if successful
                if self.hw.read_imu_converted().pitch_angle < 10.0:
                    print("  [INFO] Flop failed. Trying reverse power...")
                    self.hw.drive_and_measure(-p, -p, 0.5)
                    self.hw.wait_for_stability()

            start_pitch = self.hw.read_imu_converted().pitch_angle
            if start_pitch < 10.0:
                print(f"  [WARNING] Could not reach FRONT posture (Pitch={start_pitch:.1f}). Retrying...")
                return None

            # Now we are at Front (Positive Pitch).
            # "Kick Up" means reducing pitch (towards 0).
            # We try Positive Power. If Positive = Forward, pitch should decrease (stand up).
            power = (self.learning_state.min_power_visible + 20) + (attempt * 10)
            print(f"  Pulsing {power}...")

            res = self._drive_and_wait(power, power, 0.4, wait_stable=False)
            return (start_pitch, res)

        def verify(data):
            if data is None: return False
            start_pitch, res = data
            end_pitch = res.final_pitch if res.samples else start_pitch

            delta_lean = abs(start_pitch) - abs(end_pitch)

            # Calculate accel
            avg_accel_fwd = 0.0
            if res.samples:
                fwd_axis = self.hw_config.accel_forward_axis.value
                invert = self.hw_config.accel_forward_invert
                # Using generator expression for sum
                total = sum(getattr(s.accel_raw, fwd_axis) for s in res.samples)
                avg_accel_fwd = - (total / len(res.samples)) if invert else (total / len(res.samples))

            print(f"  Delta Lean: {delta_lean:.1f}, Avg Fwd Accel: {avg_accel_fwd:.2f}")

            if delta_lean > 2.0 or avg_accel_fwd > 0.15:
                 print("  [SUCCESS] Direction Correct.")
                 self._update_learning_state(motor_direction_verified=True)
                 return True
            elif delta_lean < -2.0 or avg_accel_fwd < -0.15:
                 print("  [FAILURE] Direction Inverted. Fixing...")
                 self._update_hw_config(motor_l_invert=not self.hw_config.motor_l_invert,
                                        motor_r_invert=not self.hw_config.motor_r_invert)
                 return False

            print("  [WARNING] Inconclusive.")
            return False

        verify_with_retries("Motor Direction", test, verify)

    # --- Phase 5: Absolute Identity (Left/Right via Right-Hand Rule) ---
    def deduce_left_right_autonomous(self):
        """
        Identify Left vs Right Motor using Gyroscope Physics (Right-Hand Rule).
        Also deduces Gyro Yaw Polarity.
        Replaces ask_human_left_right.
        """
        print(">>> Autonomous Left/Right Verification & Yaw Calibration <<<")
        print("I am going to spin to determine my physical identity.")
        self.hw.wait_for_stability()

        def test(attempt: int):
            # 1. Establish Up Vector (Opposite of Gravity)
            print("  Measuring Gravity for Reference...")
            accel, _ = self.hw.read_imu_raw()
            up_vector = -accel  # Gravity points DOWN. Up Vector = -Gravity.

            # 2. Arc Command ("The Arc")
            # Drive Ch0 High, Ch1 Low (but same sign).
            power_high = self.learning_state.min_power_visible + 20 + (attempt * 10)
            power_low = power_high * 0.5
            print(f"  Driving Arc (Ch0={power_high:.0f}, Ch1={power_low:.0f})...")

            result = self.hw.execute_maneuver([(power_high, power_low, 1.5)])
            raw_gyro_samples = [s.gyro_raw for s in result.samples if s.gyro_raw]

            if not raw_gyro_samples:
                print("  [WARNING] No gyro samples collected.")
                return None

            # Average Gyro Vector
            gyro_sum = glm.vec3(0.0)
            for g in raw_gyro_samples:
                gyro_sum += g
            avg_gyro = gyro_sum / len(raw_gyro_samples)

            # 3. Calculate Dot Product: Up . Omega
            dot_prod = glm.dot(up_vector, avg_gyro)
            print(f"  Dot Product (Up . Gyro): {dot_prod:.2f}")

            if abs(dot_prod) < 10.0:
                 print("  [WARNING] Spin rate too low to determine direction.")
                 return None

            return (dot_prod, avg_gyro)

        def verify(data):
            if data is None: return False
            dot_prod, avg_gyro = data

            # 4. Deduce Motor Mapping
            if dot_prod > 0:
                print("  -> Detected Counter-Clockwise (Left) Turn. (Ch0 is Right).")
                if self.hw_config.motor_l == 0:
                    print("  -> ACTION: Swapping Channels to Correct Mapping.")
                    self._update_hw_config(
                        motor_l=self.hw_config.motor_r,
                        motor_r=self.hw_config.motor_l,
                        motor_l_invert=self.hw_config.motor_r_invert,
                        motor_r_invert=self.hw_config.motor_l_invert
                    )
                else:
                    print("  -> Mapping is already correct.")
            else:
                print("  -> Detected Clockwise (Right) Turn. (Ch0 is Left).")
                if self.hw_config.motor_l == 1:
                     print("  -> ACTION: Swapping Channels to Correct Mapping.")
                     self._update_hw_config(
                        motor_l=self.hw_config.motor_r,
                        motor_r=self.hw_config.motor_l,
                        motor_l_invert=self.hw_config.motor_r_invert,
                        motor_r_invert=self.hw_config.motor_l_invert
                     )
                else:
                    print("  -> Mapping is already correct.")

            # 5. Deduce Gyro Yaw Polarity
            # Re-read config flags (since they might have changed if we wanted to support dynamic axis remapping, but here we just check polarity)
            yaw_axis = self.hw_config.gyro_yaw_axis
            yaw_invert = self.hw_config.gyro_yaw_invert

            raw_yaw_component = getattr(avg_gyro, yaw_axis.value)
            current_yaw_rate = -raw_yaw_component if yaw_invert else raw_yaw_component

            print(f"  Current Config Yaw Rate: {current_yaw_rate:.1f}")

            # Check agreement: CCW (>0) should imply Pos Yaw. CW (<0) should imply Neg Yaw.
            if (dot_prod > 0 and current_yaw_rate < 0) or \
               (dot_prod < 0 and current_yaw_rate > 0):
                print("  -> Gyro Yaw Polarity is Inverted relative to reality.")
                print("  -> ACTION: Inverting Gyro Yaw.")
                self._update_hw_config(gyro_yaw_invert=not self.hw_config.gyro_yaw_invert)
            else:
                print("  -> Gyro Yaw Polarity is Correct.")

            self._update_learning_state(motor_channels_verified=True)
            return True

        verify_with_retries("Left/Right Identity", test, verify, fail_fatal=True)

    # --- Phase 6: The Stride (Motor Trimming) ---
    def calibrate_motor_trim(self):
        """
        Calibrate Motor Trim to ensure straight driving.
        Adaptive logic handles mismatched motors (up to 30%).
        """
        print(">>> Motor Trim Calibration <<<")
        print("I will drive straight and measure drift.")
        self.hw.wait_for_stability()

        def test(attempt: int):
            current_trim = self.learning_state.motor_trim
            print(f"  [Info] Testing Trim: {current_trim:.3f}")
            power = self.learning_state.min_power_visible + 15 + (attempt * 2)
            # Drive Straight
            return self.hw.drive_and_measure(power, power, 1.0)

        def verify(result):
            if not result.samples: return False # Retry

            avg_yaw = result.avg_yaw_rate
            print(f"    Avg Yaw Drift: {avg_yaw:.2f} deg/s")

            if abs(avg_yaw) < 2.0:
                print("  [SUCCESS] Drift is negligible.")
                return True

            # Adaptive Gain
            base_gain = 0.005
            gain = 0.015 if abs(avg_yaw) > 10.0 else base_gain
            correction = avg_yaw * gain

            # Apply Correction
            current_trim = self.learning_state.motor_trim
            new_trim = max(-0.4, min(0.4, current_trim + correction))

            if abs(new_trim - current_trim) < 0.001 and abs(avg_yaw) > 5.0:
                 print("  [WARNING] Trim saturated or not moving.")

            self._update_learning_state(motor_trim=new_trim)
            print(f"    -> Correction: {correction:+.4f} -> New Trim: {self.learning_state.motor_trim:.3f}")
            return False # Force Retry

        if not verify_with_retries("Motor Trim", test, verify, max_attempts=15, fail_fatal=False):
             print("  [WARNING] Could not perfectly trim motors. Saving best effort.")

    # --- Phase 6b: Mechanical Backlash ---
    def measure_backlash(self):
        """
        Measure gear slop by pushing gears forward, then timing how long
        reverse power takes to actually move the chassis.
        """
        print(">>> Measuring Mechanical Backlash (Gear Slop) <<<")
        self.hw.wait_for_stability(duration=1.0)

        test_power = self.learning_state.min_power_visible + 10

        # Step 1: Engage gears FORWARD
        print("  Engaging gears forward...")
        self.hw.set_motors(test_power, test_power)
        time.sleep(0.3)
        self.hw.stop()
        self.hw.wait_for_stability(duration=1.0)

        # Step 2: Slam REVERSE and time the dead zone
        print("  Reversing and timing IMU response...")
        start_time = time.time()
        self.hw.set_motors(-test_power, -test_power)

        slop_time = 0.0
        while True:
            reading = self.hw.read_imu_converted()

            # If the chassis pitch rate spikes, the wheels have finally caught
            if abs(reading.pitch_rate) > 5.0:
                slop_time = time.time() - start_time
                break

            # Timeout safety
            if time.time() - start_time > 1.0:
                print("  [WARNING] Could not detect movement. Assuming max slop.")
                slop_time = 0.2
                break

            time.sleep(0.005)

        self.hw.stop()

        # Subtract ~0.02s to account for basic chassis inertia (so we don't overcompensate)
        compensated_slop = max(0.0, slop_time - 0.02)

        print(f"  [SUCCESS] Backlash crossing takes ~{compensated_slop:.3f} seconds.")

        # Save it
        self.learning_state.control.backlash_pulse_time = compensated_slop
        self.learning_state.save()
        self._update_learning_state(backlash_verified=True)

    # --- Phase 7: Kick-Up Dynamics ---
    def _force_posture(self, target_posture: str, max_attempts: int = 5):
        """
        Autonomously forces the robot onto the target bumper ('FRONT' or 'BACK').
        Raises an exception (dies) if it gets stuck, allowing the watchdog to function.
        """
        base_power = self.learning_state.min_power_visible + 10

        for attempt in range(max_attempts):
            self.hw.wait_for_stability(duration=1.0)
            curr = self.hw.read_imu_converted()
            pitch = curr.pitch_angle

            # Are we already where we want to be?
            if target_posture == "FRONT" and pitch > 10.0:
                return  # Success
            if target_posture == "BACK" and pitch < -10.0:
                return  # Success

            print(f"  [AUTO-CORRECT] Wrong posture (Pitch={pitch:.1f}). Attempting to flop to {target_posture}...")

            # To go FRONT (Pos Pitch), we need Negative Power (Pitch Increase).
            # To go BACK (Neg Pitch), we need Positive Power (Pitch Decrease).
            power = base_power + (attempt * 10)

            if target_posture == "FRONT":
                # Drive Reverse to flop Forward
                self.hw.drive_and_measure(-power, -power, 0.4)
            else:
                # Drive Forward to flop Backward
                self.hw.drive_and_measure(power, power, 0.4)

        # If we exit the loop, we failed to achieve the posture.
        print(f"  [FATAL] Failed to reach {target_posture} posture after {max_attempts} attempts. Am I stuck?")
        import sys
        sys.exit(1) # Die. Watchdog is happy, evolution continues.

    def _attempt_dynamic_flop(self, start_from: str, power: float) -> str:
        """
        Perform a Dynamic "Roll & Slam" maneuver.
        start_from: "BACK" or "FRONT".
        Returns: "SUCCESS", "FAIL_POWER", or "FAIL_ALIGNMENT".
        """
        # Define Physics Directions
        # Convention: Positive Power = Forward.
        # To Stand Up from BACK (Pitch < 0): Need Backward Wheels (Neg Power).
        # To Stand Up from FRONT (Pitch > 0): Need Forward Wheels (Pos Power).
        if start_from == "BACK":
            # Leaning Back -> Kick Up (Increase Pitch).
            kick_sign = -1.0
            setup_sign = 1.0
            check_success = lambda p: p > 10
        elif start_from == "FRONT":
            # Leaning Front -> Kick Up (Decrease Pitch).
            kick_sign = 1.0
            setup_sign = -1.0
            check_success = lambda p: p < -10
        else:
            raise ValueError(f"Unknown direction {start_from}")

        # Execute Maneuver (No delays between Setup and Kick)
        setup_p = power * setup_sign * 0.7  # Gentle Roll
        kick_p = power * kick_sign * 1.0    # Hard Kick

        print(f"  [Action] Roll {setup_p:.1f} (0.3s) -> Slam {kick_p:.1f} (0.4s)...")

        result = self.hw.execute_maneuver([
            (setup_p, setup_p, 0.3),
            (kick_p, kick_p, 0.4)
        ])
        samples = result.samples

        # Coast & Check
        time.sleep(1.0)
        final_reading = self.hw.read_imu_converted()
        final_pitch = final_reading.pitch_angle

        # Alignment Check
        if not samples:
            avg_yaw = 0.0
        else:
            # Use signed sum to detect net drift, not vibration
            avg_yaw = sum(s.yaw_rate for s in samples) / len(samples)

        print(f"    Result: Pitch={final_pitch:.1f}, AvgDrift={avg_yaw:.1f} d/s")

        if abs(avg_yaw) > 15.0:
            return "FAIL_ALIGNMENT"

        if check_success(final_pitch):
            return "SUCCESS"
        else:
            return "FAIL_POWER"

    def _run_kickup_test(self, direction: str, config_attr: str):
        """Run kick-up test for a single direction."""
        print(f"\n[Test] Kick-Up from {direction}")

        self._kickup_alignment_retries = 0

        def action(p):
            self._force_posture(direction)
            return self._attempt_dynamic_flop(direction, p)

        def check(res):
            return res == "SUCCESS"

        def fail(res):
            if res == "FAIL_ALIGNMENT":
                print("  [DETECTED DRIFT] Re-aligning...")
                if self._kickup_alignment_retries < 3:
                    self._kickup_alignment_retries += 1
                    self.calibrate_motor_trim()
                    self.learning_state.save()
                    return True # Retry same power
                print("  [WARNING] Ignoring drift.")
            return False # Increment power

        start_p = max(20, self.learning_state.min_power_visible + 10)
        heartbeat_fn = self.watchdog.heartbeat if self.watchdog else None
        found = find_threshold(f"KickUp {direction}", start_p, 5, 100, action, check, fail, heartbeat_fn=heartbeat_fn)

        if found:
            setattr(self.learning_state.control, config_attr, found)
            self.learning_state.save()

    def find_flop_thresholds(self):
        """
        Discover Kick-Up Power for both directions using Dynamic Momentum.
        Uses calibrated rest angles for robust start detection.
        """
        print(">>> Dynamic Kick-Up Calibration (Roll & Slam) <<<")

        self._run_kickup_test("BACK", "kickup_power_forward")
        self._run_kickup_test("FRONT", "kickup_power_backward")

    # --- Tier 6: Final Verification ---
    def verify_final_configuration(self):
        """
        Autonomous Verification of the learned configuration.
        Pessimistic check:
        1. Drive Straight: Verify Yaw Rate is low, Accel/Pitch reflects movement.
        2. Turn Right: Verify Yaw Rate is Negative (or matches convention).
        """
        print("  Running Autonomous Verification...")

        # 1. Verify Straight Drive
        print("  [Check 1] Drive Straight...")
        power = self.learning_state.min_power_visible + 10

        result = self.hw.drive_and_measure(power, power, 1.0)
        time.sleep(0.5)

        avg_yaw = result.abs_avg_yaw_rate

        print(f"    Avg Yaw Rate: {avg_yaw:.1f} deg/s")

        if avg_yaw > 40.0:
             print("  [FAILURE] Robot spun while trying to drive straight.")
             print("  -> Possible Phase mismatch or Motor Direction mismatch despite checks.")
             self._update_learning_state(
                motor_direction_verified=False,
                motor_channels_verified=False,
                motor_trim_verified=False
             )
             sys.exit(1)

        # 2. Verify Right Turn
        print("  [Check 2] Turn Right (Clockwise)...")
        # Command L+, R- (Spin) -> Changed to Arc (L+, R_low) to avoid drag

        power_high = self.learning_state.min_power_visible + 15
        power_low = power_high * 0.5

        # Turn Right -> Drive Left Motor (High), Right Motor (Low)
        result = self.hw.drive_and_measure(power_high, power_low, 1.0)

        avg_yaw_signed = result.avg_yaw_rate

        print(f"    Avg Yaw Rate (Signed): {avg_yaw_signed:.1f} deg/s")

        # Expect Negative Yaw (Clockwise)
        if avg_yaw_signed > -10.0:
             # It should be significantly negative (e.g. -30)
             if avg_yaw_signed > 10.0:
                 print("  [FAILURE] Robot turned LEFT when commanded RIGHT.")
                 print("  -> Gyro Yaw or Motor Channel mismatch.")
                 self._update_learning_state(
                    motor_direction_verified=False,
                    motor_channels_verified=False,
                    motor_trim_verified=False
                 )
                 sys.exit(1)
             else:
                 print("  [FAILURE] Robot did not turn significantly.")
                 self._update_learning_state(
                    motor_direction_verified=False,
                    motor_channels_verified=False,
                    motor_trim_verified=False
                 )
                 sys.exit(1)

        print("  [PASS] Configuration Verified.")

if __name__ == "__main__":
    try:
        WiringCheck().run()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"\nCRITICAL ERROR: {e}")
        import traceback
        traceback.print_exc()
