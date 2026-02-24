import time
import sys
import random
import glm
from typing import Optional

from .configuration import HardwareConfig, LearningState
from .hardware.robot_hardware import RobotHardware
from .watchdog import SurvivalWatchdog
from .enums import Axis
from .dag_executor import DAGExecutor
from .utils import (
    analyze_dominance,
    run_diagnostics,
    scan_i2c_or_die,
    verify_with_retries,
    find_threshold,
    sort_resting_vectors,
    vector_angle,
    clamp,
    calculate_pitch
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

        # Cascading Invalidation: Force hardware re-initialization
        if self.hw:
            self.hw.cleanup()
            self.hw = None
        # We return immediately. The DAG will detect 'hardware_init' condition is missing (hw is None)
        # and re-run init_hw in the next step.

    def run(self):
        """
        Main Knowledge Dependency Loop.
        Uses a Directed Acyclic Graph (DAG) to resolve configuration dependencies.
        """
        print("Beginning Self-Discovery Protocol...")
        run_diagnostics()

        knowledge_graph = {
            "i2c_buses": {
                "description": "Locate the physical addresses of the motor controller and IMU.",
                "condition": lambda: self.hw_config.motor_i2c_bus is not None and self.hw_config.imu_i2c_bus is not None,
                "depends_on": [],
                "action": self.discover_buses
            },

            "hardware_init": {
                "description": "Instantiate the hardware abstraction layer using discovered buses.",
                "condition": lambda: self.hw is not None,
                "depends_on": ["i2c_buses"],
                "action": self.init_hw
            },

            "motor_candidates": {
                "description": "Assign default logical channels to the left and right motors.",
                "condition": lambda: self.hw_config.motor_l is not None and self.hw_config.motor_r is not None,
                "depends_on": ["hardware_init"],
                "action": self._set_default_motors
            },

            "friction_threshold": {
                "description": "Find the minimum PWM required to overcome internal gear friction.",
                "condition": lambda: self.learning_state.min_power_visible > 0,
                "depends_on": ["hardware_init", "motor_candidates"],
                "action": self.find_min_power
            },

            "motor_phasing": {
                "description": "Ensure both wheels drive the same direction for a given sign.",
                "condition": lambda: self.learning_state.motor_phasing_verified,
                "depends_on": ["friction_threshold"], # Motor-First Axiom: No longer needs spatial_orientation
                "action": self.align_motors_phase
            },

            "motor_direction": {
                "description": "Ensure positive PWM results in 'Forward' movement (reducing pitch when leaning forward).",
                "condition": lambda: self.learning_state.motor_direction_verified,
                "depends_on": ["motor_phasing"],
                "action": self.determine_motor_direction
            },

            "spatial_orientation": {
                "description": "Toddler flail to find resting gravity vectors, labeling resting positions based on known motors.",
                "condition": lambda: self.hw_config.accel_vertical_axis is not None,
                "depends_on": ["motor_direction"], # Must know motors to label Forward/Backward
                "action": self.calibrate_static_orientation
            },

            "left_right_identity": {
                "description": "Drive an arc and use the Right-Hand Rule to determine Left vs Right.",
                "condition": lambda: self.learning_state.motor_channels_verified,
                "depends_on": ["motor_direction", "spatial_orientation"],
                "action": self.deduce_left_right_autonomous
            },

            "motor_trim": {
                "description": "Drive straight and calculate the PWM offset needed to eliminate yaw drift.",
                "condition": lambda: self.learning_state.motor_trim_verified,
                "depends_on": ["left_right_identity"], # Must know true L/R to apply correct offsets
                "action": lambda: (self.calibrate_motor_trim(), self._update_learning_state(motor_trim_verified=True))
            },

            "mechanical_backlash": {
                "description": "Measure the dead-time delay when reversing gear direction.",
                "condition": lambda: self.learning_state.backlash_verified,
                "depends_on": ["motor_trim"], # Requires reliable straight-line driving
                "action": self.measure_backlash
            },

            "kickup_dynamics": {
                "description": "Determine the impulse power needed to stand up from resting positions.",
                "condition": lambda: self.learning_state.control.kickup_power_forward > 0.0,
                "depends_on": ["motor_trim", "spatial_orientation"],
                "action": self.find_flop_thresholds
            }
        }

        executor = DAGExecutor(knowledge_graph)

        # Run until success
        if executor.run(max_iterations=100):
            print("-> [VERIFYING] Final Configuration Check.")
            self.verify_final_configuration()
            self._print_summary()

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
            vec = self.hw.measure_gravity()
            collected.append(vec)
            # print(f"    Collected: {vec}")

        return collected

    def calibrate_static_orientation(self):
        """
        Map Physical Axes (X, Y, Z) to Logical Axes (Vertical) using known Forward/Pitch axes.
        """
        print(">>> Calibrating Orientation (The Labeler) <<<")

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

        # 4. Determine Vertical Axis
        # Look at the raw gravity vector of p1. Excluding the now-known accel_forward_axis and gyro_pitch_axis.
        known_axes = {self.hw_config.accel_forward_axis.value, self.hw_config.gyro_pitch_axis.value} # gyro_pitch uses same raw axes names usually? Yes X,Y,Z.

        candidates = {k: abs(getattr(p1, k)) for k in ['x', 'y', 'z'] if k not in known_axes}

        if len(candidates) == 1:
            vert_axis_name = list(candidates.keys())[0]
            print(f"  [Analysis] Only one axis remaining for Vertical: {vert_axis_name.upper()}")
        else:
            vert_axis_name, _, _ = analyze_dominance(candidates, "Vertical Axis (Gravity)")

        update_dict = {}
        update_dict['accel_vertical_axis'] = Axis(vert_axis_name)

        # Invert if gravity is pointing 'up' relative to the sensor?
        # Standard: Gravity points DOWN (-Z).
        # If Accel measures +1g on Axis K, then Axis K points UP (against gravity).
        # We want mapped Vertical to be Positive UP? Or Positive DOWN?
        # Usually Robot code assumes Vertical is UP (Positive Z).
        # If raw is +9.8 (Sensor points UP), mapped should be +9.8. So Invert = False.
        # If raw is -9.8 (Sensor points DOWN), mapped should be +9.8? No, usually we want World Frame.
        # Let's stick to the convention: Mapped Value should represent standard physics frame if possible.
        # But wait, logic in old code: `update_dict['accel_vertical_invert'] = vert_val < 0`.
        # If vert_val is -9.8 (pointing down), invert is True -> +9.8.
        # So it seems the goal is to make the resting vertical vector Positive.
        vert_val = getattr(p1, vert_axis_name)
        update_dict['accel_vertical_invert'] = vert_val < 0
        print(f"  -> Vertical Axis: {vert_axis_name.upper()} (Invert: {update_dict['accel_vertical_invert']})")

        # Also deduce Gyro Yaw/Roll (Orthogonal to others)
        # Yaw is usually around Vertical Axis.
        # Roll is usually around Forward Axis.
        update_dict['gyro_yaw_axis'] = Axis(vert_axis_name)
        update_dict['gyro_roll_axis'] = self.hw_config.accel_forward_axis # Axis(forward_axis_name)

        # Need to save config NOW so get_mapped_value uses it
        self._update_hw_config(**update_dict)

        # 5. Calculate and Assign Resting Angles
        print("  Measuring Rest Angles...")

        # Map p1 and p2 using newly discovered Forward and Vertical axes.
        # Note: RobotHardware uses self.hw_config, which we just updated.

        def calc_pitch_for_vec(vec):
            fwd = self.hw.get_mapped_value(vec, "accel_forward")
            vert = self.hw.get_mapped_value(vec, "accel_vertical")
            return calculate_pitch(fwd, vert)

        angle_p1 = calc_pitch_for_vec(p1)
        angle_p2 = calc_pitch_for_vec(p2)

        print(f"  [DEBUG] Angle P1: {angle_p1:.1f}, Angle P2: {angle_p2:.1f}")

        # The vector resulting in a Positive pitch must be assigned to rest_angle_forward.
        # The vector resulting in a Negative pitch must be assigned to rest_angle_backward.

        if angle_p1 > 0 and angle_p2 < 0:
            self._update_learning_state(rest_angle_forward=angle_p1, rest_angle_backward=angle_p2)
        elif angle_p2 > 0 and angle_p1 < 0:
            self._update_learning_state(rest_angle_forward=angle_p2, rest_angle_backward=angle_p1)
        else:
            print("  [WARNING] Could not distinguish Front/Back based on pitch sign (both same sign?). using magnitude separation?")
            # Fallback: Forward is usually larger positive number? Or maybe we are upside down?
            # If we are strictly following the prompt:
            # "The vector resulting in a Positive pitch must be assigned to rest_angle_forward."
            # If both are positive, we are in trouble.
            if angle_p1 > angle_p2:
                 self._update_learning_state(rest_angle_forward=angle_p1, rest_angle_backward=angle_p2)
            else:
                 self._update_learning_state(rest_angle_forward=angle_p2, rest_angle_backward=angle_p1)

        print(f"  -> Calibrated Rest Angles: Back={self.learning_state.rest_angle_backward:.1f}, Front={self.learning_state.rest_angle_forward:.1f}")


    # --- Phase 3a: Friction Threshold ---
    def find_min_power(self):
        """
        Find minimum PWM to overcome friction.
        Uses raw gyro magnitude to detect movement without axis calibration.
        """
        print(">>> Finding Minimum Power (Raw) <<<")
        print("Ensuring robot is on the floor...")

        def action(p):
            res = self.hw.drive_and_measure(p, p, 0.3, wait_for_stability=False)
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

            # Ensure stability first
            self.hw.wait_for_stability()
            # Capture baseline
            baseline_imu = self.hw.read_imu_converted()

            # Drive (don't wait for stability again inside, to reduce lag)
            res = self.hw.drive_and_measure(p, p, 0.5, wait_for_stability=False)
            time.sleep(0.5)

            return (baseline_imu, res)

        def verify(data):
            if not data: return False
            baseline, res = data

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
            baseline_fwd_accel = self.hw.get_mapped_value(baseline.accel_raw, "accel_forward")
            fwd_accels = [self.hw.get_mapped_value(s.accel_raw, "accel_forward") for s in res.samples]
            avg_fwd_accel = sum(fwd_accels) / len(res.samples)

            delta_fwd_accel = avg_fwd_accel - baseline_fwd_accel

            delta_pitch = abs(res.samples[-1].pitch_angle - res.samples[0].pitch_angle)

            print(f"    Avg Fwd Accel: {avg_fwd_accel:.2f}g (Baseline: {baseline_fwd_accel:.2f}g, Delta: {delta_fwd_accel:.2f}g)")
            print(f"    Delta Pitch: {delta_pitch:.1f} deg")

            if delta_fwd_accel > 0.1 or delta_pitch > 2.0:
                 print("  -> Translation detected. Motors are aligned (Straight).")
                 self._update_learning_state(motor_phasing_verified=True)
                 return True

            print("  [WARNING] Movement detected but not clearly translation. Retrying...")
            return False

        verify_with_retries("Motor Phasing", test, verify)

    # --- Phase 4: Sense of Forward (Pitch Axis & Polarity) ---
    def determine_motor_direction(self):
        """
        The "Yank" Test.
        We establish axes via motor movement (Motor-First Axiom).
        +PWM is decreed to be Absolute Forward.
        We measure the inertial reaction to map IMU axes.
        """
        print(">>> Determining Motor Direction (The Yank Test) <<<")

        # 1. Wait for stability and read baseline
        print("  Stabilizing...")
        self.hw.wait_for_stability()
        baseline_accel, _ = self.hw.read_imu_raw()

        # 2. Pulse Motors Forward
        power = self.learning_state.min_power_visible + 30
        print(f"  Yanking Motors Forward (+{power})...")

        res = self.hw.drive_and_measure(power, power, 0.3, wait_for_stability=False)
        time.sleep(0.5)

        if not res.samples:
            print("  [FAILURE] No samples collected during yank.")
            return # Will fail verification check if handled, but here we might need to retry?
            # Ideally verify_with_retries would be better, but prompt asked for this specific sequence.
            # Assuming it works or we crash/retry next run.

        # 3. Find Pitch Axis (Dominant Rotation)
        # "Because driving the base forward causes the top to inertially pitch backward,
        # save this axis ... and set gyro_pitch_invert so this ... reads as a NEGATIVE pitch rate."

        # Calculate Average Gyro Vector during burst
        gyro_sum = glm.vec3(0.0)
        for s in res.samples:
            if s.gyro_raw:
                gyro_sum += s.gyro_raw
        avg_gyro = gyro_sum / len(res.samples)

        print(f"  [Analysis] Avg Gyro during yank: {avg_gyro}")

        pitch_axis_name, _, _ = analyze_dominance(avg_gyro, "Pitch Axis (Yank)")

        update_dict = {}
        update_dict['gyro_pitch_axis'] = Axis(pitch_axis_name)

        # Invert logic: We want the detected rotation to be NEGATIVE.
        # If detected is +50, we need Invert=True (to make it -50).
        # If detected is -50, we need Invert=False (to keep it -50).
        detected_val = getattr(avg_gyro, pitch_axis_name)
        update_dict['gyro_pitch_invert'] = detected_val > 0

        print(f"  -> Pitch Axis: {pitch_axis_name.upper()} (Invert: {update_dict['gyro_pitch_invert']})")

        # 4. Find Forward Axis (Dominant Accel Delta)
        # "Calculate the delta (moving_accel - baseline_accel). Exclude the pitch axis."

        # Calculate Average Accel during burst
        accel_sum = glm.vec3(0.0)
        for s in res.samples:
            if s.accel_raw:
                accel_sum += s.accel_raw
        avg_accel = accel_sum / len(res.samples)

        delta_accel = avg_accel - baseline_accel
        print(f"  [Analysis] Accel Delta: {delta_accel}")

        # Exclude pitch axis
        candidates = {k: abs(getattr(delta_accel, k)) for k in ['x', 'y', 'z'] if k != pitch_axis_name}
        fwd_axis_name, _, _ = analyze_dominance(candidates, "Forward Axis (Yank Lag)")

        update_dict['accel_forward_axis'] = Axis(fwd_axis_name)

        # Invert logic: "set accel_forward_invert so this delta reads as NEGATIVE."
        delta_val = getattr(delta_accel, fwd_axis_name)
        update_dict['accel_forward_invert'] = delta_val > 0

        print(f"  -> Forward Axis: {fwd_axis_name.upper()} (Invert: {update_dict['accel_forward_invert']})")

        self._update_hw_config(**update_dict)
        self._update_learning_state(motor_direction_verified=True)
        print("  [SUCCESS] Motor Direction & Axes Mapped.")


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

    def _get_dir_sign(self, target_posture: str) -> float:
        """
        Returns the motor direction sign required to move TOWARDS the target posture.
        BACK (Negative Pitch) -> Drive Forward (+1.0) to decrease pitch.
        FRONT (Positive Pitch) -> Drive Reverse (-1.0) to increase pitch.
        """
        return 1.0 if target_posture == "BACK" else -1.0

    def _force_posture(self, target_posture: str, max_attempts: int = 5):
        """
        Autonomously forces the robot onto the target bumper ('FRONT' or 'BACK').
        Raises an exception (dies) if it gets stuck, allowing the watchdog to function.
        """
        base_power = self.learning_state.min_power_visible + 10
        dir_sign = self._get_dir_sign(target_posture)

        for attempt in range(max_attempts):
            self.hw.wait_for_stability(duration=1.0)
            pitch = self.hw.read_imu_converted().pitch_angle

            # Check if we are already there (opposite logic of direction sign)
            # If target=BACK (sign=1.0), we want pitch < -10.
            # If target=FRONT (sign=-1.0), we want pitch > 10.
            # Simplified: pitch * sign < -10 means we reached the target "zone"
            # Example: pitch=-15, sign=1.0 -> -15 < -10 (True) -> Success
            # Example: pitch=15, sign=-1.0 -> -15 < -10 (True) -> Success
            if pitch * dir_sign < -10.0:
                return  # Success

            print(f"  [AUTO-CORRECT] Wrong posture (Pitch={pitch:.1f}). Attempting to flop to {target_posture}...")

            power = (base_power + (attempt * 10)) * dir_sign
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
        # Determine target posture (opposite of start)
        target_posture = "FRONT" if start_from == "BACK" else "BACK"

        # Logic:
        # Kick: Move FROM start TO target. Use target direction.
        # Setup: Roll deeper INTO start. Use start direction.
        kick_sign = self._get_dir_sign(target_posture)
        setup_sign = self._get_dir_sign(start_from)

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
        final_pitch = self.hw.read_imu_converted().pitch_angle

        # Alignment Check
        avg_yaw = sum(s.yaw_rate for s in samples) / len(samples) if samples else 0.0
        print(f"    Result: Pitch={final_pitch:.1f}, AvgDrift={avg_yaw:.1f} d/s")

        if abs(avg_yaw) > 15.0:
            return "FAIL_ALIGNMENT"

        # Success Check: Did we reach the target zone?
        # Target Zone: pitch * target_sign < -10
        if final_pitch * kick_sign < -10.0:
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
        1. Drive Straight: Verify Yaw Rate is low.
        2. Turn Right: Verify Yaw Rate is Negative.
        """
        print("  Running Autonomous Verification...")
        base_p = self.learning_state.min_power_visible

        checks = [
            {
                "name": "Straight Drive",
                "left": base_p + 10, "right": base_p + 10,
                "check": lambda res: res.abs_avg_yaw_rate < 40.0,
                "fail_msg": "Robot spun while trying to drive straight."
            },
            {
                "name": "Right Turn (Clockwise)",
                "left": base_p + 15, "right": (base_p + 15) * 0.5,
                "check": lambda res: res.avg_yaw_rate < -10.0,
                "fail_msg": "Robot did not turn Right (Clockwise). Expected Yaw < -10."
            }
        ]

        for c in checks:
            print(f"  [Check] {c['name']}...")
            res = self.hw.drive_and_measure(c['left'], c['right'], 1.0)
            time.sleep(0.5)

            print(f"    Avg Yaw Rate: {res.avg_yaw_rate:.1f} deg/s (Abs: {res.abs_avg_yaw_rate:.1f})")

            if not c['check'](res):
                print(f"  [FAILURE] {c['fail_msg']}")
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
