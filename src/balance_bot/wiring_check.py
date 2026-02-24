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
from .dag_executor import DAGExecutor
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
                "depends_on": ["friction_threshold"],
                "action": self.align_motors_phase
            },

            "motor_direction": {
                "description": "Ensure positive PWM results in 'Forward' movement (Yank Test).",
                "condition": lambda: self.learning_state.motor_direction_verified,
                "depends_on": ["motor_phasing"],
                "action": self.determine_motor_direction
            },

            "spatial_orientation": {
                "description": "Toddler flail to find resting gravity vectors (The Labeler).",
                "condition": lambda: self.hw_config.accel_vertical_axis is not None,
                "depends_on": ["motor_direction"],
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

    def calibrate_static_orientation(self):
        """
        The Labeler.
        Discovers the Vertical Axis and labels the resting positions.
        """
        print(">>> Calibrating Orientation (The Labeler) <<<")

        # 1. Collect Resting Vectors
        vectors = self._toddler_flail_collection(duration=24.0)

        # 2. Sort Vectors
        try:
            p1, p2 = sort_resting_vectors(vectors)
        except ValueError as e:
            print(f"  [FAILURE] Sorting failed: {e}")
            raise

        print(f"  [DEBUG] Resting Position A: {p1}")
        print(f"  [DEBUG] Resting Position B: {p2}")

        # Failsafe Check
        spread = vector_angle(p1, p2)
        print(f"  [DEBUG] Angle Spread: {spread:.1f} degrees")
        if spread < 15.0:
            msg = f"Resting vectors are too close ({spread:.1f} deg). Is the robot flat on its side?"
            print(f"  [CRITICAL] {msg}")
            raise RuntimeError(msg)

        # 3. Find Vertical Axis
        # We know Pitch Axis (Gyro) and Forward Axis (Accel) from Yank Test.
        pitch_axis = self.hw_config.gyro_pitch_axis
        fwd_axis = self.hw_config.accel_forward_axis

        # We assume physical axes align (Accel X ~ Gyro X).
        # So we exclude the physical axes used for Pitch and Forward.
        used_axes = {pitch_axis, fwd_axis}
        all_axes = {Axis.X, Axis.Y, Axis.Z}
        remaining = list(all_axes - used_axes)

        if not remaining:
             print("  [CRITICAL] Axes Overlap! Pitch and Forward are same?")
             # Fallback: Find dominant gravity component on p1 ignoring others
             vert_axis_name, _, _ = analyze_dominance(p1, "Vertical Gravity")
             vert_axis = Axis(vert_axis_name)
        else:
             vert_axis = remaining[0]

        # Determine Inversion
        # We assume robot is upright-ish in resting state.
        # Vertical component should be Positive.
        raw_val = getattr(p1, vert_axis.value)
        vert_invert = raw_val < 0 # If Negative, Invert to make Positive.

        print(f"  -> Discovered Vertical Axis: {vert_axis.value.upper()} (Invert: {vert_invert})")

        self._update_hw_config(
            accel_vertical_axis=vert_axis,
            accel_vertical_invert=vert_invert,
            # Also set Gyro Yaw (usually same as Vertical)
            gyro_yaw_axis=vert_axis,
            # Default Yaw Invert? We check this later in Left/Right.
            gyro_yaw_invert=False,
            # Set Roll Axis (Remaining)
            gyro_roll_axis=fwd_axis # Typically Roll is around Forward axis
        )

        # 4. Calculate and Assign Rest Angles
        print("  Labeling Resting Positions...")

        def get_pitch(vec):
            fwd = self.hw.get_mapped_value(vec, "accel_forward")
            vert = self.hw.get_mapped_value(vec, "accel_vertical")
            return calculate_pitch(fwd, vert)

        angle1 = get_pitch(p1)
        angle2 = get_pitch(p2)

        print(f"  Vector 1 Pitch: {angle1:.1f}")
        print(f"  Vector 2 Pitch: {angle2:.1f}")

        if angle1 > 0 and angle2 < 0:
            self._update_learning_state(rest_angle_forward=angle1, rest_angle_backward=angle2)
        elif angle2 > 0 and angle1 < 0:
            self._update_learning_state(rest_angle_forward=angle2, rest_angle_backward=angle1)
        else:
             print("  [WARNING] Both angles have same sign? Robot might not be crossing vertical.")
             # Fallback: Assign based on value
             if angle1 > angle2:
                 self._update_learning_state(rest_angle_forward=angle1, rest_angle_backward=angle2)
             else:
                 self._update_learning_state(rest_angle_forward=angle2, rest_angle_backward=angle1)


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

            # Ensure stability first
            self.hw.wait_for_stability()
            # Capture baseline
            baseline_imu = self.hw.read_imu_converted()

            # Drive (don't wait for stability again inside, to reduce lag)
            res = self._drive_and_wait(p, p, 0.5, wait_stable=False)

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
            # Calculate average Gyro Magnitude (Axis Agnostic)
            avg_gyro_mag = sum(glm.length(s.gyro_raw) for s in res.samples) / len(res.samples)
            print(f"    Avg Gyro Mag: {avg_gyro_mag:.1f} deg/s")

            if avg_gyro_mag > 30.0:
                print("  -> Spinning detected. Motors are fighting.")
                print("  -> ACTION: Inverting Right Motor logic.")
                self._update_hw_config(motor_r_invert=not self.hw_config.motor_r_invert)
                return False

            # Check 3 (Are we translating?):
            # We moved (Check 1) and we are NOT spinning (Check 2).
            # Look for Net Acceleration Change (Axis Agnostic).

            # Calculate Mean Accel Vector during move
            sum_accel = glm.vec3(0.0)
            count = 0
            for s in res.samples:
                if s.accel_raw:
                    sum_accel += s.accel_raw
                    count += 1
            avg_accel = sum_accel / count if count > 0 else glm.vec3(0.0)

            # Calculate Vector Delta Magnitude
            delta_vec = avg_accel - baseline.accel_raw
            delta_mag = glm.length(delta_vec)

            print(f"    Accel Delta Mag: {delta_mag:.3f}g")

            # Threshold for significant translation (0.05g is conservative but safe for toddlers)
            if delta_mag > 0.05:
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
        Establishes the Motor-First Axiom: +PWM is Forward.
        Discovers Pitch Axis and Forward Axis based on inertial reaction.
        """
        print(">>> Determining Motor Direction (Yank Test) <<<")
        self.hw.wait_for_stability()

        # 1. Measure Baseline
        baseline_accel, _ = self.hw.read_imu_raw()

        # 2. Pulse Motors Forward
        p = self.learning_state.min_power_visible + 30
        print(f"  Pulsing Motors Forward (+{p})...")
        # Drive for 0.3s
        res = self._drive_and_wait(p, p, 0.3, wait_stable=False)

        if not res.samples:
            print("  [FAILURE] No samples collected.")
            sys.exit(1)

        # 3. Analyze Gyro (Pitch Axis)
        # Find dominant axis of rotation during the burst
        gyro_sum = glm.vec3(0.0)
        for s in res.samples:
             if s.gyro_raw:
                 gyro_sum += s.gyro_raw
        avg_gyro = gyro_sum / len(res.samples)

        pitch_axis_name, _, _ = analyze_dominance(avg_gyro, "Inertial Pitch (Gyro)")

        # We assume driving forward causes inertial BACKWARD pitch (Nose Up).
        # We want this to be NEGATIVE Pitch Rate.
        # So if raw value is POSITIVE, we must INVERT.
        raw_pitch_val = getattr(avg_gyro, pitch_axis_name)
        pitch_invert = raw_pitch_val > 0

        print(f"  -> Discovered Pitch Axis: {pitch_axis_name.upper()} (Invert: {pitch_invert})")
        self._update_hw_config(
            gyro_pitch_axis=Axis(pitch_axis_name),
            gyro_pitch_invert=pitch_invert
        )

        # 4. Analyze Accel (Forward Axis)
        # Find dominant axis of acceleration delta.
        # Calculate Delta
        accel_sum = glm.vec3(0.0)
        count = 0
        for s in res.samples:
            if s.accel_raw:
                accel_sum += s.accel_raw
                count += 1
        avg_accel = accel_sum / count if count > 0 else glm.vec3(0.0)

        delta_accel = avg_accel - baseline_accel

        fwd_axis_name, _, _ = analyze_dominance(delta_accel, "Inertial Lag (Accel)")

        # Prompt: "set accel_forward_invert so this delta reads as *negative*."
        raw_fwd_val = getattr(delta_accel, fwd_axis_name)
        fwd_invert = raw_fwd_val > 0 # If Positive, Invert to make it Negative.

        print(f"  -> Discovered Forward Axis: {fwd_axis_name.upper()} (Invert: {fwd_invert})")
        self._update_hw_config(
            accel_forward_axis=Axis(fwd_axis_name),
            accel_forward_invert=fwd_invert
        )

        self._update_learning_state(motor_direction_verified=True)

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
