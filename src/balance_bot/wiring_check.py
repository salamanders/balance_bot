import time
import sys
try:
    import smbus
except ImportError:
    smbus = None

from typing import Callable, Any
from .diagnostics import run_diagnostics
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware, IMUReading
from .enums import Axis
from .utils import analyze_dominance, cross_product, Vector3


class WiringCheck:
    """
    Zero-Knowledge "Self-Discovery" Wiring Check.
    Implements a state-machine that incrementally discovers robot configuration.
    See LEARN.md for specification.
    """

    def __init__(self):
        self.config = RobotConfig.load()
        self.hw = None

    def init_hw(self):
        """
        Initialize hardware with current known config.
        If config is incomplete, fills in safe defaults for discovery.
        """
        if self.hw:
            return

        # We need buses to be discovered first
        if self.config.motor_i2c_bus is None or self.config.imu_i2c_bus is None:
            # Cannot init HW without buses.
            return

        self.hw = RobotHardware(self.config)
        self.hw.init()

    def cleanup(self):
        if self.hw:
            self.hw.stop()
            self.hw.cleanup()

    def run(self):
        """
        Main Knowledge Dependency Loop.
        Iterates until all configuration requirements are met.
        """
        print("Beginning Self-Discovery Protocol...")

        # 0. Diagnostics (Always run first to check system health)
        run_diagnostics()

        while True:
            # Reload config from disk (or use current memory state)
            # We trust self.config tracks the state.
            c = self.config

            print("\n---------------------------------------------------")
            print("Checking Knowledge Base...")

            # --- Tier 1: Hardware Connectivity ---
            if c.motor_i2c_bus is None or c.imu_i2c_bus is None:
                print("-> [MISSING] I2C Bus Assignments.")
                self.discover_buses()
                self.config.save()
                continue

            # --- Tier 2: The Physical World (Sensors) ---
            # We need to know Up, Front, and the Pivot Axis before we can move.
            if c.accel_vertical_axis is None:
                print("-> [MISSING] Spatial Orientation (Vertical/Forward/Pitch).")
                # We need HW initialized to read sensors
                self.init_hw()
                self.calibrate_static_orientation()
                self.config.save()
                continue

            # --- Tier 3: Action/Reaction (Motors) ---
            # Ensure HW is initialized for motor tests
            if self.hw is None:
                self.init_hw()

            # Check for unset motors before we try to move them
            if c.motor_l is None or c.motor_r is None:
                print("-> [INFO] Motor channels not set. Assuming default candidates (0, 1) for discovery.")
                c.motor_l = 0
                c.motor_r = 1
                self.init_hw()

            # 3a. Friction Threshold
            if c.min_power_visible == 0:
                print("-> [MISSING] Minimum Power Threshold (Friction).")
                self.find_min_power()
                self.config.save()
                continue

            # 3b. Phasing (Are motors spinning together or fighting?)
            if not c.motor_phasing_verified:
                print("-> [MISSING] Motor Phasing (Spin vs Drive).")
                self.align_motors_phase()
                self.config.save()
                continue

            # 3c. Direction (Does +Power make me Stand Up or Dig In?)
            if not c.motor_direction_verified:
                print("-> [MISSING] Motor Polarity (Stand Up Direction).")
                self.determine_motor_direction()
                self.config.save()
                continue

            # --- Tier 4: The Human Anchor (Autonomous) ---
            if not c.motor_channels_verified:
                print("-> [MISSING] Left/Right Identification.")
                self.deduce_left_right_autonomous()
                self.config.save()
                continue

            # --- Tier 4.5: The Stride (Trim) ---
            if not c.motor_trim_verified:
                print("-> [MISSING] Motor Trim (Straight Drive).")
                self.calibrate_motor_trim()
                self.config.motor_trim_verified = True
                self.config.save()
                continue

            # --- Tier 5: Dynamics (Optional/Advanced) ---
            if c.control.kickup_power_forward == 0.0:
                print("-> [MISSING] Kick-Up Dynamics.")
                self.find_flop_thresholds()
                self.config.save()
                continue

            # --- Tier 6: Final Verification ---
            print("-> [VERIFYING] Final Configuration Check.")
            self.verify_final_configuration()

            # STOP HERE. Do not add find_balance_point.
            # The Agent will handle the rest.

            print("\n[SUCCESS] Hardware Verified. Ready for Agent (Main Brain).")
            print("Summary:")
            print(f"  Buses: Motor={c.motor_i2c_bus}, IMU={c.imu_i2c_bus}")
            print(f"  Axes: Vert={c.accel_vertical_axis}, Fwd={c.accel_forward_axis}, Pitch={c.gyro_pitch_axis}, Yaw={c.gyro_yaw_axis}")
            print(f"  Motors: MinPower={c.min_power_visible}, L={c.motor_l}(Inv={c.motor_l_invert}), R={c.motor_r}(Inv={c.motor_r_invert})")
            print(f"  Trim: {c.motor_trim:.3f}")
            print(f"  KickUp: Fwd={c.control.kickup_power_forward:.1f}, Bwd={c.control.kickup_power_backward:.1f}")
            break

        self.cleanup()

    # --- Phase 1: Hardware Connectivity ---
    def _scan_candidates(self, name: str, check_fn: Callable[[Any], bool]) -> int | None:
        """
        Scans I2C buses for a device using a callback.
        Returns the bus ID if found, else None.
        """
        candidates = [1, 3, 0, 2]
        for bus_id in candidates:
            try:
                bus = smbus.SMBus(bus_id)
                try:
                    if check_fn(bus):
                        print(f"  [FOUND] {name} on Bus {bus_id}")
                        return bus_id
                except OSError:
                    pass
                finally:
                    try:
                        bus.close()
                    except Exception:
                        pass
            except Exception:
                pass
        return None

    def _scan_or_die(self, name: str, check_fn: Callable[[Any], bool]) -> int:
        bus = self._scan_candidates(name, check_fn)
        if bus is None:
            print(f"  [FAILURE] Could not find {name} on any bus.")
            sys.exit(1)
        return bus

    def discover_buses(self):
        """
        Scan I2C buses [1, 3, 0, 2] for PiconZero (0x22) and MPU6050 (0x68).
        """
        print("Scanning I2C Buses...")

        # 1. Find Motors (0x22)
        def check_motor(bus):
            bus.read_byte_data(0x22, 0)
            return True

        self.config.motor_i2c_bus = self._scan_or_die("PiconZero (Motors)", check_motor)

        # 2. Find IMU (0x68)
        def check_imu(bus):
            return bus.read_byte_data(0x68, 0x75) == 0x68

        self.config.imu_i2c_bus = self._scan_or_die("MPU6050 (IMU)", check_imu)

    # --- Phase 2: The Physical World (Sensors) ---
    def _measure_gravity_vectors(self) -> tuple[Vector3, Vector3]:
        """Measure gravity vector at Back and Front resting positions."""
        print(">>> Calibrating Orientation <<<")
        print("Please place the robot on the FLOOR.")
        print("Ensure motors are OFF and it is resting on the BACK training wheel.")
        self.hw.wait_for_stability(duration=1.5)

        # 1. Read Back Rest Gravity
        print("  Measuring Gravity (Back Position)...")
        time.sleep(0.5)
        samples = 100
        accel_sum_back = Vector3(0.0, 0.0, 0.0)

        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            accel_sum_back += a
            time.sleep(0.01)

        avg_back = accel_sum_back / samples
        print(f"  [DEBUG] Avg Back Vector: {avg_back}")

        # 2. Read Front Rest Gravity
        print("\n  Now TIP ROBOT FORWARD to Front Wheel.")
        input("Press Enter to measure FRONT position...")
        self.hw.wait_for_stability(duration=1.0)

        accel_sum_front = Vector3(0.0, 0.0, 0.0)
        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            accel_sum_front += a
            time.sleep(0.01)

        avg_front = accel_sum_front / samples
        print(f"  [DEBUG] Avg Front Vector: {avg_front}")
        return avg_back, avg_front

    def _deduce_axes(self, avg_back: Vector3, avg_front: Vector3):
        """Deduce Pitch, Vertical, and Forward axes from gravity vectors."""
        # 3. Determine Pitch Axis via Cross Product (Normal to the motion plane)
        pitch_vec = avg_back.cross(avg_front)

        pitch_axis_name, pitch_magnitude, _ = analyze_dominance(
            {k: abs(v) for k, v in pitch_vec.items()},
            "Pitch Axis (CrossProd)"
        )

        if pitch_magnitude < 1.1:
            print("  [WARNING] Pitch axis unclear. Did the robot actually move?")

        self.config.gyro_pitch_axis = Axis(pitch_axis_name)
        self.config.gyro_pitch_invert = pitch_vec[pitch_axis_name] < 0
        print(f"  -> Pitch Axis: {pitch_axis_name.upper()} (Invert: {self.config.gyro_pitch_invert})")

        # 4. Determine Vertical Axis
        candidates = {k: abs(v) for k, v in avg_back.items() if k != pitch_axis_name}
        vert_axis_name, _, _ = analyze_dominance(candidates, "Vertical Axis (Gravity)")

        self.config.accel_vertical_axis = Axis(vert_axis_name)
        # Invert if gravity component is negative
        self.config.accel_vertical_invert = avg_back[vert_axis_name] < 0
        print(f"  -> Vertical Axis: {vert_axis_name.upper()} (Invert: {self.config.accel_vertical_invert})")

        # 5. Determine Forward Axis
        all_axes = {'x', 'y', 'z'}
        used = {pitch_axis_name, vert_axis_name}
        remaining = list(all_axes - used)

        if not remaining:
            print("  [CRITICAL ERROR] Axis deduction failed. Overlapping axes.")
            sys.exit(1)

        forward_axis_name = remaining[0]
        self.config.accel_forward_axis = Axis(forward_axis_name)

        # Deduce Forward Inversion
        delta_fwd = avg_front[forward_axis_name] - avg_back[forward_axis_name]
        self.config.accel_forward_invert = delta_fwd < 0
        print(f"  -> Forward Axis: {forward_axis_name.upper()} (Invert: {self.config.accel_forward_invert})")

        # Deduce Gyro Yaw/Roll
        self.config.gyro_yaw_axis = Axis(vert_axis_name)
        self.config.gyro_roll_axis = Axis(forward_axis_name)

    def _calculate_rest_angles(self, avg_back: Vector3):
        """Calculate and store rest angles based on deduced axes."""
        print("  Measuring Rest Angles...")
        # Currently at FRONT position
        curr_front_reading = self.hw.read_imu_converted()
        angle_front = curr_front_reading.pitch_angle

        # Calculate Back Angle from avg_back using known axes
        fwd_val_back = getattr(avg_back, self.config.accel_forward_axis.value)
        if self.config.accel_forward_invert:
            fwd_val_back = -fwd_val_back

        vert_val_back = getattr(avg_back, self.config.accel_vertical_axis.value)
        if self.config.accel_vertical_invert:
            vert_val_back = -vert_val_back

        from .utils import calculate_pitch
        angle_back = calculate_pitch(fwd_val_back, vert_val_back)

        self.config.rest_angle_forward = angle_front
        self.config.rest_angle_backward = angle_back

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
        Uses statistical analysis to avoid noise triggers.
        """
        print(">>> Finding Minimum Power <<<")
        print("Ensuring robot is on the floor...")

        pwm = 10
        step = 5
        found = False

        while pwm <= 100:
            print(f"  Testing PWM {pwm}...")

            # Pulse and Measure (Longer pulse for better signal)
            result = self.hw.drive_and_measure(pwm, pwm, 0.3)
            time.sleep(0.5) # Wait for stop

            # Check for motion
            # Robust Check: Average magnitude of Gyro should be high, not just max peak (which could be a bump).
            # But short pulse means we might only move briefly.
            # Let's check max_rate > 15 deg/s (increased from 10).

            rate = result.max_rate
            print(f"    Max Rate: {rate:.1f} deg/s")

            if rate > 15.0:
                print(f"  [FOUND] Motion detected at PWM {pwm}.")
                self.config.min_power_visible = pwm
                found = True
                break

            pwm += step
        if not found:
             print("  [FAILURE] Robot did not move even at PWM 100.")
             sys.exit(1)

    # --- Phase 3b: Phasing ---
    def align_motors_phase(self):
        """
        Ensure motors spin together (Straight), not opposite (Spin).
        Verifies via pessimistic loop.
        """
        print(">>> Verifying Motor Phasing <<<")

        attempts = 0
        max_attempts = 3

        while attempts < max_attempts:
            attempts += 1
            print(f"  [Attempt {attempts}] Checking Phasing...")

            # Test 1: Drive with current config
            power = self.config.min_power_visible + 10
            print(f"  Pulse Drive with PWM {power}...")

            result = self.hw.drive_and_measure(power, power, 0.5)
            time.sleep(1.0)

            avg_yaw = result.abs_avg_yaw_rate

            print(f"  -> Avg Yaw Rate: {avg_yaw:.1f} deg/s")

            # Threshold for "Spinning"
            if avg_yaw > 40.0:
                print("  -> High Yaw Rate detected. Motors are fighting (Spinning).")
                print("  -> ACTION: Inverting Right Motor logic to align phases.")
                self.config.motor_r_invert = not self.config.motor_r_invert
                # Loop back to verify
                continue
            else:
                print("  -> Low Yaw Rate. Motors are aligned (Straight).")
                # Proven correct.
                self.config.motor_phasing_verified = True
                return

        print("  [FAILURE] Could not align motor phases after multiple attempts.")
        sys.exit(1)

    # --- Phase 4: Sense of Forward (Pitch Axis & Polarity) ---
    def determine_motor_direction(self):
        """
        Ensure Positive Power = "Stand Up" (Reduce Lean).
        "Kick Up" Test.
        Requirement: Robot must be leaning FORWARD.
        Verifies via pessimistic loop.
        """
        print(">>> Verifying Motor Direction (Kick Up Check) <<<")

        attempts = 0
        max_attempts = 3

        while attempts < max_attempts:
            attempts += 1
            print(f"  [Attempt {attempts}] Checking Direction...")

            # 1. Measure Start Pitch
            imu = self.hw.read_imu_converted()
            start_pitch = imu.pitch_angle
            print(f"  Start Pitch: {start_pitch:.1f}")

            # Enforce Forward Lean (Positive Pitch)
            # If start_pitch is Negative, we are leaning Back.
            if start_pitch < -10.0:
                print("  [ERROR] Robot is leaning BACK (Negative Pitch).")
                print("  -> Positive Power = Forward. Starting from Back would require Negative Power to stand up.")
                print("  -> Please lean the robot FORWARD (on its face/front kickstand).")
                input("Press Enter when re-positioned...")
                continue

            if abs(start_pitch) < 10:
                print("  [WARNING] Robot is too upright. Please lean it over FORWARD.")
                input("Press Enter when leaned...")
                continue # Retry measurement

            # 2. Pulse Positive
            # Calculate direction sign to enforce Positive = Forward
            direction_sign = 1.0 if start_pitch > 0 else -1.0
            power = (self.config.min_power_visible + 20) * direction_sign
            print(f"  Pulsing {power} (Positive=Forward Check)...")

            # We need raw samples for Accel Check
            result = self.hw.drive_and_measure(power, power, 0.4)
            time.sleep(1.0)

            end_pitch = result.final_pitch if result.samples else start_pitch
            print(f"  End Pitch: {end_pitch:.1f}")

            # Check Physics 1: Tilt Change (Kick Up)
            started_leaning = abs(start_pitch)
            ended_leaning = abs(end_pitch)
            delta_lean = started_leaning - ended_leaning

            # Improved (Positive Delta) = Stood Up
            # Worsened (Negative Delta) = Dug In

            # Check Physics 2: Linear Acceleration (The Attempt)
            # If robot is stuck on training wheels, tilt won't change, but it will ACCELERATE forward.
            # Forward Axis is self.config.accel_forward_axis
            avg_accel_fwd = 0.0
            if result.samples:
                # Calculate average forward acceleration from raw samples
                fwd_axis_name = self.config.accel_forward_axis.value
                invert = self.config.accel_forward_invert

                # Sum raw values
                total_fwd = sum(getattr(s.accel_raw, fwd_axis_name) for s in result.samples)
                avg_raw = total_fwd / len(result.samples)

                # Apply inversion
                avg_accel_fwd = -avg_raw if invert else avg_raw

            print(f"  Delta Lean: {delta_lean:.1f}, Avg Fwd Accel: {avg_accel_fwd:.2f}")

            # Decision Logic
            # Case A: Significant Tilt Improvement
            if delta_lean > 2.0:
                print("  [SUCCESS] Robot moved towards upright (Stood Up). Direction is Correct.")
                self.config.motor_direction_verified = True
                return

            # Case B: Significant Tilt Worsening
            elif delta_lean < -2.0:
                 print("  [FAILURE] Robot dug in (Leaned More). Direction is Inverted.")
                 print("  -> ACTION: Inverting BOTH motors to fix direction.")
                 self.config.motor_l_invert = not self.config.motor_l_invert
                 self.config.motor_r_invert = not self.config.motor_r_invert
                 continue

            # Case C: No Tilt Change (Stuck on wheels?), check Accel
            else:
                print("  [WARNING] Tilt did not change significantly. Checking Linear Acceleration...")
                # Threshold for Accel? Gravity is ~1.0 (or 9.8 depending on unit).
                # MPU6050 raw is usually normalized to g's in RobotHardware?
                # Let's check RobotHardware units.
                # Assuming g's. 0.1g is detectable.

                if avg_accel_fwd > 0.15:
                    print("  [SUCCESS] Detected Forward Acceleration. Direction is Correct.")
                    self.config.motor_direction_verified = True
                    return
                elif avg_accel_fwd < -0.15:
                    print("  [FAILURE] Detected Backward Acceleration. Direction is Inverted.")
                    print("  -> ACTION: Inverting BOTH motors to fix direction.")
                    self.config.motor_l_invert = not self.config.motor_l_invert
                    self.config.motor_r_invert = not self.config.motor_r_invert
                    continue
                else:
                    print("  [WARNING] Neither Tilt nor Acceleration was conclusive. Retrying with higher power...")
                    # Loop will retry. Maybe we should bump power?
                    # The loop uses min_power + 20. Maybe increase min_power_visible temporarily?
                    # Or just retry.
                    continue

        print("  [FAILURE] Could not determine motor direction after multiple attempts.")
        sys.exit(1)

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

        attempts = 0
        while attempts < 3:
            attempts += 1

            # 1. Establish Up Vector (Opposite of Gravity)
            # We need to know which way is Up in the Raw Sensor Frame.
            # Re-read static gravity to be sure.
            print("  Measuring Gravity for Reference...")
            accel, _ = self.hw.read_imu_raw()
            # Gravity points DOWN. Up Vector = -Gravity.
            up_vector = -accel

            # 2. Arc Command ("The Arc")
            # Drive Ch0 High, Ch1 Low (but same sign).
            # This reduces drag on training wheels compared to spinning in place.
            power_high = self.config.min_power_visible + 20
            power_low = power_high * 0.5

            print(f"  Driving Arc (Ch0={power_high:.0f}, Ch1={power_low:.0f})...")

            # We need Raw Gyro data to do the Dot Product.
            # execute_maneuver returns IMUReading which contains raw data.
            result = self.hw.execute_maneuver([(power_high, power_low, 1.5)])
            raw_gyro_samples = [s.gyro_raw for s in result.samples if s.gyro_raw]

            if not raw_gyro_samples:
                print("  [WARNING] No gyro samples collected. Retrying...")
                continue

            # Average Gyro Vector
            gyro_sum = Vector3(0.0, 0.0, 0.0)
            for g in raw_gyro_samples:
                gyro_sum += g
            avg_gyro = gyro_sum / len(raw_gyro_samples)

            # 3. Calculate Dot Product: Up . Omega
            # If Positive: Rotation is CCW (Left) around Up.
            # If Negative: Rotation is CW (Right) around Up.
            dot_prod = up_vector.dot(avg_gyro)

            print(f"  Dot Product (Up . Gyro): {dot_prod:.2f}")

            if abs(dot_prod) < 500: # Threshold depends on units. Raw gyro is usually LSBs or deg/s.
                # Assuming deg/s, 500 is very fast?
                # Raw units from MPU6050 might be large integers or scaled floats.
                # If floats (deg/s): 20 deg/s is significant.
                # Let's check typical magnitudes. If raw is deg/s, 20 is good.
                # If raw is LSB (e.g. 16384/g), it's huge.
                # MPU6050Adapter returns Vector3.from_dict.
                # mpu6050 library usually returns degrees/second for gyro.
                pass

            # Robustness check
            if abs(dot_prod) < 10.0:
                 print("  [WARNING] Spin rate too low to determine direction. Retrying...")
                 continue

            # 4. Deduce Motor Mapping
            # We commanded (Ch0=High, Ch1=Low).
            # If Ch0=Left, Ch1=Right -> Left is Outer -> Turn Right (CW).
            # If Ch0=Right, Ch1=Left -> Right is Outer -> Turn Left (CCW).

            if dot_prod > 0:
                # CCW Turn (Left).
                # Means Right Motor was Outer (High).
                # So Ch0 is Right.
                print("  -> Detected Counter-Clockwise (Left) Turn.")
                print("  -> Deduction: Ch0 is Right, Ch1 is Left.")

                # Check current config
                # We want L=1, R=0.
                if self.config.motor_l == 0:
                    print("  -> ACTION: Swapping Channels to Correct Mapping.")
                    self.config.motor_l, self.config.motor_r = self.config.motor_r, self.config.motor_l
                else:
                    print("  -> Mapping is already correct.")

            else:
                # CW Turn (Right).
                # Means Left Motor was Outer (High).
                # So Ch0 is Left.
                print("  -> Detected Clockwise (Right) Turn.")
                print("  -> Deduction: Ch0 is Left, Ch1 is Right.")

                # Check current config
                # We want L=0, R=1.
                if self.config.motor_l == 1:
                     print("  -> ACTION: Swapping Channels to Correct Mapping.")
                     self.config.motor_l, self.config.motor_r = self.config.motor_r, self.config.motor_l
                else:
                    print("  -> Mapping is already correct.")

            # 5. Deduce Gyro Yaw Polarity
            # We want Positive Yaw Rate for CCW (Left) Spin.
            # We detected the physical spin direction via Dot Product.
            # Now let's see what the "Converted" Yaw Rate says with current config.
            # We need to re-calc the yaw rate from raw data using current config flags.

            # Re-read config flags
            yaw_axis = self.config.gyro_yaw_axis
            yaw_invert = self.config.gyro_yaw_invert

            # Calc average raw yaw component
            raw_yaw_component = getattr(avg_gyro, yaw_axis.value)

            # Apply current invert
            current_yaw_rate = -raw_yaw_component if yaw_invert else raw_yaw_component

            print(f"  Current Config Yaw Rate: {current_yaw_rate:.1f}")

            # If Dot Prod > 0 (CCW), we WANT Positive Yaw Rate.
            # If Dot Prod < 0 (CW), we WANT Negative Yaw Rate.

            # Check agreement
            if (dot_prod > 0 and current_yaw_rate < 0) or \
               (dot_prod < 0 and current_yaw_rate > 0):
                print("  -> Gyro Yaw Polarity is Inverted relative to reality.")
                print("  -> ACTION: Inverting Gyro Yaw.")
                self.config.gyro_yaw_invert = not self.config.gyro_yaw_invert
            else:
                print("  -> Gyro Yaw Polarity is Correct.")

            self.config.motor_channels_verified = True
            return

        print("  [FAILURE] Could not deduce Left/Right after multiple attempts.")
        sys.exit(1)

    # --- Phase 6: The Stride (Motor Trimming) ---
    def calibrate_motor_trim(self):
        """
        Calibrate Motor Trim to ensure straight driving.
        Adaptive logic handles mismatched motors (up to 30%).
        """
        print(">>> Motor Trim Calibration <<<")
        print("I will drive straight and measure drift.")
        self.hw.wait_for_stability()

        # We will adjust trim until Yaw Rate is small.
        max_attempts = 15

        for attempt in range(max_attempts):
            current_trim = self.config.motor_trim
            print(f"  [Attempt {attempt+1}/{max_attempts}] Trim: {current_trim:.3f}")

            power = self.config.min_power_visible + 15
            # Drive Straight
            result = self.hw.drive_and_measure(power, power, 1.0)

            if not result.samples:
                continue

            # Average Yaw Rate
            avg_yaw = result.avg_yaw_rate
            print(f"    Avg Yaw Drift: {avg_yaw:.2f} deg/s")

            if abs(avg_yaw) < 2.0:
                print("  [SUCCESS] Drift is negligible.")
                return

            # Adaptive Gain
            # If error is large (>10 deg/s), take bigger steps.
            # If Yaw > 0 (Left Turn), Right is stronger -> Increase Trim.
            base_gain = 0.005
            if abs(avg_yaw) > 10.0:
                gain = 0.015 # 3x faster
            else:
                gain = base_gain

            correction = avg_yaw * gain

            # Apply Correction
            new_trim = current_trim + correction

            # Check bounds (Clamp to +/- 0.4 now, slightly wider range)
            new_trim = max(-0.4, min(0.4, new_trim))

            if abs(new_trim - current_trim) < 0.001 and abs(avg_yaw) > 5.0:
                 print("  [WARNING] Trim saturated or not moving.")

            self.config.motor_trim = new_trim
            print(f"    -> Correction: {correction:+.4f} -> New Trim: {self.config.motor_trim:.3f}")

        print("  [WARNING] Could not perfectly trim motors. Saving best effort.")

    # --- Phase 7: Kick-Up Dynamics ---
    def _wait_for_start_condition(self, check_fn: Callable[[float], bool] | None, msg: str):
        """
        Wait for stability and a specific pitch condition.
        """
        print(msg)
        while True:
            self.hw.wait_for_stability(duration=1.0)

            if check_fn is None:
                return

            curr = self.hw.read_imu_converted()
            if check_fn(curr.pitch_angle):
                print("  [OK] Position Verified.")
                return
            else:
                print(f"  [WAITING] Position incorrect (Pitch={curr.pitch_angle:.1f}). Please adjust.")
                time.sleep(1.0)

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

        # check success
        if check_success(final_pitch):
             # Even if successful, check alignment briefly?
             # No, if it worked, it worked. But let's be safe.
             pass
        else:
             # If it failed to flip, we still check alignment to see if that was the cause.
             pass

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

    def _run_kickup_test(self, direction: str, check_fn: Callable[[float], bool], config_attr: str, msg: str):
        """Run kick-up test for a single direction."""
        print(f"\n[Test] Kick-Up from {direction}")

        # Start loop
        power = self.config.min_power_visible + 10  # Start conservative? Or 20?
        power = max(20, power)
        alignment_retries = 0

        # Ensure we start in position
        self._wait_for_start_condition(check_fn, msg)

        while power <= 100:
            print(f"  Trying Power {power}...")

            result = self._attempt_dynamic_flop(direction, power)

            if result == "SUCCESS":
                print(f"  [SUCCESS] Flopped at {power}.")
                setattr(self.config.control, config_attr, power)
                self.config.save()
                break

            elif result == "FAIL_ALIGNMENT":
                print("  [DETECTED DRIFT] Robot is not moving straight. Pausing to re-align.")
                if alignment_retries < 3:
                    alignment_retries += 1
                    self.calibrate_motor_trim()
                    self.config.save()  # Save the new trim
                    print(f"  [RETRY] Retrying Power {power} with new trim...")
                    self._wait_for_start_condition(check_fn, "Reset position...")
                    # Continue loop with SAME power
                    continue
                else:
                    print("  [WARNING] Alignment failed too many times. Ignoring drift.")
                    # Treat as FAIL_POWER -> Increment

            # FAIL_POWER or alignment gave up
            print("  [FAIL] Did not flop.")
            power += 5
            if power <= 100:
                self._wait_for_start_condition(check_fn, "Reset position...")

    def find_flop_thresholds(self):
        """
        Discover Kick-Up Power for both directions using Dynamic Momentum.
        Uses calibrated rest angles for robust start detection.
        """
        print(">>> Dynamic Kick-Up Calibration (Roll & Slam) <<<")

        # Robust Start Conditions using Calibrated Rest Angles
        # If not calibrated (None), fallback to default -10/10
        rest_back = self.config.rest_angle_backward if self.config.rest_angle_backward is not None else -10.0
        rest_front = self.config.rest_angle_forward if self.config.rest_angle_forward is not None else 10.0

        def check_back(p):
            return p < (rest_back + 10.0) and p < -5.0

        def check_front(p):
            return p > (rest_front - 10.0) and p > 5.0

        self._run_kickup_test("BACK", check_back, "kickup_power_forward", "Place robot on BACK wheel.")
        self._run_kickup_test("FRONT", check_front, "kickup_power_backward", "Place robot on FRONT wheel.")

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
        power = self.config.min_power_visible + 10

        result = self.hw.drive_and_measure(power, power, 1.0)
        time.sleep(0.5)

        avg_yaw = result.abs_avg_yaw_rate

        print(f"    Avg Yaw Rate: {avg_yaw:.1f} deg/s")

        if avg_yaw > 40.0:
             print("  [FAILURE] Robot spun while trying to drive straight.")
             print("  -> Possible Phase mismatch or Motor Direction mismatch despite checks.")
             sys.exit(1)

        # 2. Verify Right Turn
        print("  [Check 2] Turn Right (Clockwise)...")
        # Command L+, R- (Spin) -> Changed to Arc (L+, R_low) to avoid drag

        power_high = self.config.min_power_visible + 15
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
                 sys.exit(1)
             else:
                 print("  [FAILURE] Robot did not turn significantly.")
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
