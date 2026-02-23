import time
import sys
try:
    import smbus2 as smbus
except ImportError:
    smbus = None

from typing import Callable, Any
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware, IMUReading
from .enums import Axis
from .utils import (
    analyze_dominance,
    cross_product,
    Vector3,
    get_i2c_failure_report,
    run_diagnostics
)


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
        run_diagnostics()

        steps = [
            ("I2C Bus Assignments",
             lambda c: c.motor_i2c_bus is None or c.imu_i2c_bus is None,
             self.discover_buses),

            ("Spatial Orientation",
             lambda c: c.accel_vertical_axis is None,
             lambda: (self.init_hw(), self.calibrate_static_orientation())),

            ("Hardware Initialization",
             lambda c: self.hw is None,
             self.init_hw),

            ("Motor Candidates",
             lambda c: c.motor_l is None or c.motor_r is None,
             lambda: (print("-> [INFO] Setting default motors (0,1)."),
                      setattr(self.config, 'motor_l', 0),
                      setattr(self.config, 'motor_r', 1),
                      self.init_hw())),

            ("Friction Threshold",
             lambda c: c.min_power_visible == 0,
             self.find_min_power),

            ("Motor Phasing",
             lambda c: not c.motor_phasing_verified,
             self.align_motors_phase),

            ("Motor Direction",
             lambda c: not c.motor_direction_verified,
             self.determine_motor_direction),

            ("Left/Right Identity",
             lambda c: not c.motor_channels_verified,
             self.deduce_left_right_autonomous),

            ("Motor Trim",
             lambda c: not c.motor_trim_verified,
             lambda: (self.calibrate_motor_trim(), setattr(self.config, 'motor_trim_verified', True))),

            ("Mechanical Backlash",
             lambda c: not c.backlash_verified,
             self.measure_backlash),

            ("Kick-Up Dynamics",
             lambda c: c.control.kickup_power_forward == 0.0,
             self.find_flop_thresholds)
        ]

        while True:
            print("\n---------------------------------------------------")
            print("Checking Knowledge Base...")

            action_taken = False
            for name, condition, action in steps:
                if condition(self.config):
                    print(f"-> [MISSING] {name}.")
                    action()
                    self.config.save()
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
        c = self.config
        print("\n[SUCCESS] Hardware Verified. Ready for Agent (Main Brain).")
        print("Summary:")
        print(f"  Buses: Motor={c.motor_i2c_bus}, IMU={c.imu_i2c_bus}")
        print(f"  Axes: Vert={c.accel_vertical_axis}, Fwd={c.accel_forward_axis}, Pitch={c.gyro_pitch_axis}, Yaw={c.gyro_yaw_axis}")
        print(f"  Motors: MinPower={c.min_power_visible}, L={c.motor_l}(Inv={c.motor_l_invert}), R={c.motor_r}(Inv={c.motor_r_invert})")
        print(f"  Trim: {c.motor_trim:.3f}")
        print(f"  KickUp: Fwd={c.control.kickup_power_forward:.1f}, Bwd={c.control.kickup_power_backward:.1f}")

    # --- Phase 1: Hardware Connectivity ---
    def _scan_candidates(self, name: str, check_fn: Callable[[Any], bool]) -> int | None:
        """
        Scans I2C buses for a device using a callback.
        Returns the bus ID if found, else None.
        """
        candidates = [1, 3, 0, 2]
        for bus_id in candidates:
            try:
                # smbus2 handles bus opening gracefully
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

            # Use improved diagnostics for the likely bus (1 or 3)
            # Default to checking both or giving a hint
            print("  [DIAGNOSTIC] Analyzing potential causes...")
            # We don't know the address here easily unless passed, but we can guess based on name
            addr = 0x22 if "PiconZero" in name else 0x68

            # Check likely buses
            for b in [1, 3]:
                print(f"  --- Bus {b} Report ---")
                print(get_i2c_failure_report(b, addr, name))

            sys.exit(1)
        return bus

    def _drive_and_wait(self, left: float, right: float, duration: float,
                        wait_stable: bool = True) -> Any:
        """Helper to drive, measure, and wait."""
        if wait_stable:
            self.hw.wait_for_stability()
        result = self.hw.drive_and_measure(left, right, duration)
        time.sleep(0.5)
        return result

    def _verify_with_retries(self, name: str, test_fn: Callable[[], Any],
                             check_fn: Callable[[Any], bool | str],
                             max_attempts: int = 3) -> None:
        """
        Generic verification loop with retries.
        check_fn should return True (Pass), False (Fail/Retry),
        or a string "FAIL_FATAL" / "FAIL_RETRY".
        """
        print(f">>> Verifying {name} <<<")
        for i in range(max_attempts):
            print(f"  [Attempt {i+1}] Checking {name}...")
            result = test_fn()

            outcome = check_fn(result)
            if outcome is True or outcome == "PASS":
                print(f"  [SUCCESS] {name} Verified.")
                return

            if outcome == "FAIL_FATAL":
                break

            # If outcome is False or "FAIL_RETRY", loop continues
            print(f"  [RETRY] {name} check failed/ambiguous.")

        print(f"  [FAILURE] Could not verify {name} after {max_attempts} attempts.")
        sys.exit(1)

    def _find_threshold(self, name: str, start: float, step: float, limit: float,
                        action_fn: Callable[[float], Any],
                        check_fn: Callable[[Any], bool],
                        fail_action: Callable[[Any], bool] = None) -> float:
        """
        Find a threshold value by incrementing.
        fail_action: Optional callback on failure. Return True to retry SAME level.
        """
        print(f">>> Finding Threshold: {name} <<<")
        val = start
        while val <= limit:
            print(f"  Testing {val}...")
            result = action_fn(val)

            if check_fn(result):
                print(f"  [FOUND] {name} at {val}.")
                return val

            retry_same = False
            if fail_action:
                retry_same = fail_action(result)

            if not retry_same:
                val += step

        print(f"  [FAILURE] Could not find {name} within limit {limit}.")
        sys.exit(1)

    def _measure_gravity_with_hardware(self) -> Vector3:
        """Measure gravity using hardware abstraction."""
        # Drive 0,0 for 1.0 second
        res = self.hw.drive_and_measure(0, 0, 1.0)
        if not res.samples:
            return Vector3(0.0, 0.0, 0.0)

        avg = Vector3(0.0, 0.0, 0.0)
        count = 0
        for s in res.samples:
            if s.accel_raw:
                avg += s.accel_raw
                count += 1
        if count > 0:
            return avg / count
        return Vector3(0.0, 0.0, 0.0)

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
        avg_back = self._measure_gravity_with_hardware()
        print(f"  [DEBUG] Avg Back Vector: {avg_back}")

        # 2. Read Front Rest Gravity
        print("\n  Now TIP ROBOT FORWARD to Front Wheel.")
        input("Press Enter to measure FRONT position...")
        self.hw.wait_for_stability(duration=1.0)

        avg_front = self._measure_gravity_with_hardware()
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
        """
        print(">>> Finding Minimum Power <<<")
        print("Ensuring robot is on the floor...")

        def action(p):
            return self._drive_and_wait(p, p, 0.3, wait_stable=False)

        def check(res):
            rate = res.max_rate
            print(f"    Max Rate: {rate:.1f} deg/s")
            return rate > 15.0

        found = self._find_threshold("Minimum Power", 10, 5, 100, action, check)
        self.config.min_power_visible = found

    # --- Phase 3b: Phasing ---
    def align_motors_phase(self):
        """
        Ensure motors spin together (Straight), not opposite (Spin).
        """
        def test():
            p = self.config.min_power_visible + 10
            return self._drive_and_wait(p, p, 0.5)

        def verify(res):
            yaw = res.abs_avg_yaw_rate
            print(f"  -> Avg Yaw Rate: {yaw:.1f} deg/s")
            if yaw > 40.0:
                print("  -> High Yaw Rate detected. Motors are fighting (Spinning).")
                print("  -> ACTION: Inverting Right Motor logic to align phases.")
                self.config.motor_r_invert = not self.config.motor_r_invert
                return False
            print("  -> Low Yaw Rate. Motors are aligned (Straight).")
            self.config.motor_phasing_verified = True
            return True

        self._verify_with_retries("Motor Phasing", test, verify)

    # --- Phase 4: Sense of Forward (Pitch Axis & Polarity) ---
    def determine_motor_direction(self):
        """
        Ensure Positive Power = "Stand Up" (Reduce Lean).
        "Kick Up" Test.
        Requirement: Robot must be leaning FORWARD.
        """
        def test():
            # Check Start Pitch
            imu = self.hw.read_imu_converted()
            start_pitch = imu.pitch_angle
            print(f"  Start Pitch: {start_pitch:.1f}")

            if start_pitch < -10.0:
                print("  [ERROR] Leaning BACK. Lean FORWARD.")
                input("Press Enter when re-positioned...")
                return None
            if abs(start_pitch) < 10:
                print("  [WARNING] Too upright. Lean FORWARD.")
                input("Press Enter when leaned...")
                return None

            direction_sign = 1.0 if start_pitch > 0 else -1.0
            power = (self.config.min_power_visible + 20) * direction_sign
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
                fwd_axis = self.config.accel_forward_axis.value
                invert = self.config.accel_forward_invert
                # Using generator expression for sum
                total = sum(getattr(s.accel_raw, fwd_axis) for s in res.samples)
                avg_accel_fwd = - (total / len(res.samples)) if invert else (total / len(res.samples))

            print(f"  Delta Lean: {delta_lean:.1f}, Avg Fwd Accel: {avg_accel_fwd:.2f}")

            if delta_lean > 2.0 or avg_accel_fwd > 0.15:
                 print("  [SUCCESS] Direction Correct.")
                 self.config.motor_direction_verified = True
                 return True
            elif delta_lean < -2.0 or avg_accel_fwd < -0.15:
                 print("  [FAILURE] Direction Inverted. Fixing...")
                 self.config.motor_l_invert = not self.config.motor_l_invert
                 self.config.motor_r_invert = not self.config.motor_r_invert
                 return False

            print("  [WARNING] Inconclusive.")
            return False

        self._verify_with_retries("Motor Direction", test, verify)

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

    # --- Phase 6b: Mechanical Backlash ---
    def measure_backlash(self):
        """
        Measure gear slop by pushing gears forward, then timing how long
        reverse power takes to actually move the chassis.
        """
        print(">>> Measuring Mechanical Backlash (Gear Slop) <<<")
        self.hw.wait_for_stability(duration=1.0)

        test_power = self.config.min_power_visible + 10

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
        self.config.control.backlash_pulse_time = compensated_slop
        self.config.backlash_verified = True

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

        self._kickup_alignment_retries = 0

        def action(p):
            self._wait_for_start_condition(check_fn, msg)
            return self._attempt_dynamic_flop(direction, p)

        def check(res):
            return res == "SUCCESS"

        def fail(res):
            if res == "FAIL_ALIGNMENT":
                print("  [DETECTED DRIFT] Re-aligning...")
                if self._kickup_alignment_retries < 3:
                    self._kickup_alignment_retries += 1
                    self.calibrate_motor_trim()
                    self.config.save()
                    return True # Retry same power
                print("  [WARNING] Ignoring drift.")
            return False # Increment power

        start_p = max(20, self.config.min_power_visible + 10)
        found = self._find_threshold(f"KickUp {direction}", start_p, 5, 100, action, check, fail)

        if found:
            setattr(self.config.control, config_attr, found)
            self.config.save()

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
