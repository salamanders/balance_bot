import time
import sys
import smbus
from typing import Callable
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

        # Temporary runtime state
        self.temp_motor_l = 0
        self.temp_motor_r = 1
        self.temp_invert_l = False
        self.temp_invert_r = False

    def init_hw(self):
        """
        Initialize hardware with current known config.
        If config is incomplete, fills in safe defaults for discovery.
        """
        if self.hw:
            try:
                self.hw.stop()
                self.hw.cleanup()
            except Exception:
                pass

        # We need buses to be discovered first
        if self.config.motor_i2c_bus is None or self.config.imu_i2c_bus is None:
            # Cannot init HW without buses.
            return

        # Use Config values or Safe Defaults
        # Note: RobotHardware requires Axis enums, but config might be None.
        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            # Sensors: Default to Z/Y/X if unknown, just to allow raw reading
            gyro_axis=self.config.gyro_pitch_axis or Axis.X,
            gyro_invert=self.config.gyro_pitch_invert,
            gyro_yaw_axis=self.config.gyro_yaw_axis or Axis.Z,
            gyro_yaw_invert=self.config.gyro_yaw_invert,
            gyro_roll_axis=self.config.gyro_roll_axis or Axis.Y,
            gyro_roll_invert=self.config.gyro_roll_invert,
            accel_vertical_axis=self.config.accel_vertical_axis or Axis.Z,
            accel_vertical_invert=self.config.accel_vertical_invert,
            accel_forward_axis=self.config.accel_forward_axis or Axis.Y,
            accel_forward_invert=self.config.accel_forward_invert,
            motor_i2c_bus=self.config.motor_i2c_bus,
            imu_i2c_bus=self.config.imu_i2c_bus,
            motor_trim=self.config.motor_trim,
        )
        self.hw.init()

    def cleanup(self):
        if self.hw:
            self.hw.stop()
            self.hw.cleanup()

    def wait_for_stability(self, duration: float = 2.0, threshold: float = 2.0):
        """
        Wait for the robot to be stable (gyro rates low) for a duration.
        Blocks until the condition is met.
        """
        print(f"Waiting for stability (rates < {threshold} deg/s) for {duration}s...")

        start_stable_time = None
        last_log = 0.0

        while True:
            try:
                # We need HW to read sensors
                if not self.hw:
                    self.init_hw()

                reading = self.hw.read_imu_converted()

                # Calculate total rate magnitude (sum of abs rates is a simple metric)
                rate = abs(reading.pitch_rate) + abs(reading.yaw_rate) + abs(reading.roll_rate)

                if rate < threshold:
                    if start_stable_time is None:
                        start_stable_time = time.time()
                    elif time.time() - start_stable_time >= duration:
                        print("  [STABLE] Robot is still.")
                        return
                else:
                    if start_stable_time is not None:
                        # Reset if movement detected
                        start_stable_time = None
                        if time.time() - last_log > 1.0:
                            print(f"  [MOVING] Rate {rate:.1f} > {threshold}. Waiting...")
                            last_log = time.time()

                time.sleep(0.05)

            except KeyboardInterrupt:
                raise
            except Exception as e:
                # Handle occasional I2C read errors gracefully
                print(f"  [Error reading IMU] {e}")
                time.sleep(0.1)

    def drive_and_measure(self, left_power: float, right_power: float, duration: float, sample_interval: float = 0.01) -> list[IMUReading]:
        """
        Drive motors for a duration and collect IMU readings.
        Returns a list of IMUReading objects collected during the drive.
        """
        samples = []
        try:
            self.hw.set_motors(left_power, right_power)
            start = time.time()
            while time.time() - start < duration:
                samples.append(self.hw.read_imu_converted())
                time.sleep(sample_interval)
        finally:
            self.hw.stop()
        return samples

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

    # --- Tier 1: Hardware Connectivity ---
    def _scan_for_device(self, name: str, check_fn) -> int | None:
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

    def discover_buses(self):
        """
        Scan I2C buses [0, 1, 2, 3] for PiconZero (0x22) and MPU6050 (0x68).
        """
        print("Scanning I2C Buses...")

        # 1. Find Motors (0x22)
        def check_motor(bus):
            bus.read_byte_data(0x22, 0)
            return True

        found_motor = self._scan_for_device("PiconZero (Motors)", check_motor)

        if found_motor is None:
            print("  [FAILURE] Could not find PiconZero (0x22) on any bus.")
            sys.exit(1)

        self.config.motor_i2c_bus = found_motor

        # 2. Find IMU (0x68)
        def check_imu(bus):
            return bus.read_byte_data(0x68, 0x75) == 0x68

        found_imu = self._scan_for_device("MPU6050 (IMU)", check_imu)

        if found_imu is None:
            print("  [FAILURE] Could not find MPU6050 (0x68) on any bus.")
            sys.exit(1)

        self.config.imu_i2c_bus = found_imu

    # --- Tier 2: The Physical World (Sensors) ---
    def calibrate_static_orientation(self):
        """
        Map Physical Axes (X, Y, Z) to Logical Axes (Vertical, Forward, Pitch).
        Requirement: Robot on floor, resting on one training wheel.
        """
        print(">>> Calibrating Orientation <<<")
        print("Please place the robot on the FLOOR.")
        print("Ensure motors are OFF and it is resting on the BACK training wheel.")
        self.wait_for_stability(duration=2.0)

        # 1. Read Gravity (Vertical Axis)
        print("  Measuring Gravity (Vertical)...")
        time.sleep(0.5)
        accel_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        samples = 50
        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            # a is Vector3
            accel_sum['x'] += a.x
            accel_sum['y'] += a.y
            accel_sum['z'] += a.z
            time.sleep(0.01)

        avg_back = Vector3(
            accel_sum['x'] / samples,
            accel_sum['y'] / samples,
            accel_sum['z'] / samples
        )

        # Dominant axis in static position is Gravity (Vertical)
        vert, _, _ = analyze_dominance(dict(avg_back.items()), "Vertical (Gravity)")
        self.config.accel_vertical_axis = Axis(vert)
        self.config.accel_vertical_invert = avg_back[vert] < 0
        print(f"  -> Vertical Axis: {vert.upper()} (Invert: {self.config.accel_vertical_invert})")

        # 2. Read Lean (Forward Axis)
        sorted_axes = sorted(avg_back.items(), key=lambda x: abs(x[1]), reverse=True)
        # 0 is Vertical. 1 is Forward. 2 is Pitch (Axle).
        forward_axis = sorted_axes[1][0]
        pitch_axis = sorted_axes[2][0]

        self.config.accel_forward_axis = Axis(forward_axis)
        print(f"  -> Forward Axis Candidate: {forward_axis.upper()}")
        print(f"  -> Pitch Axis Candidate: {pitch_axis.upper()}")

        # 3. Determine Pitch Axis Orientation (Using Cross Product)
        print("\n  Now TIP ROBOT FORWARD to Front Wheel.")
        input("Press Enter to measure FRONT position...")

        accel_sum_front = {"x": 0.0, "y": 0.0, "z": 0.0}
        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            accel_sum_front['x'] += a.x
            accel_sum_front['y'] += a.y
            accel_sum_front['z'] += a.z
            time.sleep(0.01)

        avg_front = Vector3(
            accel_sum_front['x'] / samples,
            accel_sum_front['y'] / samples,
            accel_sum_front['z'] / samples
        )

        # Cross Product: Back x Front -> Pitch Axis Vector
        axis_vec = cross_product(avg_back, avg_front)

        # Verify that our calculated pitch axis matches the cross product magnitude
        detected_pitch, _, _ = analyze_dominance({k: abs(v) for k, v in axis_vec.items()}, "Pitch Axis (CrossProd)")

        if detected_pitch != pitch_axis:
             print(f"  [WARNING] Cross product suggested {detected_pitch} but magnitude suggested {pitch_axis}. Trusting Cross Product.")
             pitch_axis = detected_pitch

        self.config.gyro_pitch_axis = Axis(pitch_axis)
        val = axis_vec[pitch_axis]
        self.config.gyro_pitch_invert = val < 0

        print(f"  -> Pitch Axis: {pitch_axis.upper()} (Invert: {self.config.gyro_pitch_invert})")

        # Deduce Yaw and Roll
        all_axes = {'x', 'y', 'z'}
        used = {vert, pitch_axis}
        remaining = list(all_axes - used)
        if len(remaining) == 1:
            # Gyro Yaw is rotation around Vertical Axis
            self.config.gyro_yaw_axis = Axis(vert)
            # Gyro Roll Axis is rotation around Forward Axis
            self.config.gyro_roll_axis = Axis(forward_axis)
        else:
            print("  [ERROR] Axis deduction logic failed.")

        # Reload HW with new sensor config
        self.init_hw()

        # Deduce Forward Axis Inversion
        delta_fwd = avg_front[forward_axis] - avg_back[forward_axis]
        self.config.accel_forward_invert = delta_fwd < 0
        print(f"  -> Forward Axis: {forward_axis.upper()} (Invert: {self.config.accel_forward_invert})")


    # --- Tier 3a: Friction Threshold ---
    def find_min_power(self):
        """
        Find minimum PWM to overcome friction.
        """
        print(">>> Finding Minimum Power <<<")
        print("Ensuring robot is on the floor...")

        pwm = 10
        step = 5
        found = False

        while pwm <= 100:
            print(f"  Testing PWM {pwm}...")

            # Pulse and Measure
            samples = self.drive_and_measure(pwm, pwm, 0.2)
            time.sleep(0.5) # Wait for stop

            # Check for motion (Gyro Rate > Threshold)
            # We look for ANY significant rotation on ANY axis.
            # Using 10 deg/s as a safe threshold for "moved".
            motion_detected = False
            if samples:
                # Compare max rate seen to threshold
                max_rate = max(
                    (abs(s.pitch_rate) + abs(s.yaw_rate) + abs(s.roll_rate))
                    for s in samples
                )
                if max_rate > 10.0:
                    motion_detected = True

            if motion_detected:
                print(f"  [FOUND] Motion detected at PWM {pwm}.")
                self.config.min_power_visible = pwm
                found = True
                break

            pwm += step

        if not found:
             print("  [FAILURE] Robot did not move even at PWM 100.")
             sys.exit(1)

    # --- Tier 3b: Phasing ---
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
            self.init_hw()

            # Test 1: Drive with current config
            power = self.config.min_power_visible + 10
            print(f"  Pulse Drive with PWM {power}...")

            samples = self.drive_and_measure(power, power, 0.5)
            time.sleep(1.0)

            if not samples:
                avg_yaw = 0.0
            else:
                yaw_sum = sum(abs(s.yaw_rate) for s in samples)
                avg_yaw = yaw_sum / len(samples)

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

    # --- Tier 3c: Direction ---
    def determine_motor_direction(self):
        """
        Ensure Positive Power = "Stand Up" (Reduce Lean).
        "Kick Up" Test.
        Verifies via pessimistic loop.
        """
        print(">>> Verifying Motor Direction (Kick Up Check) <<<")

        attempts = 0
        max_attempts = 3

        while attempts < max_attempts:
            attempts += 1
            print(f"  [Attempt {attempts}] Checking Direction...")
            self.init_hw()

            # 1. Measure Start Pitch
            imu = self.hw.read_imu_converted()
            start_pitch = imu.pitch_angle
            print(f"  Start Pitch: {start_pitch:.1f}")

            if abs(start_pitch) < 10:
                print("  [WARNING] Robot is too upright. Please lean it over (Front or Back).")
                input("Press Enter when leaned...")
                continue # Retry measurement

            # 2. Pulse Positive
            # Calculate direction sign to enforce Positive = Forward
            direction_sign = 1.0 if start_pitch > 0 else -1.0
            power = (self.config.min_power_visible + 20) * direction_sign
            print(f"  Pulsing {power} (Positive=Forward Check)...")

            samples = self.drive_and_measure(power, power, 0.3)
            time.sleep(1.0)

            end_pitch = samples[-1].pitch_angle if samples else start_pitch

            print(f"  End Pitch: {end_pitch:.1f}")

            # Check Physics
            started_leaning = abs(start_pitch)
            ended_leaning = abs(end_pitch)

            # Did we move towards upright (0)?
            # If start=-30, upright=0.
            # If end=-20, abs(-20) < abs(-30) -> Improved.
            # If end=-40, abs(-40) > abs(-30) -> Worsened.

            # Note: We need significant movement to be sure.
            if abs(started_leaning - ended_leaning) < 2.0:
                print("  [WARNING] Movement too small to determine direction. Retrying...")
                continue

            if ended_leaning < started_leaning:
                print("  [SUCCESS] Robot moved towards upright (Stood Up). Direction is Correct.")
                self.config.motor_direction_verified = True
                return
            else:
                print("  [FAILURE] Robot dug in (Leaned More). Direction is Inverted.")
                print("  -> ACTION: Inverting BOTH motors to fix direction.")
                self.config.motor_l_invert = not self.config.motor_l_invert
                self.config.motor_r_invert = not self.config.motor_r_invert
                # Loop back to verify
                continue

        print("  [FAILURE] Could not determine motor direction after multiple attempts.")
        sys.exit(1)

    # --- Tier 4: The Human Anchor ---
    def deduce_left_right_autonomous(self):
        """
        Identify Left vs Right Motor using Gyroscope Physics (Right-Hand Rule).
        Also deduces Gyro Yaw Polarity.
        Replaces ask_human_left_right.
        """
        print(">>> Autonomous Left/Right Verification & Yaw Calibration <<<")
        print("I am going to spin to determine my physical identity.")
        self.wait_for_stability()

        attempts = 0
        while attempts < 3:
            attempts += 1
            self.init_hw()

            # 1. Establish Up Vector (Opposite of Gravity)
            # We need to know which way is Up in the Raw Sensor Frame.
            # Re-read static gravity to be sure.
            print("  Measuring Gravity for Reference...")
            accel, _ = self.hw.read_imu_raw()
            # Gravity points DOWN. Up Vector = -Gravity.
            up_vector = Vector3(-accel.x, -accel.y, -accel.z)

            # 2. Spin Command
            # Drive Ch0 Forward, Ch1 Backward.
            # If Ch0=Left, Ch1=Right -> Right Turn (CW).
            # If Ch0=Right, Ch1=Left -> Left Turn (CCW).
            power = self.config.min_power_visible + 15
            print(f"  Spinning with Power {power}...")

            # We need Raw Gyro data to do the Dot Product
            # drive_and_measure returns converted readings.
            # We'll do a manual drive loop to get raw data.
            raw_gyro_samples = []
            try:
                self.hw.set_motors(power, -power)
                start = time.time()
                while time.time() - start < 1.5:
                    _, g = self.hw.read_imu_raw()
                    raw_gyro_samples.append(g)
                    time.sleep(0.01)
            finally:
                self.hw.stop()

            if not raw_gyro_samples:
                print("  [WARNING] No gyro samples collected. Retrying...")
                continue

            # Average Gyro Vector
            avg_gyro = Vector3(
                sum(g.x for g in raw_gyro_samples) / len(raw_gyro_samples),
                sum(g.y for g in raw_gyro_samples) / len(raw_gyro_samples),
                sum(g.z for g in raw_gyro_samples) / len(raw_gyro_samples)
            )

            # 3. Calculate Dot Product: Up . Omega
            # If Positive: Rotation is CCW (Left) around Up.
            # If Negative: Rotation is CW (Right) around Up.
            dot_prod = (up_vector.x * avg_gyro.x) + (up_vector.y * avg_gyro.y) + (up_vector.z * avg_gyro.z)

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
            if dot_prod > 0:
                # CCW Spin (Left).
                # To spin Left: Right Motor Fwd, Left Motor Back.
                # We commanded: Ch0 (Fwd), Ch1 (Back).
                # Therefore: Ch0 is Right, Ch1 is Left.
                print("  -> Detected Counter-Clockwise (Left) Spin.")
                print("  -> Deduction: Ch0 is Right, Ch1 is Left.")

                # Check current config
                # If config says L=0, R=1, it is WRONG.
                # We want L=1, R=0.
                if self.config.motor_l == 0:
                    print("  -> ACTION: Swapping Channels to Correct Mapping.")
                    self.config.motor_l, self.config.motor_r = self.config.motor_r, self.config.motor_l
                else:
                    print("  -> Mapping is already correct.")

            else:
                # CW Spin (Right).
                # To spin Right: Left Motor Fwd, Right Motor Back.
                # We commanded: Ch0 (Fwd), Ch1 (Back).
                # Therefore: Ch0 is Left, Ch1 is Right.
                print("  -> Detected Clockwise (Right) Spin.")
                print("  -> Deduction: Ch0 is Left, Ch1 is Right.")

                # Check current config
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

    # --- Tier 4.5: The Stride (Trim) ---
    def calibrate_motor_trim(self):
        """
        Calibrate Motor Trim to ensure straight driving.
        Phase 4.5.
        """
        print(">>> Motor Trim Calibration <<<")
        print("I will drive straight and measure drift.")
        self.wait_for_stability()

        # We will adjust trim until Yaw Rate is small.
        # Max attempts
        for attempt in range(10):
            self.init_hw()
            print(f"  [Attempt {attempt+1}] Trim: {self.config.motor_trim:.3f}")

            power = self.config.min_power_visible + 15
            # Drive Straight
            samples = self.drive_and_measure(power, power, 1.0)

            if not samples:
                continue

            # Average Yaw Rate
            avg_yaw = sum(s.yaw_rate for s in samples) / len(samples)
            print(f"    Avg Yaw Drift: {avg_yaw:.2f} deg/s")

            if abs(avg_yaw) < 2.0:
                print("  [SUCCESS] Drift is negligible.")
                return

            # Adjust Trim
            # If Yaw > 0 (Turning Left), Right Motor is Stronger.
            # We want to INCREASE trim (Positive Trim reduces Right Motor).
            # Gain: How much trim per deg/s?
            # Start gentle. 0.005 per deg/s?
            # If Drift is 10 deg/s, 0.05 change.
            correction = avg_yaw * 0.005

            self.config.motor_trim += correction

            # Clamp Trim
            self.config.motor_trim = max(-0.3, min(0.3, self.config.motor_trim))
            print(f"    -> New Trim: {self.config.motor_trim:.3f}")

        print("  [WARNING] Could not perfectly trim motors. Saving best effort.")

    # --- Tier 5: Dynamics ---
    def _wait_for_start_condition(self, check_fn: Callable[[float], bool] | None, msg: str):
        """
        Wait for stability and a specific pitch condition.
        """
        print(msg)
        while True:
            self.wait_for_stability(duration=1.0)

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

        samples = []
        try:
            # Phase 1: Setup (Roll)
            self.hw.set_motors(setup_p, setup_p)
            start = time.time()
            while time.time() - start < 0.3:
                samples.append(self.hw.read_imu_converted())
                time.sleep(0.01)

            # Phase 2: Kick (Slam)
            self.hw.set_motors(kick_p, kick_p)
            start = time.time()
            while time.time() - start < 0.4:
                samples.append(self.hw.read_imu_converted())
                time.sleep(0.01)
        finally:
            self.hw.stop()

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

    def find_flop_thresholds(self):
        """
        Discover Kick-Up Power for both directions using Dynamic Momentum.
        """
        print(">>> Dynamic Kick-Up Calibration (Roll & Slam) <<<")
        self.init_hw()

        # Define Tests: (Direction, Start Check Msg, Start Check Fn, Config Attr)
        tests = [
            ("BACK", "Place robot on BACK wheel.", lambda p: p < -10, "kickup_power_forward"),
            ("FRONT", "Place robot on FRONT wheel.", lambda p: p > 10, "kickup_power_backward")
        ]

        for direction, msg, check_fn, config_attr in tests:
            print(f"\n[Test] Kick-Up from {direction}")

            # Start loop
            power = self.config.min_power_visible + 10 # Start conservative? Or 20?
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
                        self.config.save() # Save the new trim
                        print(f"  [RETRY] Retrying Power {power} with new trim...")
                        self._wait_for_start_condition(check_fn, "Reset position...")
                        # Continue loop with SAME power
                        continue
                    else:
                        print("  [WARNING] Alignment failed too many times. Ignoring drift.")
                        # Treat as FAIL_POWER -> Increment
                        power += 5
                        self._wait_for_start_condition(check_fn, "Reset position...")

                else: # FAIL_POWER
                    print("  [FAIL] Did not flop.")
                    power += 5
                    if power <= 100:
                        self._wait_for_start_condition(check_fn, "Reset position...")

    # --- Tier 6: Final Verification ---
    def verify_final_configuration(self):
        """
        Autonomous Verification of the learned configuration.
        Pessimistic check:
        1. Drive Straight: Verify Yaw Rate is low, Accel/Pitch reflects movement.
        2. Turn Right: Verify Yaw Rate is Negative (or matches convention).
        """
        self.init_hw()
        print("  Running Autonomous Verification...")

        # 1. Verify Straight Drive
        print("  [Check 1] Drive Straight...")
        power = self.config.min_power_visible + 10

        samples = self.drive_and_measure(power, power, 1.0)
        time.sleep(0.5)

        if not samples:
            avg_yaw = 0.0
        else:
            avg_yaw = sum(abs(s.yaw_rate) for s in samples) / len(samples)

        print(f"    Avg Yaw Rate: {avg_yaw:.1f} deg/s")

        if avg_yaw > 40.0:
             print("  [FAILURE] Robot spun while trying to drive straight.")
             print("  -> Possible Phase mismatch or Motor Direction mismatch despite checks.")
             sys.exit(1)

        # 2. Verify Right Turn
        print("  [Check 2] Turn Right (Clockwise)...")
        # Command L+, R-

        samples = self.drive_and_measure(power, -power, 1.0)

        if not samples:
            avg_yaw_signed = 0.0
        else:
            avg_yaw_signed = sum(s.yaw_rate for s in samples) / len(samples)

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
