import time
import sys
import smbus
from .diagnostics import run_diagnostics
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware, IMUReading
from .enums import Axis
from .utils import analyze_dominance, cross_product


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
        )
        self.hw.init()

    def cleanup(self):
        if self.hw:
            self.hw.stop()
            self.hw.cleanup()

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

            # --- Tier 4: The Human Anchor ---
            if not c.motor_channels_verified:
                print("-> [MISSING] Left/Right Identification.")
                self.ask_human_left_right()
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

            print("\n[SUCCESS] Self-Discovery Complete. I know everything.")
            print("Summary:")
            print(f"  Buses: Motor={c.motor_i2c_bus}, IMU={c.imu_i2c_bus}")
            print(f"  Axes: Vert={c.accel_vertical_axis}, Fwd={c.accel_forward_axis}, Pitch={c.gyro_pitch_axis}, Yaw={c.gyro_yaw_axis}")
            print(f"  Motors: MinPower={c.min_power_visible}, L={c.motor_l}(Inv={c.motor_l_invert}), R={c.motor_r}(Inv={c.motor_r_invert})")
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
        input("Press Enter when ready...")

        # 1. Read Gravity (Vertical Axis)
        print("  Measuring Gravity (Vertical)...")
        time.sleep(0.5)
        accel_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        samples = 50
        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            for k in a:
                accel_sum[k] += a[k]
            time.sleep(0.01)

        avg_back = {k: v / samples for k, v in accel_sum.items()}

        # Dominant axis in static position is Gravity (Vertical)
        vert, _, _ = analyze_dominance(avg_back, "Vertical (Gravity)")
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
            for k in a:
                accel_sum_front[k] += a[k]
            time.sleep(0.01)
        avg_front = {k: v / samples for k, v in accel_sum_front.items()}

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
        self.init_hw()

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
            power = self.config.min_power_visible + 20
            print(f"  Pulsing +{power} (Positive)...")

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
    def ask_human_left_right(self):
        """
        Identify Left vs Right Motor.
        Robot spins. Human confirms direction.
        Also infers Gyro Yaw Polarity based on human feedback.
        """
        print(">>> Left/Right Verification & Yaw Calibration <<<")

        while True:
            self.init_hw()
            print("I am going to spin. Watch me.")
            input("Press Enter to Spin...")

            # Drive Ch0 Forward, Ch1 Backward
            # Command: L+, R- (Assuming L=0, R=1)
            # We treat this as a "Right Turn Command" in our internal logic if L=Left, R=Right.
            # But physically, we just want to know what happens.

            power = self.config.min_power_visible + 15

            samples = self.drive_and_measure(power, -power, 1.5)

            if not samples:
                avg_yaw_rate = 0.0
            else:
                avg_yaw_rate = sum(s.yaw_rate for s in samples) / len(samples)

            print(f"  [Measured] Avg Yaw Rate (Internal): {avg_yaw_rate:.1f} deg/s")

            print("\nDid I spin CLOCKWISE (Right) or COUNTER-CLOCKWISE (Left)?")
            print("  [r] Clockwise (Right)")
            print("  [l] Counter-Clockwise (Left)")
            ans = input("Choice: ").strip().lower()

            if ans not in ['r', 'l']:
                print("Invalid input. Retrying...")
                continue

            # 1. Deduce Motor Channels (Left/Right)
            if ans == 'r':
                # We commanded L+, R-. Robot spun Right.
                # This matches expectation for standard wiring (Left Motor on Left, Right Motor on Right).
                print("  -> Human confirmed Spin Right.")
                print("  -> Motor Channel Mapping is CORRECT.")
                # No swap needed.
            elif ans == 'l':
                # We commanded L+, R-. Robot spun Left.
                # This implies "Left" channel is actually on Right side, or wired backwards?
                # We already verified Phasing and Direction.
                # So L+ moves Robot "Forward/Up". R- moves Robot "Back/Down".
                # If L(Right Side) moves Fwd, R(Left Side) moves Back -> Spin Left.
                # So Ch0 is Right, Ch1 is Left.
                print("  -> Human confirmed Spin Left.")
                print("  -> ACTION: Swapping Channels (Ch0 <-> Ch1).")
                self.config.motor_l, self.config.motor_r = self.config.motor_r, self.config.motor_l

                # IMPORTANT: If we swap channels, we must re-verify!
                print("  -> Re-verifying with new mapping...")
                continue

            # 2. Deduce Gyro Yaw Polarity
            # We now know the PHYSICAL direction was 'ans'.
            # Right Turn = Positive Yaw (conventionally).
            # If User says Right ('r'):
            #   Expected Yaw: POSITIVE.
            #   Measured Yaw: avg_yaw_rate.
            #   If Measured < 0: Invert Gyro Yaw.
            # If User says Left ('l'):
            #   Expected Yaw: NEGATIVE.
            #   Measured Yaw: avg_yaw_rate.
            #   If Measured > 0: Invert Gyro Yaw.

            # Note: Standard Right Hand Rule around Z-Up: Counter-Clockwise is Positive.
            # But for vehicles, "Turn Right" often implies Positive Yaw in some conventions, or Negative in others.
            # Let's standardize: Turn Right (Clockwise) is NEGATIVE Yaw (Z-Up).
            # Turn Left (CCW) is POSITIVE Yaw.

            print("  -> Calibrating Gyro Yaw Polarity...")
            # Let's assume Z-Up.
            expected_sign = 1.0 if ans == 'l' else -1.0

            if (avg_yaw_rate * expected_sign) < 0:
                print(f"  -> Measured {avg_yaw_rate:.1f} opposes expected direction.")
                print("  -> ACTION: Inverting Gyro Yaw.")
                self.config.gyro_yaw_invert = not self.config.gyro_yaw_invert
            else:
                print("  -> Gyro Yaw polarity is correct.")

            self.config.motor_channels_verified = True
            break

        self.init_hw()

    # --- Tier 5: Dynamics ---
    def find_flop_thresholds(self):
        """
        Discover Kick-Up Power for both directions.
        """
        print(">>> Dynamic Kick-Up Calibration <<<")
        self.init_hw()

        # 1. Forward Flop (Back -> Front)
        print("\n[Test 1] Kick-Up from BACK (Forward Flop)")
        print("Place robot on BACK wheel.")
        input("Press Enter...")

        curr = self.hw.read_imu_converted()
        if curr.pitch_angle > -10:
             print(f"  Warning: Pitch {curr.pitch_angle:.1f} is not Back enough.")

        power = self.config.min_power_visible + 10
        found_fwd = None

        while power <= 100:
            print(f"  Trying Power {power}...")
            # Drive Forward to kick Back->Front
            self.drive_and_measure(power, power, 0.4)

            time.sleep(1.0)
            c = self.hw.read_imu_converted()
            if c.pitch_angle > 10: # Flopped to Front
                print(f"  [SUCCESS] Flopped at {power}.")
                found_fwd = power
                break

            power += 5
            print("  Reset to Back...")
            input("Enter...")

        if found_fwd:
            self.config.control.kickup_power_forward = found_fwd

        # 2. Backward Flop (Front -> Back)
        print("\n[Test 2] Kick-Up from FRONT (Backward Flop)")
        print("Place robot on FRONT wheel.")
        input("Press Enter...")

        power = self.config.min_power_visible + 10
        found_bwd = None

        while power <= 100:
            print(f"  Trying Power {power}...")
            # Drive Backward to kick Front->Back
            self.drive_and_measure(-power, -power, 0.4)

            time.sleep(1.0)
            c = self.hw.read_imu_converted()
            if c.pitch_angle < -10: # Flopped to Back
                print(f"  [SUCCESS] Flopped at {power}.")
                found_bwd = power
                break

            power += 5
            print("  Reset to Front...")
            input("Enter...")

        if found_bwd:
            self.config.control.kickup_power_backward = found_bwd

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
