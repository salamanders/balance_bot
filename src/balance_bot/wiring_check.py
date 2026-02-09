import time
import sys
import smbus
from .diagnostics import run_diagnostics
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware
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

            print("\n[SUCCESS] Self-Discovery Complete. I know everything.")
            print(f"Summary:")
            print(f"  Buses: Motor={c.motor_i2c_bus}, IMU={c.imu_i2c_bus}")
            print(f"  Axes: Vert={c.accel_vertical_axis}, Fwd={c.accel_forward_axis}, Pitch={c.gyro_pitch_axis}")
            print(f"  Motors: MinPower={c.min_power_visible}, L={c.motor_l}(Inv={c.motor_l_invert}), R={c.motor_r}(Inv={c.motor_r_invert})")
            print(f"  KickUp: Fwd={c.control.kickup_power_forward:.1f}, Bwd={c.control.kickup_power_backward:.1f}")
            break

        self.cleanup()

    # --- Tier 1: Hardware Connectivity ---
    def discover_buses(self):
        """
        Scan I2C buses [0, 1, 2, 3] for PiconZero (0x22) and MPU6050 (0x68).
        """
        print("Scanning I2C Buses...")
        candidates = [1, 3, 0, 2]

        # 1. Find Motors (0x22)
        found_motor = None
        for bus_id in candidates:
            try:
                bus = smbus.SMBus(bus_id)
                try:
                    # Try to read a byte from PiconZero
                    bus.read_byte_data(0x22, 0)
                    print(f"  [FOUND] PiconZero (Motors) on Bus {bus_id}")
                    found_motor = bus_id
                    bus.close()
                    break
                except OSError:
                    pass
                finally:
                    try:
                        bus.close()
                    except:
                        pass
            except Exception:
                pass

        if found_motor is None:
            print("  [FAILURE] Could not find PiconZero (0x22) on any bus.")
            sys.exit(1)

        self.config.motor_i2c_bus = found_motor

        # 2. Find IMU (0x68)
        found_imu = None
        for bus_id in candidates:
            try:
                bus = smbus.SMBus(bus_id)
                try:
                    # Read WHO_AM_I register
                    who = bus.read_byte_data(0x68, 0x75)
                    if who == 0x68:
                        print(f"  [FOUND] MPU6050 (IMU) on Bus {bus_id}")
                        found_imu = bus_id
                        bus.close()
                        break
                except OSError:
                    pass
                finally:
                    try:
                        bus.close()
                    except:
                        pass
            except Exception:
                pass

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
        # Check sign: Gravity pulls Down. Accelerometer measures Normal Force (Up).
        # Standard: +1g on Z means Z is Up.
        # We want Vertical Axis to be positive UP.
        self.config.accel_vertical_invert = avg_back[vert] < 0 # If reading is negative, we invert to make it positive?
        # Wait, if Z reads -1g (upside down), then we invert so Up is positive.
        # Actually, typically we align it so +Vertical is "Up".
        # If robot is sitting flat-ish, Z should be +1g or -1g.
        # If it is +1g, Invert=False. If -1g, Invert=True.
        # But wait, the robot is leaning. Vertical component is still dominant.
        self.config.accel_vertical_invert = avg_back[vert] < 0
        print(f"  -> Vertical Axis: {vert.upper()} (Invert: {self.config.accel_vertical_invert})")

        # 2. Read Lean (Forward Axis) - Deduce from residue?
        # Actually, spec says: "The axis with the next largest value is Forward".
        # Because we are leaning, gravity leaks into Forward axis.
        # We sort by absolute value.
        sorted_axes = sorted(avg_back.items(), key=lambda x: abs(x[1]), reverse=True)
        # 0 is Vertical. 1 is Forward. 2 is Pitch (Axle).
        forward_axis = sorted_axes[1][0]
        pitch_axis = sorted_axes[2][0]

        self.config.accel_forward_axis = Axis(forward_axis)
        print(f"  -> Forward Axis Candidate: {forward_axis.upper()}")
        print(f"  -> Pitch Axis Candidate: {pitch_axis.upper()}")

        # 3. Determine Pitch Axis Orientation (Using Cross Product)
        # We need a second data point: Tip Forward.
        print("\n  Now TIP ROBOT FORWARD to Front Wheel.")
        input("Press Enter to measure FRONT position...")

        accel_sum_front = {"x": 0.0, "y": 0.0, "z": 0.0}
        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            for k in a:
                accel_sum_front[k] += a[k]
            time.sleep(0.01)
        avg_front = {k: v / samples for k, v in accel_sum_front.items()}

        # Cross Product: Back x Front
        # Back vector (roughly Up + Back)
        # Front vector (roughly Up + Front)
        # The rotation axis is the Pitch Axis.
        axis_vec = cross_product(avg_back, avg_front)

        # Verify that our calculated pitch axis matches the cross product magnitude
        detected_pitch, _, _ = analyze_dominance({k: abs(v) for k, v in axis_vec.items()}, "Pitch Axis (CrossProd)")

        if detected_pitch != pitch_axis:
             print(f"  [WARNING] Cross product suggested {detected_pitch} but magnitude suggested {pitch_axis}. Trusting Cross Product.")
             pitch_axis = detected_pitch

        self.config.gyro_pitch_axis = Axis(pitch_axis)

        # Polarity: Right Hand Rule.
        # Back -> Front rotation.
        # If we rotate "Forward" (Positive Pitch usually), the gyro vector points along the axis.
        # Wait, Back -> Front is changing pitch from Negative (Leaning Back) to Positive (Leaning Front).
        # So the change is Positive.
        # The cross product `Back x Front` points in the direction of the axis of rotation for that move.
        # If `Back x Front` is Positive on the Axis, then the Axis is aligned with the rotation vector.
        # Standard Gyro: +Rotation about X is "Roll Right". +Rotation about Y is "Pitch Down"?
        # Let's simplify:
        # We defined Back->Front as "Positive Pitch Change".
        # If the gyro reading during that move is Positive, then Invert=False.
        # Here we used Accelerometer vectors.
        # Cross product direction: perpendicular to both.
        # Polarity:
        val = axis_vec[pitch_axis]
        # If CrossProd is positive, it means the rotation "vector" is positive along that axis.
        # If we want Positive Pitch to be that direction, we don't invert.
        self.config.gyro_pitch_invert = val < 0 # If negative, invert to make it positive.

        print(f"  -> Pitch Axis: {pitch_axis.upper()} (Invert: {self.config.gyro_pitch_invert})")

        # Deduce Yaw and Roll
        all_axes = {'x', 'y', 'z'}
        used = {vert, pitch_axis}
        remaining = list(all_axes - used)
        if len(remaining) == 1:
            # Usually Yaw is Z (Vertical) but here Vertical is Gravity.
            # Wait, Gyro Yaw is rotation around Vertical Axis.
            # So Gyro Yaw Axis == Accel Vertical Axis.
            self.config.gyro_yaw_axis = Axis(vert)
            # Gyro Roll Axis == Accel Forward Axis? No, Roll is around Forward.
            # So Gyro Roll Axis == Accel Forward Axis.
            self.config.gyro_roll_axis = Axis(forward_axis)
        else:
            print("  [ERROR] Axis deduction logic failed.")

        # Reload HW with new sensor config
        self.init_hw()

        # Deduce Forward Axis Inversion
        # We need the sign of the Forward Axis change between Back and Front.
        # Back: Leaning Back. Forward Axis component?
        # Front: Leaning Front. Forward Axis component?
        # Pitch increases (Back -> Front).
        # We want Forward Axis to point "Forward".
        # When leaning Front, Gravity component on Forward Axis should be... ?
        # Gravity pulls Down. If we lean Front (Nose Down), Forward Axis points somewhat Down.
        # So Gravity projects POSITIVE onto Forward Axis (if Forward is +).
        # When leaning Back (Nose Up), Forward Axis points somewhat Up.
        # Gravity projects NEGATIVE onto Forward Axis.
        # So: Value(Front) - Value(Back) should be POSITIVE.
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

            # Pulse
            try:
                self.hw.set_motors(pwm, pwm)
                # Check for motion via Gyro or Accel Noise
                start_time = time.time()
                motion_detected = False

                # Read initial
                _, g_start = self.hw.read_imu_raw()

                while time.time() - start_time < 0.2:
                    _, g = self.hw.read_imu_raw()
                    # Simple check: Is gyro rate significantly different?
                    # Or check for spike.
                    # Let's check diff from start.
                    diff = sum(abs(g[k] - g_start[k]) for k in g)
                    if diff > 10.0: # Threshold
                         motion_detected = True
                         break
                    time.sleep(0.01)

            finally:
                self.hw.stop()
                time.sleep(0.5) # Wait for stop

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
        """
        print(">>> Verifying Motor Phasing <<<")
        self.init_hw()

        # Test 1: Drive with current config
        power = self.config.min_power_visible + 10
        print(f"  Pulse Drive with PWM {power}...")

        yaw_sum = 0.0
        accel_fwd_sum = 0.0
        count = 0

        try:
            self.hw.set_motors(power, power)
            start = time.time()
            while time.time() - start < 0.5:
                imu = self.hw.read_imu_converted()
                yaw_sum += abs(imu.yaw_rate)
                # Measure change in velocity (approx by accel? No, IMU accel is noisy when moving)
                # But we can check if Yaw Rate is huge.
                count += 1
                time.sleep(0.01)
        finally:
            self.hw.stop()
            time.sleep(1.0)

        avg_yaw = yaw_sum / max(1, count)
        print(f"  -> Avg Yaw Rate: {avg_yaw:.1f} deg/s")

        # If Yaw Rate is high (> 50 deg/s?), we are probably spinning in place -> Motors Opposed.
        if avg_yaw > 40.0:
            print("  -> High Yaw Rate detected. Motors are fighting (Spinning).")
            print("  -> Inverting Right Motor logic to align phases.")
            self.config.motor_r_invert = not self.config.motor_r_invert
        else:
            print("  -> Low Yaw Rate. Motors are aligned (Straight).")

        self.config.motor_phasing_verified = True
        self.init_hw() # Apply changes

    # --- Tier 3c: Direction ---
    def determine_motor_direction(self):
        """
        Ensure Positive Power = "Stand Up" (Reduce Lean).
        "Kick Up" Test.
        """
        print(">>> Verifying Motor Direction (Kick Up Check) <<<")
        self.init_hw()

        # 1. Measure Start Pitch
        imu = self.hw.read_imu_converted()
        start_pitch = imu.pitch_angle
        print(f"  Start Pitch: {start_pitch:.1f}")

        if abs(start_pitch) < 10:
            print("  [WARNING] Robot is too upright. Please lean it over (Front or Back).")
            input("Press Enter when leaned...")
            imu = self.hw.read_imu_converted()
            start_pitch = imu.pitch_angle

        # 2. Pulse Positive
        power = self.config.min_power_visible + 20
        print(f"  Pulsing +{power} (Positive)...")

        try:
            self.hw.set_motors(power, power)
            time.sleep(0.3)
        finally:
            self.hw.stop()

        # 3. Measure End Pitch
        time.sleep(0.5) # Wait for settle? No, we want the immediate reaction.
        # Actually, if we pulse, it moves.
        # We want to know if it moved TOWARDS vertical (0) or AWAY.
        # Note: After pulse, it might fall back. We should have measured DURING the pulse?
        # Or measure the peak change.
        # Let's try measuring during pulse.

        # Retry with measurement
        print("  Retrying with measurement...")
        end_pitch = start_pitch # fallback
        try:
            self.hw.set_motors(power, power)
            t_end = time.time() + 0.3
            while time.time() < t_end:
                 imu = self.hw.read_imu_converted()
                 end_pitch = imu.pitch_angle
                 time.sleep(0.01)
        finally:
            self.hw.stop()
            time.sleep(1.0)

        print(f"  End Pitch: {end_pitch:.1f}")

        # Check Physics
        started_leaning = abs(start_pitch)
        ended_leaning = abs(end_pitch)

        if ended_leaning < started_leaning:
            print("  [SUCCESS] Robot moved towards upright (Stood Up). Direction is Correct.")
        else:
            print("  [FAILURE] Robot dug in (Leaned More). Direction is Inverted.")
            print("  -> Inverting BOTH motors to fix direction.")
            self.config.motor_l_invert = not self.config.motor_l_invert
            self.config.motor_r_invert = not self.config.motor_r_invert

            # Also check Gyro Pitch polarity
            # If we moved "Up" (pitch change opposed gravity), gyro should reflect that.
            # But we already calibrated gyro pitch axis/invert in Tier 2 based on static vectors.
            # So Gyro should be correct relative to physical world.
            # We only change Motors here.

        self.config.motor_direction_verified = True
        self.init_hw()

    # --- Tier 4: The Human Anchor ---
    def ask_human_left_right(self):
        """
        Identify Left vs Right Motor.
        Robot spins. Human confirms direction.
        """
        print(">>> Left/Right Verification <<<")
        self.init_hw()

        print("I am going to spin. Watch me.")
        input("Press Enter to Spin...")

        # Drive Ch0 Forward, Ch1 Backward
        # We don't know which is Left/Right yet.
        # Current config: motor_l=0, motor_r=1 (Defaults).
        # We use set_motors.
        # If we set Left=+Val, Right=-Val.
        # If L=Ch0, R=Ch1 => Ch0=+, Ch1=-.

        power = self.config.min_power_visible + 10
        try:
            # We command a "Right Turn" (Clockwise?): Left Fwd, Right Bwd.
            self.hw.set_motors(power, -power)
            time.sleep(1.5)
        finally:
            self.hw.stop()

        print("\nDid I spin CLOCKWISE (Right) or COUNTER-CLOCKWISE (Left)?")
        print("  [r] Clockwise (Right)")
        print("  [l] Counter-Clockwise (Left)")
        ans = input("Choice: ").strip().lower()

        if ans == 'r':
            # We commanded L+, R-. Robot spun Right.
            # So Left Motor IS Left side. Right Motor IS Right side.
            # Current mapping (L=0, R=1) is CORRECT.
            pass
        elif ans == 'l':
            # We commanded L+, R-. Robot spun Left.
            # This means the "Left" command actually moved the Right wheel forward?
            # Or Left wheel backward?
            # Wait. "Spin Left" means turning towards Left. Left Wheel Back, Right Wheel Forward.
            # We commanded L+, R-.
            # If result is Spin Left, then L+ made it go Back? No, we verified Direction in Tier 3c.
            # So L+ means "This motor pushes forward".
            # So "Left" Motor pushed forward. "Right" Motor pushed backward.
            # Result was Spin Left.
            # This implies the motor we call "Left" (Ch0) is actually on the RIGHT side of the robot?
            # (Right Fwd, Left Back => Turn Left).
            # So Ch0 is Right. Ch1 is Left.
            print("  -> Swapping Channels.")
            self.config.motor_l = 1
            self.config.motor_r = 0
        else:
            print("Invalid input. Retrying...")
            return # Will loop

        self.config.motor_channels_verified = True
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
            try:
                self.hw.set_motors(power, power)
                time.sleep(0.4)
            finally:
                self.hw.stop()

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
            try:
                self.hw.set_motors(-power, -power)
                time.sleep(0.4)
            finally:
                self.hw.stop()

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


if __name__ == "__main__":
    try:
        WiringCheck().run()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except Exception as e:
        print(f"\nCRITICAL ERROR: {e}")
        import traceback
        traceback.print_exc()
