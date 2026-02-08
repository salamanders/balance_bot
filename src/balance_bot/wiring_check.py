import time
import sys
import threading
import smbus
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware
from .enums import Axis, Orientation
from .utils import analyze_dominance, to_signed


class WiringCheck:
    """
    Streamlined Wiring Check & Calibration Tool.
    """

    def __init__(self):
        self.config = RobotConfig.load()
        self.hw = None
        # Default safe config for discovery
        self.temp_motor_l = 0
        self.temp_motor_r = 1
        self.temp_invert_l = False
        self.temp_invert_r = False

    def init_hw(self):
        """Initialize hardware with current known config."""
        if self.hw:
            self.hw.stop()
            self.hw.cleanup()

        self.hw = RobotHardware(
            motor_l=self.temp_motor_l,
            motor_r=self.temp_motor_r,
            invert_l=self.temp_invert_l,
            invert_r=self.temp_invert_r,
            # Sensors
            gyro_axis=self.config.gyro_pitch_axis,
            gyro_invert=self.config.gyro_pitch_invert,
            gyro_yaw_axis=self.config.gyro_yaw_axis,
            gyro_yaw_invert=self.config.gyro_yaw_invert,
            gyro_roll_axis=self.config.gyro_roll_axis,
            gyro_roll_invert=self.config.gyro_roll_invert,
            accel_vertical_axis=self.config.accel_vertical_axis,
            accel_vertical_invert=self.config.accel_vertical_invert,
            accel_forward_axis=self.config.accel_forward_axis,
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
        print("\n=== Robot Auto-Setup Wizard ===")
        print("Steps: Bus -> Motors -> Sensors -> Dynamics (Flop)")

        # 1. Bus Detection
        self.detect_i2c_buses()
        input("Press Enter to continue...")

        # 2. Motors
        print("\n--- Phase 1: Motors ---")
        print("Please ensure the robot is on a STAND or wheels are lifted.")
        input("Press Enter to BEGIN MOTORS...")
        self.setup_motors()

        # 3. Sensors & Dynamics
        print("\n--- Phase 2: Sensors & Dynamics ---")
        print("Please place the robot on the FLOOR.")
        print("Rest it on its BACK training wheel (Leaning Back).")
        input("Press Enter when ready...")

        # We need to initialize HW with the motors we just found
        self.init_hw()

        # Prerequisite: Identify Gravity/Pitch axes before we can do dynamic moves
        self.setup_static_sensors()

        # Now do the dynamic checks (Steps 7-14)
        self.verify_straight_motion() # Steps 7-8
        self.verify_yaw_turns()       # Steps 9-10
        self.test_flops()             # Steps 11-14

        # 4. Save
        print("\n[SUCCESS] Configuration Complete!")
        self.config.save()
        print("Settings saved to disk.")
        self.cleanup()

    # --- Step 1: Bus ---
    def detect_i2c_buses(self):
        print("Detecting I2C Buses...")
        candidates = [1, 3, 0, 2]

        # Motors
        found_motor = None
        for bus_id in candidates:
            try:
                bus = smbus.SMBus(bus_id)
                try:
                    bus.read_word_data(0x22, 0)
                    print(f"-> Found PiconZero (Motors) on Bus {bus_id}")
                    found_motor = bus_id
                    break
                except OSError:
                    pass
                finally:
                    bus.close()
            except Exception:
                pass

        if found_motor is not None:
            self.config.motor_i2c_bus = found_motor
        else:
            print("Warning: PiconZero not found.")

        # IMU
        found_imu = None
        for bus_id in candidates:
            try:
                bus = smbus.SMBus(bus_id)
                try:
                    who = bus.read_byte_data(0x68, 0x75)
                    if who == 0x68:
                        print(f"-> Found MPU6050 (IMU) on Bus {bus_id}")
                        found_imu = bus_id
                        break
                except OSError:
                    pass
                finally:
                    bus.close()
            except Exception:
                pass

        if found_imu is not None:
            self.config.imu_i2c_bus = found_imu
        else:
            print("Warning: MPU6050 not found.")

    # --- Step 2: Motors ---
    def setup_motors(self):
        print("\n>>> Motor Identification")
        self.temp_motor_l = 0
        self.temp_motor_r = 1
        self.temp_invert_l = False
        self.temp_invert_r = False
        self.init_hw()

        # 1. Find Minimum Power
        print("finding minimum power...")
        power = 20
        found_power = False
        while power <= 120:
            print(f"Twitching BOTH motors at Power {power}...")
            self.hw.set_motors(power, power)
            time.sleep(0.5)
            self.hw.stop()

            ans = input("Did the robot move? [y/n]: ").strip().lower()
            if ans == 'y':
                self.config.min_power_visible = power
                found_power = True
                print(f"-> Minimum Power set to {power}")
                break
            else:
                power += 20

        if not found_power:
            print("Failed to move even at max power. Check battery/wiring.")
            sys.exit(1)

        # 2. Check Motor A (Channel 0)
        print("\nTwitching Motor A (Channel 0)...")
        self.hw.set_motors(self.config.min_power_visible, 0)
        time.sleep(0.5)
        self.hw.stop()

        print("Which wheel ran, and in which direction?")
        print("a) LEFT Forward")
        print("b) LEFT Backward")
        print("c) RIGHT Forward")
        print("d) RIGHT Backward")
        choice = input("Select (a/b/c/d): ").strip().lower()

        if choice == 'a':
            self.config.motor_l = 0; self.config.motor_l_invert = False; self.config.motor_r = 1
        elif choice == 'b':
            self.config.motor_l = 0; self.config.motor_l_invert = True; self.config.motor_r = 1
        elif choice == 'c':
            self.config.motor_r = 0; self.config.motor_r_invert = False; self.config.motor_l = 1
        elif choice == 'd':
            self.config.motor_r = 0; self.config.motor_r_invert = True; self.config.motor_l = 1
        else:
            print("Invalid selection.")
            sys.exit(1)

        # Update temp
        self.temp_motor_l = self.config.motor_l
        self.temp_motor_r = self.config.motor_r
        self.temp_invert_l = self.config.motor_l_invert
        self.temp_invert_r = False # Unknown
        self.init_hw()

        # Check Motor B
        other = "Right" if choice in ['a','b'] else "Left"
        print(f"\nTwitching {other} Motor (Channel 1)...")
        if other == "Right":
            self.hw.set_motors(0, self.config.min_power_visible)
        else:
            self.hw.set_motors(self.config.min_power_visible, 0)

        time.sleep(0.5)
        self.hw.stop()

        ans = input(f"Did {other} spin Forward? [y/n] (n=Backward): ").strip().lower()
        is_inverted = (ans != 'y')

        if other == "Right":
            self.config.motor_r_invert = is_inverted
        else:
            self.config.motor_l_invert = is_inverted

        self.temp_invert_l = self.config.motor_l_invert
        self.temp_invert_r = self.config.motor_r_invert
        self.init_hw()
        print(f"-> Motors Configured: L={self.config.motor_l}, R={self.config.motor_r}")

    # --- Prerequisite: Static Sensors ---
    def setup_static_sensors(self):
        """Identify Vertical (Gravity) and Pitch Axis (via Tip)."""
        print("\n>>> Static Sensor Calibration")

        # 1. Vertical (Gravity)
        print("Detecting Gravity (Vertical Axis)... Keep Still.")
        time.sleep(0.5)
        accel_sum = {"x": 0, "y": 0, "z": 0}
        for _ in range(50):
            a, _ = self.hw.read_imu_raw()
            for k in a: accel_sum[k] += a[k]
            time.sleep(0.01)

        avg = {k: v/50 for k,v in accel_sum.items()}
        vert, _, _ = analyze_dominance(avg, "Vertical")
        self.config.accel_vertical_axis = Axis(vert)
        self.config.accel_vertical_invert = avg[vert] > 0
        print(f"-> Vertical Axis: {vert.upper()} (Inv: {self.config.accel_vertical_invert})")

        # 2. Pitch (Tip)
        print("\nDetecting Pitch Axis. Robot is on BACK wheel.")
        input("Press Enter, then immediately TIP ROBOT FORWARD to Front Wheel...")
        print("Recording Tip...")

        gyro_data = []
        for _ in range(100): # 1 second
            _, g = self.hw.read_imu_raw()
            gyro_data.append(g)
            time.sleep(0.01)

        print("Done.")
        # Integrate
        integrals = {"x": 0, "y": 0, "z": 0}
        for g in gyro_data:
            for k in integrals: integrals[k] += g[k]

        abs_ints = {k: abs(v) for k,v in integrals.items()}
        pitch_axis, _, _ = analyze_dominance(abs_ints, "Pitch")

        # Polarity: We tipped Forward (Positive Pitch change in standard NED, but usually we define Nose Down as positive?)
        # Let's check Utils. 'Nose Down (leaning forward) corresponds to a Positive Pitch angle.'
        # So Tipping Back->Front is a POSITIVE change.
        # We need the integrated value to be POSITIVE.
        raw_int = integrals[pitch_axis]
        pitch_inv = raw_int < 0

        self.config.gyro_pitch_axis = Axis(pitch_axis)
        self.config.gyro_pitch_invert = pitch_inv
        print(f"-> Pitch Axis: {pitch_axis.upper()} (Inv: {pitch_inv})")

        # Reload HW with new sensor config
        self.init_hw()

    # --- Step 7-8: Straight Motion ---
    def verify_straight_motion(self):
        print("\n>>> Verifying Straight Motion (Steps 7-8)")

        # 1. Measure Static Baseline first
        print("Measuring baseline...")
        static_sum = {'x':0.0, 'y':0.0, 'z':0.0}
        for _ in range(20):
            a, _ = self.hw.read_imu_raw()
            for k in a: static_sum[k] += a[k]
            time.sleep(0.01)
        static_avg = {k: v/20 for k,v in static_sum.items()}

        input("Press Enter to run Straight Forward (1s) then Backward (1s)...")

        # Forward
        print("Forward...")
        self.hw.set_motors(60, 60)

        # Record Accel for Forward Axis
        accel_data = []
        start = time.time()
        while time.time() - start < 1.0:
            a, _ = self.hw.read_imu_raw()
            accel_data.append(a)
            time.sleep(0.01)

        # Backward
        print("Backward...")
        self.hw.set_motors(-60, -60)
        time.sleep(1.0)
        self.hw.stop()

        ans = input("Did it move Forward then Backward? [y/n]: ").lower()
        if ans != 'y':
            print("Exit: Movement failed or confused.")
            sys.exit(1)

        # Deduce Forward Axis
        moving_avg = {k: sum(d[k] for d in accel_data)/len(accel_data) for k in ['x','y','z']}

        # Calculate Shift (Delta)
        deltas = {k: moving_avg[k] - static_avg[k] for k in ['x','y','z']}

        # Use Delta for Dominance
        fwd, _, _ = analyze_dominance(deltas, "Forward", exclude=[self.config.accel_vertical_axis.value])

        # Invert check:
        # When accelerating Forward (+Velocity), the accelerometer (mass) feels a force BACKWARD relative to the sensor frame?
        # F = ma. Sensor accelerates Forward. Mass resists. Mass presses against Back wall of sensor.
        # This is usually registered as NEGATIVE acceleration on the Forward Axis?
        # Or Positive?
        # Actually, standard convention: +1g on Z when sitting flat.
        # Let's trust the sign of the delta.
        # If delta is POSITIVE, and we assume standard mapping, then Invert=False.
        # If delta is NEGATIVE, Invert=True?
        # Let's assume Delta matches Axis Direction.
        self.config.accel_forward_axis = Axis(fwd)
        self.config.accel_forward_invert = deltas[fwd] < 0
        print(f"-> Forward Axis: {fwd.upper()} (Inv: {self.config.accel_forward_invert})")

    # --- Step 9-10: Yaw Turns ---
    def verify_yaw_turns(self):
        print("\n>>> Verifying Yaw (Steps 9-10)")

        # Left 360
        input("Press Enter to turn LEFT 360...")
        self.hw.set_motors(-60, 60) # Left Turn

        gyro_data = []
        start = time.time()
        while time.time() - start < 1.5: # Guess time
            _, g = self.hw.read_imu_raw()
            gyro_data.append(g)
            time.sleep(0.01)
        self.hw.stop()

        input("Did it turn Left approx 360? [Enter=Yes, Ctrl+C=No]")

        # Analyze Yaw Axis
        avgs = {k: sum(d[k] for d in gyro_data)/len(gyro_data) for k in ['x','y','z']}
        yaw, _, _ = analyze_dominance({k: abs(v) for k,v in avgs.items()}, "Yaw")

        # Left Turn = Negative Yaw Rate usually?
        # Standard: Right is Positive. Left is Negative.
        val = avgs[yaw]
        yaw_inv = val > 0 # If val is pos during left turn, we must invert to make it neg

        self.config.gyro_yaw_axis = Axis(yaw)
        self.config.gyro_yaw_invert = yaw_inv
        print(f"-> Yaw Axis: {yaw.upper()} (Inv: {yaw_inv})")

        # Deduce Roll (Remaining)
        all_axes = {'x','y','z'}
        roll = list(all_axes - {self.config.gyro_pitch_axis.value, yaw})[0]
        self.config.gyro_roll_axis = Axis(roll)
        print(f"-> Roll Axis: {roll.upper()}")

        # Reload HW
        self.init_hw()

        # Right 360
        input("Press Enter to turn RIGHT 360 (Verification)...")
        self.hw.set_motors(60, -60)
        time.sleep(1.5)
        self.hw.stop()
        input("Did it turn Right? [Enter]")

    # --- Steps 11-14: Flop Tests ---
    def test_flops(self):
        print("\n>>> Dynamic Flop Tests (Steps 11-14)")

        # 1. Forward Flop (Back -> Front)
        print("\n[Test 1] Forward Flop (Back -> Front)")
        print("Ensure robot is resting on BACK wheel.")
        input("Press Enter to start...")

        # Verify position
        curr_pitch, _ = self.hw.read_imu_converted()
        if curr_pitch > -10:
             print(f"Warning: Pitch is {curr_pitch:.1f}. Should be < -10 (Leaning Back).")
             input("Fix position and Press Enter...")

        # Loop
        power = 60
        found_power = None
        while power <= 100:
            print(f"Trying 'Reverse' Kick with Power {power}...")
            # Drive Forward 1s (Setup)
            self.hw.set_motors(self.config.min_power_visible, self.config.min_power_visible)
            time.sleep(0.5)
            # Hard Reverse (Kick)
            self.hw.set_motors(-power, -power)
            time.sleep(0.6)
            self.hw.stop()

            time.sleep(1.0) # Wait for flop

            # Check Pitch
            p, _ = self.hw.read_imu_converted()
            if p > 10: # Flopped to Front
                print(f"-> Success! Flopped Forward at Power {power}.")
                found_power = power
                break
            else:
                print(f"-> Failed (Pitch {p:.1f}). Increasing power...")
                power += 10
                # Reset to Back?
                print("Please reset robot to BACK wheel if needed.")
                input("Press Enter...")

        if found_power:
            self.config.control.kickup_power_forward = found_power

        # 2. Backward Flop (Front -> Back)
        print("\n[Test 2] Backward Flop (Front -> Back)")
        print("Ensure robot is resting on FRONT wheel.")
        input("Press Enter to start...")

        curr_pitch, _ = self.hw.read_imu_converted()
        if curr_pitch < 10:
             print(f"Warning: Pitch is {curr_pitch:.1f}. Should be > 10 (Leaning Front).")
             input("Fix position and Press Enter...")

        power = 60
        found_power = None
        while power <= 100:
            print(f"Trying 'Forward' Kick with Power {power}...")
            # Drive Backward 1s
            self.hw.set_motors(-self.config.min_power_visible, -self.config.min_power_visible)
            time.sleep(0.5)
            # Hard Forward
            self.hw.set_motors(power, power)
            time.sleep(0.6)
            self.hw.stop()

            time.sleep(1.0)

            p, _ = self.hw.read_imu_converted()
            if p < -10: # Flopped to Back
                print(f"-> Success! Flopped Backward at Power {power}.")
                found_power = power
                break
            else:
                print(f"-> Failed (Pitch {p:.1f}). Increasing power...")
                power += 10
                input("Reset to FRONT wheel if needed, then Enter...")

        if found_power:
            self.config.control.kickup_power_backward = found_power

if __name__ == "__main__":
    WiringCheck().run()
