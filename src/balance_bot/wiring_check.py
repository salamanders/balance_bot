import time
import sys
import threading
import smbus2
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware
from .enums import Axis


class WiringCheck:
    """
    Streamlined Wiring Check & Calibration Tool.
    Combines motor identification and sensor calibration into a single flow.
    Includes pessimistic verification steps to ensure hardware is truly responsive.
    """

    def __init__(self):
        self.config = RobotConfig.load()
        self.hw = None
        # Default safe config for discovery
        self.temp_motor_l = 0
        self.temp_motor_r = 1
        self.temp_invert_l = False
        self.temp_invert_r = False

    def init_hw(self, for_calibration=False):
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

    def _check_dominance(self, data_dict, test_name):
        """
        Verify that the 'winner' axis is significantly stronger than others.
        Returns the winner key.
        """
        sorted_items = sorted(data_dict.items(), key=lambda x: abs(x[1]), reverse=True)
        winner, winner_val = sorted_items[0]
        runner, runner_val = sorted_items[1]

        ratio = abs(winner_val) / (abs(runner_val) + 1e-9)
        print(f"   [Analysis] {test_name}: Winner={winner.upper()} ({abs(winner_val):.2f}) vs Runner={runner.upper()} ({abs(runner_val):.2f}) -> Ratio: {ratio:.1f}")

        if ratio < 1.5:
            print(f"   [WARNING] Ambiguous Result for {test_name}! The detected axis is not significantly stronger than others.")
            print("   Please check mounting or perform the test more carefully.")
            input("   Press Enter to acknowledge and continue (or Ctrl+C to abort)...")
        else:
            print(f"   [PASS] Strong signal for {test_name}.")

        return winner

    def detect_i2c_buses(self):
        """
        Auto-detect I2C buses with Pessimistic Verification.
        - Pulses motors to confirm Motor Driver.
        - Reads live accel data to confirm IMU.
        """
        print("Detecting I2C Buses...")
        candidates = [1, 3, 0, 2] # User hint: Motor on 1, Gyro on 3

        # 1. Find Motor Driver
        found_motor = None
        print("Scanning for PiconZero (0x22)...")
        for bus_id in candidates:
            try:
                with smbus2.SMBus(bus_id) as bus:
                    try:
                        # Try to read revision (Reg 0) from address 0x22
                        bus.read_word_data(0x22, 0)
                        print(f"-> Found PiconZero candidate on Bus {bus_id}")

                        # PESSIMISM: Pulse Motor
                        print("   Pulsing Motor A to verify...")
                        # Write [30, 0] to Reg 0 (Motor A=30, Motor B=0)
                        bus.write_i2c_block_data(0x22, 0, [30, 0])
                        time.sleep(0.3)
                        # Stop [0, 0]
                        bus.write_i2c_block_data(0x22, 0, [0, 0])

                        ans = input(f"   Did the robot move/click on Bus {bus_id}? [y/n]: ").strip().lower()
                        if ans == 'y':
                            found_motor = bus_id
                            print(f"   [CONFIRMED] Motor on Bus {bus_id}")
                            break
                        else:
                            print(f"   [REJECTED] User denied movement on Bus {bus_id}.")

                    except OSError:
                        pass
            except (OSError, FileNotFoundError):
                pass

        if found_motor is not None:
            self.config.motor_i2c_bus = found_motor
        else:
            print(f"Warning: Could not detect PiconZero (or user rejected all candidates). Keeping: {self.config.motor_i2c_bus}")
            # We don't exit here, allowing user to potentially force it later or retry?
            # But usually this is fatal.
            ans = input("   Continue anyway? [y/n]: ").lower()
            if ans != 'y': sys.exit(1)

        # 2. Find IMU
        found_imu = None
        print("Scanning for MPU6050 (0x68)...")
        for bus_id in candidates:
            try:
                with smbus2.SMBus(bus_id) as bus:
                    try:
                        # Try to read WHO_AM_I (Reg 0x75)
                        who_am_i = bus.read_byte_data(0x68, 0x75)
                        if who_am_i != 0x68:
                             continue

                        print(f"-> Found MPU6050 candidate on Bus {bus_id} (WHO_AM_I=0x{who_am_i:02X})")

                        # PESSIMISM: Read Live Accel Data
                        # IMPORTANT: MPU6050 starts in SLEEP mode. We must wake it up to get data.
                        # Reg 0x6B (PWR_MGMT_1) = 0
                        try:
                            bus.write_byte_data(0x68, 0x6B, 0)
                            time.sleep(0.1)
                        except OSError:
                            print(f"   [WARNING] Failed to wake MPU6050 on Bus {bus_id}")
                            continue

                        # Reg 0x3B (ACCEL_XOUT_H) -> 6 bytes
                        data = bus.read_i2c_block_data(0x68, 0x3B, 6)

                        def to_signed(h, l):
                            val = (h << 8) | l
                            if val >= 32768: val -= 65536
                            return val

                        ax = to_signed(data[0], data[1])
                        ay = to_signed(data[2], data[3])
                        az = to_signed(data[4], data[5])

                        print(f"   Live Reading: Ax={ax}, Ay={ay}, Az={az}")

                        # Check for gravity or noise
                        mag = (ax**2 + ay**2 + az**2)**0.5
                        if mag > 500: # 1g is ~16384 usually, but even if scale is different, >500 is likely valid data
                            found_imu = bus_id
                            print(f"   [CONFIRMED] IMU data looks valid (Magnitude {mag:.0f})")
                            break
                        else:
                            print(f"   [WARNING] IMU data extremely low? Magnitude {mag:.0f}. Skipping.")

                    except OSError:
                        pass
            except (OSError, FileNotFoundError):
                pass

        if found_imu is not None:
            self.config.imu_i2c_bus = found_imu
        else:
            print(f"Warning: Could not detect MPU6050. Using default: {self.config.imu_i2c_bus}")

    def run(self):
        print("\n=== Robot Auto-Setup Wizard (Pessimistic Mode) ===")
        print("We will configure Motors, then Sensors.")

        # Step 0: Auto-Detect Bus
        self.detect_i2c_buses()

        print("Please ensure the robot is on a STAND or wheels are lifted.")
        input("Press Enter to BEGIN...")

        # --- Phase 1: Motors ---
        self.setup_motors()

        # --- Phase 2: Sensors ---
        print("\n[Phase 2] Sensor Calibration")
        print("Please place the robot on the FLOOR.")
        print("Rest it on its BACK training wheel (Leaning Back).")
        input("Press Enter when ready...")

        self.setup_sensors()

        # --- Final Save ---
        print("\n[SUCCESS] Configuration Complete!")
        self.config.save()
        print("Settings saved to disk.")
        self.cleanup()

    def setup_motors(self):
        """Identify Motor Channels and Directions."""
        print("\n>>> Motor Identification")
        # We start by assuming Ch0 is 'Motor A' and Ch1 is 'Motor B'.
        # We drive Ch0 and ask what moved.

        # Initialize with raw mapping 0->L, 1->R (arbitrary)
        self.temp_motor_l = 0
        self.temp_motor_r = 1
        self.temp_invert_l = False
        self.temp_invert_r = False
        self.init_hw()

        # Step 1: Run Channel 0 (Currently mapped to Left)
        print("\nRunning 'Channel A' (0) for 0.5s...")
        self.hw.set_motors(30, 0) # Left=30, Right=0
        time.sleep(0.5)
        self.hw.stop()

        print("Which wheel ran, and in which direction?")
        print("a) LEFT Forward")
        print("b) LEFT Backward")
        print("c) RIGHT Forward")
        print("d) RIGHT Backward")
        print("e) None / I didn't see")

        choice = input("Select (a/b/c/d/e): ").strip().lower()

        # Deduce Config
        if choice == 'a':
            self.config.motor_l = 0
            self.config.motor_l_invert = False
            self.config.motor_r = 1
            first_motor = "Left"
        elif choice == 'b':
            self.config.motor_l = 0
            self.config.motor_l_invert = True
            self.config.motor_r = 1
            first_motor = "Left"
        elif choice == 'c':
            self.config.motor_r = 0
            self.config.motor_r_invert = False
            self.config.motor_l = 1
            first_motor = "Right"
        elif choice == 'd':
            self.config.motor_r = 0
            self.config.motor_r_invert = True
            self.config.motor_l = 1
            first_motor = "Right"
        else:
            print("Test inconclusive. Please check battery/connections and try again.")
            sys.exit(1)

        # Update temp config to match reality so far
        self.temp_motor_l = self.config.motor_l
        self.temp_motor_r = self.config.motor_r
        self.temp_invert_l = self.config.motor_l_invert
        self.temp_invert_r = False # Reset Right/Other invert for testing

        # Step 2: Run the OTHER motor
        other_motor = "Right" if first_motor == "Left" else "Left"
        print(f"\nNow running the {other_motor.upper()} wheel (Channel 1)...")

        # Reload HW
        self.init_hw()

        if other_motor == "Right":
            self.hw.set_motors(0, 30)
        else:
            self.hw.set_motors(30, 0)

        time.sleep(0.5)
        self.hw.stop()

        print(f"Did the {other_motor.upper()} wheel spin Forward or Backward?")
        print("f) Forward")
        print("b) Backward")

        ans = input("Select (f/b): ").strip().lower()

        is_inverted = (ans == 'b')

        if other_motor == "Right":
            self.config.motor_r_invert = is_inverted
        else:
            self.config.motor_l_invert = is_inverted

        # Update Temp for Phase 2
        self.temp_invert_l = self.config.motor_l_invert
        self.temp_invert_r = self.config.motor_r_invert
        self.init_hw()

        print(f"-> Motors Configured: L={self.config.motor_l}(Inv:{self.config.motor_l_invert}), R={self.config.motor_r}(Inv:{self.config.motor_r_invert})")

    def setup_sensors(self):
        """Calibrate Gyro and Accelerometer Axes."""

        # --- 1. Static (Vertical Axis) ---
        print("\n[Step 1/4] Detecting Gravity (Vertical Axis)...")
        print("Reading for 1 second (Stay Still)...")

        accel_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        samples = 50
        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            for k in a:
                accel_sum[k] += a[k]
            time.sleep(0.02)

        avg_accel = {k: v/samples for k,v in accel_sum.items()}

        # Dominance Check
        vert_axis = self._check_dominance(avg_accel, "Vertical Axis")
        vert_invert = avg_accel[vert_axis] > 0

        self.config.accel_vertical_axis = Axis(vert_axis)
        self.config.accel_vertical_invert = vert_invert
        print(f"-> Vertical Axis: {vert_axis.upper()} (Inv: {vert_invert})")

        # --- 2. Drive Forward (Forward Axis) ---
        print("\n[Step 2/4] Detecting Forward Motion (Forward Axis)...")
        print("Make sure the robot has space (moving ~1 ft).")
        input("Press Enter to DRIVE FORWARD...")

        print("Driving...")
        # Start Driving
        self.hw.set_motors(30, 30)

        accel_data = []
        end_time = time.time() + 1.0
        while time.time() < end_time:
            a, _ = self.hw.read_imu_raw()
            accel_data.append(a)
            time.sleep(0.01)

        self.hw.stop()

        # Analyze: Axis with highest variance or shift from static
        candidates = [k for k in ["x", "y", "z"] if k != vert_axis]
        shifts = {}
        for k in candidates:
            mean_move = sum(d[k] for d in accel_data) / len(accel_data)
            shifts[k] = mean_move - avg_accel[k]

        # Dominance Check
        fwd_axis = self._check_dominance(shifts, "Forward Axis")
        fwd_invert = shifts[fwd_axis] < 0

        self.config.accel_forward_axis = Axis(fwd_axis)
        self.config.accel_forward_invert = fwd_invert
        print(f"-> Forward Axis: {fwd_axis.upper()} (Inv: {fwd_invert})")

        # --- 3. Spin Right (Yaw Axis) ---
        print("\n[Step 3/4] Detecting Rotation (Yaw Axis)...")
        input("Press Enter to SPIN RIGHT...")

        print("Spinning...")
        self.hw.set_motors(30, -30) # Spin Right

        gyro_data = []
        end_time = time.time() + 1.0
        while time.time() < end_time:
            _, g = self.hw.read_imu_raw()
            gyro_data.append(g)
            time.sleep(0.01)

        self.hw.stop()

        # Analyze: Axis with highest absolute mean rate
        avg_rates = {k: abs(sum(d[k] for d in gyro_data)/len(gyro_data)) for k in ["x","y","z"]}

        # Dominance Check
        yaw_axis = self._check_dominance(avg_rates, "Yaw Axis")

        # Polarity: Right Turn = Positive Rate
        raw_mean = sum(d[yaw_axis] for d in gyro_data) / len(gyro_data)
        yaw_invert = raw_mean < 0

        self.config.gyro_yaw_axis = Axis(yaw_axis)
        self.config.gyro_yaw_invert = yaw_invert
        print(f"-> Yaw Axis: {yaw_axis.upper()} (Inv: {yaw_invert})")

        # --- 4. Manual Tip (Pitch Axis) ---
        print("\n[Step 4/4] Detecting Tilt (Pitch Axis)...")
        print("Preparation: Robot is on Back Training Wheel.")
        print("Task: Tip it FORWARD to rest on Front Training Wheel.")
        print("I will record while you do this.")
        input("Press Enter to START RECORDING. (Press Enter again when done tipping)...")

        print("Recording... (Tip the robot now!)")

        # Threaded recording
        self.recording = True
        self.tip_data = []

        def record_loop():
            while self.recording:
                _, g = self.hw.read_imu_raw()
                self.tip_data.append(g)
                time.sleep(0.01)

        t = threading.Thread(target=record_loop)
        t.start()

        input("Press Enter when DONE tipping...")
        self.recording = False
        t.join()

        # Analyze: Axis with highest Integral
        integrals = {"x": 0.0, "y": 0.0, "z": 0.0}
        for sample in self.tip_data:
            for k in integrals:
                integrals[k] += sample[k] * 0.01 # approx dt

        # Filter out Yaw Axis
        # But we pass all to dominance check for transparency
        abs_integrals = {k: abs(v) for k,v in integrals.items()}

        # Dominance Check
        pitch_axis = self._check_dominance(abs_integrals, "Pitch Axis")

        raw_integral = integrals[pitch_axis]
        pitch_invert = raw_integral < 0

        self.config.gyro_pitch_axis = Axis(pitch_axis)
        self.config.gyro_pitch_invert = pitch_invert
        print(f"-> Pitch Axis: {pitch_axis.upper()} (Inv: {pitch_invert})")

        # --- 5. Deduce Roll ---
        all_axes = {"x", "y", "z"}
        roll_gyro = list(all_axes - {yaw_axis, pitch_axis})[0]
        self.config.gyro_roll_axis = Axis(roll_gyro)
        self.config.gyro_roll_invert = False # Default

        print(f"-> Roll Axis (Delineated): {roll_gyro.upper()}")

        # Update Config Object
        self.config.gyro_yaw_axis = Axis(yaw_axis)

    def cleanup(self):
        if self.hw:
            self.hw.stop()
            self.hw.cleanup()

if __name__ == "__main__":
    try:
        WiringCheck().run()
    except KeyboardInterrupt:
        print("\nExiting...")
        sys.exit(0)
