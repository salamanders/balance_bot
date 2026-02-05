import time
import sys
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware
from .enums import MotorSide, Axis


class WiringCheck:
    """
    Interactive Tool for verifying robot wiring and sensor orientation.
    """

    def __init__(self):
        self.config = RobotConfig.load()
        # Initialize hardware using current config
        self.init_hw()
        print("\n=== Wiring Check & Calibration Tool ===")
        print("!!! IMPORTANT !!!")
        print("Ensure the robot is on a STAND or wheels are lifted off the ground.")
        print("The motors WILL spin during testing.")
        print("!!! IMPORTANT !!!")

    def init_hw(self):
        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
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
            i2c_bus=self.config.i2c_bus,
        )
        self.hw.init()

    def run(self):
        """Main Menu Loop."""
        while True:
            print("\n--- Main Menu ---")
            print(
                f"1. Check LEFT Motor (Current: Ch {self.config.motor_l}, Inv: {self.config.motor_l_invert})"
            )
            print(
                f"2. Check RIGHT Motor (Current: Ch {self.config.motor_r}, Inv: {self.config.motor_r_invert})"
            )
            print(
                "3. Check Gyro Orientation (Auto-Detect Wizard)"
            )
            print(
                f"4. Check I2C Bus (Current: {self.config.i2c_bus})"
            )
            print("5. Final Verification Test (Left -> Right -> Both)")
            print("6. Save & Exit")
            print("7. Exit without Saving")

            choice = input("Select option: ").strip()

            match choice:
                case "1":
                    self.check_motor(MotorSide.LEFT)
                case "2":
                    self.check_motor(MotorSide.RIGHT)
                case "3":
                    self.check_gyro()
                case "4":
                    self.check_i2c_bus()
                case "5":
                    self.verify_movement()
                case "6":
                    self.config.save()
                    self.cleanup()
                    print("\nWiring Check Complete.")
                    print("You can now place the robot on the floor and run the main program.")
                    sys.exit(0)
                case "7":
                    self.cleanup()
                    sys.exit(0)
                case _:
                    print("Invalid option.")

    def reload_hw(self):
        """Reload hardware with updated config."""
        self.hw.stop()
        self.init_hw()

    def check_motor(self, side: MotorSide):
        """Interactive Motor Check."""
        print(f"\n>>> Checking {side.upper()} Motor...")
        print("Spinning with POSITIVE speed (+30) for 2 seconds...")

        if side == MotorSide.LEFT:
            self.hw.set_motors(30, 0)
        else:
            self.hw.set_motors(0, 30)

        time.sleep(2)
        self.hw.stop()

        print(f"\nDid the {side.upper()} motor spin FORWARD?")
        print("Definition of FORWARD: The TOP of the wheel moves towards the FRONT of the robot.")
        print("y: Yes, it spun forward (Correct)")
        print("n: No, it spun BACKWARD (Reverse Needed)")
        print("o: No, the OTHER motor spun (Swap Needed)")

        ans = input("Result (y/n/o): ").strip().lower()

        match ans:
            case "y":
                print("Good.")
            case "n":
                print(f"-> Marking {side.upper()} motor as INVERTED.")
                if side == MotorSide.LEFT:
                    self.config.motor_l_invert = not self.config.motor_l_invert
                else:
                    self.config.motor_r_invert = not self.config.motor_r_invert
                self.reload_hw()
            case "o":
                print("-> SWAPPING Left and Right Motor Channels.")
                # Swap channels
                self.config.motor_l, self.config.motor_r = (
                    self.config.motor_r,
                    self.config.motor_l,
                )
                self.reload_hw()
                print("Channels swapped.")
                print("!!! IMPORTANT: You MUST re-check the direction for BOTH motors now. !!!")
                print("Select option 1 and 2 again to verify.")
            case _:
                print("Invalid input.")

    def check_gyro(self):
        """Interactive Gyro/Accel Auto-Detection."""
        print("\n>>> Checking Gyro Orientation...")
        print("This wizard will automatically detect your sensor mounting.")
        print("It supports any mounting orientation.")

        # --- Step 1: Rest on Back ---
        print("\n[STEP 1] Place robot on floor, resting on BACK training wheel/strut.")
        print("Wait for it to be still.")
        input("Press Enter to Measure...")

        print("Reading accelerometer (Gravity)...")
        accel_back = self.measure_avg_accel()
        print(f"Back Accel: {accel_back}")

        # --- Step 2: Rest on Front ---
        print("\n[STEP 2] Prepare to tip robot forward to rest on FRONT training wheel.")
        print("When you press Enter, I will record for 3 seconds.")
        print("During this time, TIP THE ROBOT FORWARD.")
        input("Press Enter to Start Recording...")

        print("Recording...")
        gyro_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        accel_history = []
        end_time = time.time() + 3.0

        while time.time() < end_time:
            a, g = self.hw.read_imu_raw()
            for k in gyro_sum:
                gyro_sum[k] += g[k]
            accel_history.append(a)
            time.sleep(0.01)

        # Calculate Front Accel (avg of last 0.5s approx)
        accel_front_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        samples_front = accel_history[-50:] # Last 50 samples
        for s in samples_front:
            for k in s:
                accel_front_sum[k] += s[k]
        accel_front = {k: v / len(samples_front) for k, v in accel_front_sum.items()}
        print(f"Front Accel: {accel_front}")

        # --- Analysis 1: Pitch, Vertical, Forward ---
        print("Analyzing Phase 1...")

        # Vertical Axis: Dominant in Sum(Back, Front)
        accel_sum_vec = {k: accel_back[k] + accel_front[k] for k in accel_back}
        abs_sum = {k: abs(v) for k, v in accel_sum_vec.items()}
        vert_axis = max(abs_sum, key=abs_sum.get)

        # Forward Axis: Dominant in Diff(Front, Back)
        accel_diff_vec = {k: accel_front[k] - accel_back[k] for k in accel_back}
        abs_diff = {k: abs(v) for k, v in accel_diff_vec.items()}
        # Exclude Vert Axis
        abs_diff.pop(vert_axis)
        fwd_axis = max(abs_diff, key=abs_diff.get)

        # Pitch Axis (Gyro): Max Integral
        abs_gyro = {k: abs(v) for k, v in gyro_sum.items()}
        pitch_axis = max(abs_gyro, key=abs_gyro.get)

        # Polarity Check
        # Vertical: Upright = -1g. Sum should be Negative.
        # If Sum > 0, Invert = True.
        vert_invert = accel_sum_vec[vert_axis] > 0

        # Forward: Front - Back.
        # Back Y = +0.7, Front Y = -0.7 -> Diff = -1.4.
        # So Diff should be Negative. If Diff > 0, Invert = True.
        fwd_invert = accel_diff_vec[fwd_axis] > 0

        # Pitch: Back -> Front = Positive Pitch Rate (Nose Down = Positive Pitch for Balancing?)
        # Standard: Nose Down = Positive Pitch.
        # So Gyro Integral should be Positive.
        # If Integral < 0, Invert = True.
        pitch_invert = gyro_sum[pitch_axis] < 0

        print(f"-> Vertical: {vert_axis.upper()} (Inv: {vert_invert})")
        print(f"-> Forward:  {fwd_axis.upper()} (Inv: {fwd_invert})")
        print(f"-> Pitch:    {pitch_axis.upper()} (Inv: {pitch_invert})")

        # Deduce Roll Axis
        all_axes = {"x", "y", "z"}
        roll_axis = list(all_axes - {vert_axis, fwd_axis})[0]
        # Assume Gyro Roll Axis matches Accel Roll Axis (physical orthogonality)
        # We will verify polarity later.

        # Apply Partial Config
        self.config.accel_vertical_axis = Axis(vert_axis)
        self.config.accel_vertical_invert = vert_invert
        self.config.accel_forward_axis = Axis(fwd_axis)
        self.config.accel_forward_invert = fwd_invert
        self.config.gyro_pitch_axis = Axis(pitch_axis)
        self.config.gyro_pitch_invert = pitch_invert

        # Pre-set Yaw/Roll axes to expected ones (Vertical/Roll)
        # Yaw corresponds to Vertical Axis
        yaw_axis = vert_axis
        self.config.gyro_yaw_axis = Axis(yaw_axis)
        self.config.gyro_roll_axis = Axis(roll_axis)

        self.reload_hw()

        # --- Step 3: Yaw (Turn Right) ---
        print("\n[STEP 3] Place robot on floor (Front Rest).")
        print("Prepare to ROTATE the robot 90 degrees RIGHT (Clockwise).")
        input("Press Enter to Start Recording...")

        print("Recording...")
        yaw_sum = 0.0
        end_time = time.time() + 2.0
        while time.time() < end_time:
            # Read converted to use current axis
            reading = self.hw.read_imu_converted()
            # We want to check the RAW value of the Yaw Axis to determine polarity
            # But read_imu_converted already applies the 'gyro_yaw_invert' from config (currently False).
            # So if we see Negative Rate, we need to flip the invert.
            yaw_sum += reading.yaw_rate
            time.sleep(0.01)

        print(f"Yaw Sum: {yaw_sum:.2f}")
        # Right Turn = Positive Rate.
        yaw_invert = False
        if yaw_sum < 0:
            print("-> Detected Negative Rate. Inverting Yaw.")
            yaw_invert = True
        else:
            print("-> Detected Positive Rate. Yaw Polarity Correct.")

        self.config.gyro_yaw_invert = yaw_invert
        self.reload_hw()

        # --- Step 4: Roll (Tilt Right) ---
        print("\n[STEP 4] Prepare to TILT the robot to the RIGHT.")
        input("Press Enter to Start Recording...")

        print("Recording...")
        roll_sum = 0.0
        end_time = time.time() + 2.0
        while time.time() < end_time:
            reading = self.hw.read_imu_converted()
            roll_sum += reading.roll_rate
            time.sleep(0.01)

        print(f"Roll Sum: {roll_sum:.2f}")
        # Right Tilt = Positive Rate?
        # Standard: Roll Right = Positive.
        roll_invert = False
        if roll_sum < 0:
             print("-> Detected Negative Rate. Inverting Roll.")
             roll_invert = True
        else:
             print("-> Detected Positive Rate. Roll Polarity Correct.")

        self.config.gyro_roll_invert = roll_invert
        self.reload_hw()

        print("\n[SUCCESS] All axes calibrated.")
        print(f"Pitch: {self.config.gyro_pitch_axis.value} (Inv: {self.config.gyro_pitch_invert})")
        print(f"Yaw:   {self.config.gyro_yaw_axis.value} (Inv: {self.config.gyro_yaw_invert})")
        print(f"Roll:  {self.config.gyro_roll_axis.value} (Inv: {self.config.gyro_roll_invert})")

        input("Press Enter to continue...")

    def measure_avg_accel(self):
        accel_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        samples = 40
        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            for k in accel_sum:
                accel_sum[k] += a[k]
            time.sleep(0.02)
        return {k: v / samples for k, v in accel_sum.items()}

    def verify_movement(self):
        """Final verification sequence: Left, Right, Both."""
        print("\n>>> FINAL VERIFICATION SEQUENCE <<<")
        print("1. Left Motor Forward (1s)")
        print("2. Right Motor Forward (1s)")
        print("3. BOTH Motors Forward (1s) -> MUST MOVE STRAIGHT")
        print("Ensure the robot has space to move!")
        input("Press Enter to START...")

        # Left
        print("-> Left Motor...")
        self.hw.set_motors(30, 0)
        time.sleep(1.0)
        self.hw.stop()
        time.sleep(0.5)

        # Right
        print("-> Right Motor...")
        self.hw.set_motors(0, 30)
        time.sleep(1.0)
        self.hw.stop()
        time.sleep(0.5)

        # Both
        print("-> BOTH Motors...")
        self.hw.set_motors(30, 30)
        time.sleep(1.0)
        self.hw.stop()

        print("\nDid the robot move STRAIGHT FORWARD in the final step?")
        print("y: Yes, it moved straight (Config is valid)")
        print("n: No, it spun or moved backwards")

        ans = input("Result (y/n): ").strip().lower()
        if ans == 'y':
            print("SUCCESS. You are ready to Save & Exit.")
        else:
            print("FAILURE. Please double check individual motors (Options 1 & 2).")
            print("If it spun in place: One motor is inverted relative to the other.")
            print("If it moved backward: Both motors are inverted.")

    def check_i2c_bus(self):
        """Interactive I2C Bus Switcher."""
        print(f"\n>>> Checking I2C Bus (Current: {self.config.i2c_bus})")
        print("If you are using the Picon Zero, you might need to use Software I2C (Bus 3).")
        print("Enter new Bus Number (e.g. 1 or 3), or press Enter to keep current.")

        ans = input("Bus Number: ").strip()
        if ans:
            try:
                new_bus = int(ans)
                self.config.i2c_bus = new_bus
                print(f"-> Switching to Bus {new_bus}...")
                self.reload_hw()

                # Test read
                print("Attempting to read from sensor...")
                try:
                    p = self.get_pitch_snapshot()
                    print(f"SUCCESS: Read pitch {p:.2f} from Bus {new_bus}.")
                except Exception as e:
                    print(f"FAILURE: Could not read from Bus {new_bus}: {e}")
                    print("Reverting might be needed if this persists.")

            except ValueError:
                print("Invalid number.")
        else:
            print("No change.")

    def get_pitch_snapshot(self) -> float:
        """Helper to get a stable average pitch."""
        p_sum = 0.0
        samples = 10
        for _ in range(samples):
            # Now using the shared logic in hw
            reading = self.hw.read_imu_converted()
            p_sum += reading.pitch_angle
            time.sleep(0.01)

        return p_sum / samples

    def cleanup(self):
        """Release hardware."""
        self.hw.stop()
        self.hw.cleanup()


def main():
    try:
        app = WiringCheck()
        app.run()
    except KeyboardInterrupt:
        print("\nExiting...")
        sys.exit(0)


if __name__ == "__main__":
    main()
