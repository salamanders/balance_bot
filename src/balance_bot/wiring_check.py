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
        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            gyro_axis=self.config.gyro_pitch_axis,
            gyro_invert=self.config.gyro_pitch_invert,
            accel_vertical_axis=self.config.accel_vertical_axis,
            accel_vertical_invert=self.config.accel_vertical_invert,
            accel_forward_axis=self.config.accel_forward_axis,
            accel_forward_invert=self.config.accel_forward_invert,
            i2c_bus=self.config.i2c_bus,
        )
        self.hw.init()
        print("\n=== Wiring Check & Calibration Tool ===")
        print("!!! IMPORTANT !!!")
        print("Ensure the robot is on a STAND or wheels are lifted off the ground.")
        print("The motors WILL spin during testing.")
        print("!!! IMPORTANT !!!")

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
        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            gyro_axis=self.config.gyro_pitch_axis,
            gyro_invert=self.config.gyro_pitch_invert,
            accel_vertical_axis=self.config.accel_vertical_axis,
            accel_vertical_invert=self.config.accel_vertical_invert,
            accel_forward_axis=self.config.accel_forward_axis,
            accel_forward_invert=self.config.accel_forward_invert,
            i2c_bus=self.config.i2c_bus,
        )
        self.hw.init()

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
        print("It supports any mounting orientation (Standard, Sideways, Upside-Down).")

        # --- Step 1: Upright ---
        print("\n[STEP 1] Hold the robot UPRIGHT (balanced vertical).")
        print("Keep it as still as possible.")
        input("Press Enter when ready...")

        print("Reading accelerometer for gravity vector...")
        accel_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        samples = 40
        for _ in range(samples):
            a, _ = self.hw.read_imu_raw()
            for k in accel_sum:
                accel_sum[k] += a[k]
            time.sleep(0.02)

        accel_upright = {k: v / samples for k, v in accel_sum.items()}
        print(
            f"Upright Accel: x={accel_upright['x']:.2f}, y={accel_upright['y']:.2f}, z={accel_upright['z']:.2f}"
        )

        # Detect Vertical Axis
        abs_accel = {k: abs(v) for k, v in accel_upright.items()}
        vert_axis = max(abs_accel, key=abs_accel.get)
        vert_val = accel_upright[vert_axis]
        vert_invert = vert_val < 0

        print(f"-> Detected Vertical Axis: {vert_axis.upper()} (Invert: {vert_invert})")

        # --- Step 2: Tilt Forward ---
        print("\n[STEP 2] Prepare to tilt the robot FORWARD (Nose Down) approx 30-45 deg.")
        print("When you press Enter, I will record for 3 seconds.")
        print("During that time, TILT IT FORWARD AND HOLD IT THERE.")
        input("Press Enter to start recording...")

        print("Recording... TILT NOW!")

        gyro_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        # Buffer to store last few accel readings for 'held' position
        accel_history = []

        # Record for 3 seconds
        end_time = time.time() + 3.0
        sample_count = 0

        while time.time() < end_time:
            a, g = self.hw.read_imu_raw()

            # Integrate gyro rate (simple sum is proportional to mean)
            for k in gyro_sum:
                gyro_sum[k] += g[k]

            accel_history.append(a)
            if len(accel_history) > 20: # Keep last ~20 samples (approx 0.5s)
                accel_history.pop(0)

            sample_count += 1
            time.sleep(0.02)

        # Analyze Gyro (Pitch Axis)
        # We use the mean rate to determine axis and direction
        gyro_avg = {k: v / sample_count for k, v in gyro_sum.items()}
        abs_gyro = {k: abs(v) for k, v in gyro_avg.items()}
        gyro_axis = max(abs_gyro, key=abs_gyro.get)
        gyro_val = gyro_avg[gyro_axis]

        # Forward Tilt -> Positive Pitch Change.
        # Rate should be Positive. If Negative, Invert.
        gyro_invert = gyro_val < 0

        # Analyze Accel (Forward Axis)
        # Use average of the last captured samples (held position)
        accel_tilt_sum = {"x": 0.0, "y": 0.0, "z": 0.0}
        for sample in accel_history:
            for k in sample:
                accel_tilt_sum[k] += sample[k]

        accel_tilted = {k: v / len(accel_history) for k, v in accel_tilt_sum.items()}

        print(f"Tilted Accel (End): x={accel_tilted['x']:.2f}, y={accel_tilted['y']:.2f}, z={accel_tilted['z']:.2f}")

        # Detect Forward Axis
        # Exclude Vertical Axis
        other_axes = [ax for ax in ["x", "y", "z"] if ax != vert_axis]
        tilted_others = {k: accel_tilted[k] for k in other_axes}
        tilted_abs = {k: abs(v) for k, v in tilted_others.items()}

        fwd_axis = max(tilted_abs, key=tilted_abs.get)
        fwd_val = tilted_others[fwd_axis]

        # Detect Forward Invert
        # Nose Down -> Positive Pitch.
        # Pitch = atan2(Fwd, Vert).
        # If Vert (Adjusted) is +1g, Fwd must be Positive to give Positive Pitch.
        # So if fwd_val is Negative, Invert.
        fwd_invert = fwd_val < 0

        print(f"-> Detected Forward Axis: {fwd_axis.upper()} (Invert: {fwd_invert})")
        print(f"-> Detected Gyro Axis:    {gyro_axis.upper()} (Invert: {gyro_invert})")

        print("\nApplying configuration...")
        self.config.accel_vertical_axis = Axis(vert_axis)
        self.config.accel_vertical_invert = vert_invert
        self.config.accel_forward_axis = Axis(fwd_axis)
        self.config.accel_forward_invert = fwd_invert
        self.config.gyro_pitch_axis = Axis(gyro_axis)
        self.config.gyro_pitch_invert = gyro_invert

        self.reload_hw()

        # --- Verification ---
        print("\n[VERIFICATION]")
        print("1. Hold Upright. Pitch should be close to 0.")
        input("   Press Enter when ready...")
        p = self.get_pitch_snapshot()
        print(f"   Current Pitch: {p:.2f}")

        print("2. Tilt Forward. Pitch should be POSITIVE.")
        input("   Press Enter when holding Forward...")
        p_tilt = self.get_pitch_snapshot()
        print(f"   Tilted Pitch: {p_tilt:.2f}")

        if p_tilt > 10:
            print("   SUCCESS! Pitch increased positively.")
        else:
            print("   WARNING: Pitch did not increase positively. Calibration might be wrong.")

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
