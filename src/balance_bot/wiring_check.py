import time
import sys
import math
from .config import RobotConfig
from .robot_hardware import RobotHardware


class WiringCheck:
    def __init__(self):
        self.config = RobotConfig.load()
        # Initialize hardware using current config
        self.hw = RobotHardware(
            self.config.motor_l,
            self.config.motor_r,
            self.config.motor_l_invert,
            self.config.motor_r_invert,
        )
        self.hw.init()
        print("\n=== Wiring Check & Calibration Tool ===")
        print("Note: Ensure the robot is on a stand or wheels are free to spin.")

    def run(self):
        while True:
            print("\n--- Main Menu ---")
            print(f"1. Check LEFT Motor (Current: Ch {self.config.motor_l}, Inv: {self.config.motor_l_invert})")
            print(f"2. Check RIGHT Motor (Current: Ch {self.config.motor_r}, Inv: {self.config.motor_r_invert})")
            print(f"3. Check Gyro Orientation (Axis: {self.config.gyro_pitch_axis}, Inv: {self.config.gyro_pitch_invert})")
            print("4. Save & Exit")
            print("5. Exit without Saving")

            choice = input("Select option: ").strip()

            if choice == "1":
                self.check_motor("left")
            elif choice == "2":
                self.check_motor("right")
            elif choice == "3":
                self.check_gyro()
            elif choice == "4":
                self.config.save()
                self.cleanup()
                sys.exit(0)
            elif choice == "5":
                self.cleanup()
                sys.exit(0)
            else:
                print("Invalid option.")

    def reload_hw(self):
        self.hw.stop()
        self.hw = RobotHardware(
            self.config.motor_l,
            self.config.motor_r,
            self.config.motor_l_invert,
            self.config.motor_r_invert,
        )
        self.hw.init()

    def check_motor(self, side: str):
        print(f"\n>>> Checking {side.upper()} Motor...")
        print("Spinning FORWARD (positive speed) for 2 seconds...")

        # Determine which motor to spin based on side logic handled by set_motors
        # But wait, set_motors takes (left, right).
        if side == "left":
            self.hw.set_motors(30, 0)
        else:
            self.hw.set_motors(0, 30)

        time.sleep(2)
        self.hw.stop()

        print(f"\nDid the {side.upper()} motor spin FORWARD? (Wheel top moves forward)")
        print("y: Yes, it spun forward (Correct)")
        print("n: No, it spun BACKWARD (Reverse Needed)")
        print("o: No, the OTHER motor spun (Swap Needed)")

        ans = input("Result (y/n/o): ").strip().lower()

        if ans == 'y':
            print("Good.")
        elif ans == 'n':
            print(f"-> Marking {side.upper()} motor as INVERTED.")
            if side == "left":
                self.config.motor_l_invert = not self.config.motor_l_invert
            else:
                self.config.motor_r_invert = not self.config.motor_r_invert
            self.reload_hw()
        elif ans == 'o':
            print("-> SWAPPING Left and Right Motor Channels.")
            # Swap channels
            self.config.motor_l, self.config.motor_r = self.config.motor_r, self.config.motor_l
            # Also swap invert flags? Usually if channels are swapped, wiring is swapped.
            # But the invert logic stays with the logical side?
            # If I swap channels, the physical motor on Ch X is now controlled by the other logical side.
            # Let's just swap channels and ask user to re-test.
            self.reload_hw()
            print("Channels swapped. Please re-test.")
        else:
            print("Invalid input.")

    def check_gyro(self):
        print("\n>>> Checking Gyro Orientation...")
        print("Hold the robot upright (approx vertical). Press Enter when ready.")
        input()

        # Read baseline
        p, _, _ = self.get_pitch_snapshot()
        print(f"Baseline Pitch: {p:.2f}")

        print("\nNow tilt the robot FORWARD (approx 20-30 degrees).")
        print("Hold it there and press Enter.")
        input()

        p_tilt, _, _ = self.get_pitch_snapshot()
        print(f"Tilted Pitch: {p_tilt:.2f}")

        diff = p_tilt - p
        print(f"Change: {diff:.2f}")

        if diff > 10:
            print("-> Logic matches: Forward tilt = Positive pitch change.")
            print("Seems CORRECT.")
        elif diff < -10:
            print("-> Forward tilt caused NEGATIVE pitch change.")
            print("Need to INVERT pitch.")
            ans = input("Apply Inversion? (y/n): ").strip().lower()
            if ans == 'y':
                self.config.gyro_pitch_invert = not self.config.gyro_pitch_invert
        else:
            print("-> Change was too small or axis is wrong.")
            print(f"Current Axis: {self.config.gyro_pitch_axis}")
            ans = input("Try swapping Pitch Axis (X <-> Y)? (y/n): ").strip().lower()
            if ans == 'y':
                if self.config.gyro_pitch_axis == "x":
                    self.config.gyro_pitch_axis = "y"
                else:
                    self.config.gyro_pitch_axis = "x"
                print(f"Axis changed to {self.config.gyro_pitch_axis}. Please re-test.")

    def get_pitch_snapshot(self):
        # Average a few readings
        p_sum = 0
        samples = 10
        for _ in range(samples):
            accel, gyro = self.hw.read_imu_raw()

            # Re-implement get_pitch logic here or refactor?
            # Duplicating logic is bad, but RobotController isn't easily importable/usable as a utility without running it.
            # I'll implement the same math here, ensuring it uses config.

            if self.config.gyro_pitch_axis == "y":
                raw_acc_y = accel["x"]
                # raw_gyro_rate = gyro["y"]
            else:
                raw_acc_y = accel["y"]
                # raw_gyro_rate = gyro["x"]

            acc_angle = math.degrees(math.atan2(raw_acc_y, accel["z"]))

            if self.config.gyro_pitch_invert:
                acc_angle = -acc_angle

            p_sum += acc_angle
            time.sleep(0.01)

        return p_sum / samples, 0, 0

    def cleanup(self):
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
