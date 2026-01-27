import time
import sys
from .config import RobotConfig
from .robot_hardware import RobotHardware


class WiringCheck:
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

            match choice:
                case "1":
                    self.check_motor("left")
                case "2":
                    self.check_motor("right")
                case "3":
                    self.check_gyro()
                case "4":
                    self.config.save()
                    self.cleanup()
                    sys.exit(0)
                case "5":
                    self.cleanup()
                    sys.exit(0)
                case _:
                    print("Invalid option.")

    def reload_hw(self):
        self.hw.stop()
        self.hw = RobotHardware(
            motor_l=self.config.motor_l,
            motor_r=self.config.motor_r,
            invert_l=self.config.motor_l_invert,
            invert_r=self.config.motor_r_invert,
            gyro_axis=self.config.gyro_pitch_axis,
            gyro_invert=self.config.gyro_pitch_invert,
        )
        self.hw.init()

    def check_motor(self, side: str):
        print(f"\n>>> Checking {side.upper()} Motor...")
        print("Spinning FORWARD (positive speed) for 2 seconds...")

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

        match ans:
            case 'y':
                print("Good.")
            case 'n':
                print(f"-> Marking {side.upper()} motor as INVERTED.")
                if side == "left":
                    self.config.motor_l_invert = not self.config.motor_l_invert
                else:
                    self.config.motor_r_invert = not self.config.motor_r_invert
                self.reload_hw()
            case 'o':
                print("-> SWAPPING Left and Right Motor Channels.")
                # Swap channels
                self.config.motor_l, self.config.motor_r = self.config.motor_r, self.config.motor_l
                self.reload_hw()
                print("Channels swapped. Please re-test.")
            case _:
                print("Invalid input.")

    def check_gyro(self):
        print("\n>>> Checking Gyro Orientation...")
        print("Hold the robot upright (approx vertical). Press Enter when ready.")
        input()

        # Read baseline
        p = self.get_pitch_snapshot()
        print(f"Baseline Pitch: {p:.2f}")

        print("\nNow tilt the robot FORWARD (approx 20-30 degrees).")
        print("Hold it there and press Enter.")
        input()

        p_tilt = self.get_pitch_snapshot()
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
                self.reload_hw()
        else:
            print("-> Change was too small or axis is wrong.")
            print(f"Current Axis: {self.config.gyro_pitch_axis}")
            ans = input("Try swapping Pitch Axis (X <-> Y)? (y/n): ").strip().lower()
            if ans == 'y':
                if self.config.gyro_pitch_axis == "x":
                    self.config.gyro_pitch_axis = "y"
                else:
                    self.config.gyro_pitch_axis = "x"
                self.reload_hw()
                print(f"Axis changed to {self.config.gyro_pitch_axis}. Please re-test.")

    def get_pitch_snapshot(self) -> float:
        # Average a few readings
        p_sum = 0.0
        samples = 10
        for _ in range(samples):
            # Now using the shared logic in hw
            reading = self.hw.read_imu_processed()
            p_sum += reading.pitch_angle
            time.sleep(0.01)

        return p_sum / samples

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
