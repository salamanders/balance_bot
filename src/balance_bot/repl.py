import argparse
import logging
import sys
import termios
import tty

from pyglm import glm

from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.utils import analyze_dominance, average_vector

# Basic logging setup
logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)


def getch() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class Repl:
    def __init__(self, allow_mocks: bool = False) -> None:
        if allow_mocks:
            import os

            os.environ["ALLOW_MOCK_FALLBACK"] = "1"

        self.config = HardwareConfig.load()
        self.state = LearningState.load()

        # If not initialized yet, we still need basic motor pins (e.g. 0 and 1)
        if self.config.motor_l is None or self.config.motor_r is None:
            updates = {"motor_l": 0, "motor_r": 1}
            self.config = self.config.model_copy(update=updates)

        # Force I2C buses to typical defaults if empty (Toddler Phase)
        if self.config.motor_i2c_bus is None or self.config.imu_i2c_bus is None:
            updates = {
                "motor_i2c_bus": self.config.motor_i2c_bus
                if self.config.motor_i2c_bus is not None
                else 1,
                "imu_i2c_bus": self.config.imu_i2c_bus
                if self.config.imu_i2c_bus is not None
                else 1,
            }
            self.config = self.config.model_copy(update=updates)

        self.hw = RobotHardware(self.config, self.state)
        self.speed = 50.0

    def print_help(self) -> None:
        print("\n--- Balance Bot Manual REPL ---")
        print("x : Switch left and right motor channels")
        print("l : Toggle left motor polarity")
        print("r : Toggle right motor polarity")
        print("+ : Increase speed by 10")
        print("- : Decrease speed by 10")
        print("w : Drive forward for 1 second")
        print("s : Drive backward for 1 second")
        print("a : Turn left 45 degrees (drives right wheel forward)")
        print("d : Turn right 45 degrees (drives left wheel forward)")
        print("g : Test gyro orientation using gravity and spin")
        print("f : Manually flop the robot to measure limits & print FULL REPORT")
        print("c : Show current polarity, channels, and speed")
        print("q : Exit REPL")
        print("? / h : Help")
        print("-------------------------------")

    def run(self) -> None:
        self.print_help()
        try:
            while True:
                print("\n> ", end="", flush=True)
                cmd = getch().lower()
                print(cmd)  # Echo the command

                if cmd in ["q", "\x03"]:  # q or Ctrl+C
                    break
                elif cmd == "x":
                    self._cmd_switch()
                elif cmd == "l":
                    self._cmd_flip_l()
                elif cmd == "r":
                    self._cmd_flip_r()
                elif cmd == "+":
                    self._cmd_speed_up()
                elif cmd == "-":
                    self._cmd_speed_down()
                elif cmd == "w":
                    self._cmd_fwd()
                elif cmd == "s":
                    self._cmd_back()
                elif cmd == "a":
                    self._cmd_turn_l()
                elif cmd == "d":
                    self._cmd_turn_r()
                elif cmd == "g":
                    self._cmd_test_gyro()
                elif cmd == "f":
                    self._cmd_flop()
                elif cmd == "c":
                    self._cmd_status()
                elif cmd in ["?", "h"]:
                    self.print_help()
                else:
                    print("Unknown command. Type '?' for help.")
        finally:
            self.hw.stop()

    def _cmd_switch(self) -> None:
        left_ch, right_ch = self.config.motor_l, self.config.motor_r
        # Keep invert flags with the wheel's physical location, not the channel ID?
        # Typically we just swap channels.
        new_config = self.config.model_copy(update={"motor_l": right_ch, "motor_r": left_ch})
        self.config = new_config
        self.hw.apply_config(self.config)
        print(f"Swapped! Left is now Channel {right_ch}, Right is Channel {left_ch}")

    def _cmd_flip_l(self) -> None:
        new_val = not self.config.motor_l_invert
        self.config = self.config.model_copy(update={"motor_l_invert": new_val})
        self.hw.apply_config(self.config)
        print(f"Left polarity is now {'INVERTED' if new_val else 'NORMAL'}")

    def _cmd_flip_r(self) -> None:
        new_val = not self.config.motor_r_invert
        self.config = self.config.model_copy(update={"motor_r_invert": new_val})
        self.hw.apply_config(self.config)
        print(f"Right polarity is now {'INVERTED' if new_val else 'NORMAL'}")

    def _cmd_speed_up(self) -> None:
        self.speed += 10.0
        print(f"Speed increased to {self.speed}")

    def _cmd_speed_down(self) -> None:
        self.speed = max(0.0, self.speed - 10.0)
        print(f"Speed decreased to {self.speed}")

    def _cmd_fwd(self) -> None:
        print(f"Driving FORWARD at {self.speed} for 1s...")
        self.hw.execute_maneuver([(self.speed, self.speed, 1.0)])

    def _cmd_back(self) -> None:
        print(f"Driving BACKWARD at {self.speed} for 1s...")
        self.hw.execute_maneuver([(-self.speed, -self.speed, 1.0)])

    def _cmd_turn_l(self) -> None:
        # Drives right wheel forward (turn left)
        print(f"Turning LEFT at {self.speed} for 0.5s...")
        self.hw.execute_maneuver([(0.0, self.speed, 0.5)])

    def _cmd_turn_r(self) -> None:
        # Drives left wheel forward (turn right)
        print(f"Turning RIGHT at {self.speed} for 0.5s...")
        self.hw.execute_maneuver([(self.speed, 0.0, 0.5)])

    def _cmd_test_gyro(self) -> None:
        from balance_bot.enums import Axis

        print("\nTesting Gyro Orientation...")
        print("1. Measuring gravity...")
        baseline_accel = self.hw.measure_gravity(1.0)

        print("2. Spinning robot (LEFT TURN) to measure gyro...")
        # L motor backward, R motor forward -> left spin
        res = self.hw.execute_maneuver([(-self.speed, self.speed, 0.5)])
        spin_gyro = average_vector([s.gyro_raw for s in res.samples if s.gyro_raw])

        if spin_gyro is None or glm.length(spin_gyro) < 5.0:
            print("Failed to detect spin. Is the robot resting on the ground?")
            return

        print("3. Deducing orientation...")
        # Up Vector is the gravity baseline
        vert_axis_name, _, _ = analyze_dominance(
            {"x": baseline_accel.x, "y": baseline_accel.y, "z": baseline_accel.z}, "Vertical"
        )
        accel_vertical_invert = getattr(baseline_accel, vert_axis_name) < 0

        # Gyro Yaw is the dominant axis during the spin
        yaw_axis_name, _, _ = analyze_dominance(
            {"x": spin_gyro.x, "y": spin_gyro.y, "z": spin_gyro.z}, "Yaw"
        )
        # For a left spin (counter-clockwise from above), the up vector should point OUT of the robot top.
        # So we align Yaw with Vertical conceptually.
        gyro_yaw_invert = (
            getattr(spin_gyro, yaw_axis_name) > 0
        )  # Left turn typically positive rate, if negative, invert

        # To get forward and pitch, we'd normally pulse forward/backwards, but here we'll just guess based on Vertical and Yaw being Z.
        # Assuming Y is forward/pitch, X is left/right roll.
        pitch_axis_name = "y" if yaw_axis_name != "y" else "x"
        fwd_axis_name = "x" if vert_axis_name != "x" and pitch_axis_name != "x" else "y"

        print(f"Vertical Axis: {vert_axis_name.upper()} (Invert: {accel_vertical_invert})")
        print(f"Yaw Axis:      {yaw_axis_name.upper()} (Invert: {gyro_yaw_invert})")

        updates = {
            "accel_vertical_axis": Axis(vert_axis_name),
            "accel_vertical_invert": accel_vertical_invert,
            "gyro_yaw_axis": Axis(yaw_axis_name),
            "gyro_yaw_invert": gyro_yaw_invert,
            "gyro_pitch_axis": Axis(pitch_axis_name),
            "accel_forward_axis": Axis(fwd_axis_name),
        }
        self.config = self.config.model_copy(update=updates)
        self.hw.apply_config(self.config)
        print("Gyro orientation updated in memory.")

    def _cmd_flop(self) -> None:
        print("\n--- Manual Flop Calibration ---")
        print("Hold the robot UPRIGHT. Press Enter to start reading angles...")
        input()

        print(
            "Now, slowly lean the robot FORWARD until it hits its mechanical limit (e.g. training wheels)."
        )
        print("Hold it there for a few seconds. Press Enter when done.")

        # Read max forward pitch
        input()
        reading = self.hw.read_imu_converted()
        fwd_angle = reading.pitch_angle
        print(f"Recorded Forward Angle: {fwd_angle:.2f} deg")

        print("\nNow, slowly lean the robot BACKWARD until it hits its mechanical limit.")
        print("Hold it there for a few seconds. Press Enter when done.")
        input()
        reading = self.hw.read_imu_converted()
        back_angle = reading.pitch_angle
        print(f"Recorded Backward Angle: {back_angle:.2f} deg")

        self.state.rest_angle_forward = fwd_angle
        self.state.rest_angle_backward = back_angle

        self.state.spatial_orientation_verified = True
        self.state.motor_direction_verified = True
        self.state.motor_phasing_verified = True
        self.state.motor_channels_verified = True

        self.config.save()
        self.state.save()

        print("\n=================================")
        print("          FULL REPORT            ")
        print("=================================")
        print("\n--- HardwareConfig ---")
        print(self.config.model_dump_json(indent=2))
        print("\n--- LearningState ---")
        print(self.state.model_dump_json(indent=2))
        print("=================================\n")
        print("Configuration saved to disk.")

    def _cmd_status(self) -> None:
        print(f"Channels : L={self.config.motor_l}, R={self.config.motor_r}")
        print(
            f"Polarity : L_invert={self.config.motor_l_invert}, R_invert={self.config.motor_r_invert}"
        )
        print(f"Speed    : {self.speed}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--allow-mocks", action="store_true", help="Allow fallback to mock hardware"
    )
    args = parser.parse_args()

    repl = Repl(allow_mocks=args.allow_mocks)
    repl.run()


if __name__ == "__main__":
    main()
