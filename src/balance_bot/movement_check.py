import time
import logging
import sys
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware

logger = logging.getLogger(__name__)

class MovementCheck:
    """
    Automated sequence to verify movement and wiring logic.
    Executes a Square pattern Forward, then a Square pattern Backward.
    Includes PESSIMISTIC verification of sensor data and user confirmation.
    """

    def __init__(self):
        print(">>> Initializing Movement Check...")
        self.config = RobotConfig.load()
        # Initialize hardware using current config
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
            motor_i2c_bus=self.config.motor_i2c_bus,
            imu_i2c_bus=self.config.imu_i2c_bus,
        )
        self.hw.init()
        print(">>> Hardware Initialized.")

    def run(self):
        """Execute the movement sequence."""
        try:
            print("\n=== Movement Verification Sequence (Pessimistic) ===")
            print("The robot will drive in a Square pattern (Forward), then Reverse.")
            print("Ensure the robot is on the floor with space to move.")
            input("Press Enter to START...")

            # 1. Forward Square
            print("\n>>> Phase 1: Forward Square")
            for i in range(4):
                print(f"\n--- Leg {i+1}/4 ---")
                self.drive_move(speed=40, duration=0.5, desc="Forward")
                time.sleep(0.2)
                self.turn_90(desc="Right Turn")
                time.sleep(0.5)

            # 2. Reverse Square
            print("\n>>> Phase 2: Reverse Square")
            for i in range(4):
                print(f"\n--- Leg {i+1}/4 ---")
                self.drive_move(speed=-40, duration=0.5, desc="Backward")
                time.sleep(0.2)
                self.turn_90(desc="Right Turn")
                time.sleep(0.5)

            print("\n>>> Sequence Complete.")

        except KeyboardInterrupt:
            print("\nSequence Aborted.")
        finally:
            self.hw.stop()
            self.hw.cleanup()

    def _analyze_dominance(self, data_dict: dict, expected_axis: str, label: str):
        """
        Check if expected_axis is the dominant signal in data_dict.
        """
        sorted_items = sorted(data_dict.items(), key=lambda x: abs(x[1]), reverse=True)
        winner, winner_val = sorted_items[0]
        runner, runner_val = sorted_items[1]

        ratio = abs(winner_val) / (abs(runner_val) + 1e-9)

        print(f"   [Analysis] {label}: Winner={winner.upper()} ({abs(winner_val):.2f}) vs Runner={runner.upper()} ({abs(runner_val):.2f})")

        if winner != expected_axis:
            print(f"   [FAILURE] Expected {expected_axis.upper()} to be dominant, but {winner.upper()} won!")
            return False

        if ratio < 1.5:
            print(f"   [WARNING] Signal is ambiguous! Ratio {ratio:.1f} < 1.5")
            return False

        print(f"   [PASS] {expected_axis.upper()} is dominant (Ratio {ratio:.1f}).")
        return True

    def drive_move(self, speed: float, duration: float, desc: str):
        """
        Drive straight and verify 'Forward' axis dominance.
        """
        print(f"-> {desc} (Speed {speed}) for {duration}s...")
        self.hw.set_motors(speed, speed)

        # Monitor Accelerometer
        fwd_axis = self.config.accel_forward_axis.value # e.g. "z"
        min_vals = {k: float('inf') for k in ['x','y','z']}
        max_vals = {k: float('-inf') for k in ['x','y','z']}

        start_time = time.monotonic()

        while (time.monotonic() - start_time) < duration:
            # Read Raw Data
            accel, _ = self.hw.read_imu_raw()
            for k in accel:
                v = accel[k]
                if v < min_vals[k]: min_vals[k] = v
                if v > max_vals[k]: max_vals[k] = v
            time.sleep(0.01)

        self.hw.stop()

        # Calculate Deltas
        deltas = {k: max_vals[k] - min_vals[k] for k in ['x','y','z']}

        # Verify Dominance
        success = self._analyze_dominance(deltas, fwd_axis, "Forward Acceleration")

        # User Confirmation
        ans = input(f"   User Check: Did the robot drive {desc.upper()}? [y/n]: ").strip().lower()
        if ans != 'y':
            print("   [FAILURE] User denied movement.")
            sys.exit(1)

        if not success:
            print("   [WARNING] Software detected possible axis mismatch, but User confirmed movement.")
            print("             Proceeding with caution...")

    def turn_90(self, desc: str):
        """
        Turn 90 degrees and verify 'Yaw' axis dominance.
        """
        print(f"-> {desc} (Target 90 deg)...")

        # Turn Right: Left Motor +, Right Motor -
        turn_speed = 40
        self.hw.set_motors(turn_speed, -turn_speed)

        target_angle = 90.0
        current_angle = 0.0

        yaw_axis = self.config.gyro_yaw_axis.value
        all_axes = ["x", "y", "z"]

        # Track mean absolute rates
        rate_sums = {k: 0.0 for k in all_axes}
        sample_count = 0

        last_time = time.monotonic()
        start_time = time.monotonic()
        timeout = 5.0

        while current_angle < target_angle:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            if now - start_time > timeout:
                print("   [WARNING] Turn Timeout! Gyro might be unresponsive.")
                break

            # 1. Get Converted (for Yaw Integration)
            reading = self.hw.read_imu_converted()
            rate = reading.yaw_rate
            current_angle += rate * dt

            # 2. Get Raw (for Axis Analysis)
            _, gyro = self.hw.read_imu_raw()
            for k in all_axes:
                rate_sums[k] += abs(gyro[k])
            sample_count += 1

            time.sleep(0.01)

        self.hw.stop()
        print(f"   Done. Measured Turn: {current_angle:.1f} deg")

        # Analysis
        if sample_count > 0:
            avg_rates = {k: rate_sums[k]/sample_count for k in all_axes}
            success = self._analyze_dominance(avg_rates, yaw_axis, "Yaw Rate")
        else:
            print("   [ERROR] No samples collected?")
            success = False

        # User Confirmation
        ans = input("   User Check: Did the robot turn roughly 90 degrees? [y/n]: ").strip().lower()
        if ans != 'y':
             print("   [FAILURE] User denied turn.")
             sys.exit(1)

        if not success:
            print("   [WARNING] Software detected possible axis mismatch, but User confirmed turn.")

if __name__ == "__main__":
    MovementCheck().run()
