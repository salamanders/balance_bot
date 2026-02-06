import time
import logging
from .config import RobotConfig
from .hardware.robot_hardware import RobotHardware

logger = logging.getLogger(__name__)

class MovementCheck:
    """
    Automated sequence to verify movement and wiring logic.
    Executes a Square pattern Forward, then a Square pattern Backward.
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
            print("\n=== Movement Verification Sequence ===")
            print("The robot will drive in a Square pattern (Forward), then Reverse.")
            print("Ensure the robot is on the floor with space to move.")
            input("Press Enter to START...")

            # 1. Forward Square
            print("\n>>> Phase 1: Forward Square")
            for i in range(4):
                print(f"--- Leg {i+1}/4 ---")
                self.drive_move(speed=40, duration=0.5, desc="Forward")
                time.sleep(0.2)
                self.turn_90(desc="Right Turn")
                time.sleep(0.2)

            # 2. Reverse Square
            print("\n>>> Phase 2: Reverse Square")
            for i in range(4):
                print(f"--- Leg {i+1}/4 ---")
                self.drive_move(speed=-40, duration=0.5, desc="Backward")
                time.sleep(0.2)
                self.turn_90(desc="Right Turn")
                time.sleep(0.2)

            print("\n>>> Sequence Complete.")

        except KeyboardInterrupt:
            print("\nSequence Aborted.")
        finally:
            self.hw.stop()
            self.hw.cleanup()

    def drive_move(self, speed: float, duration: float, desc: str):
        """
        Drive straight for a duration.
        Monitors Accelerometer on the 'Forward' axis to confirm motion.
        """
        print(f"-> {desc} (Speed {speed}) for {duration}s...")
        self.hw.set_motors(speed, speed)

        # Monitor Accelerometer
        fwd_axis = self.hw.accel_forward_axis
        min_accel = float('inf')
        max_accel = float('-inf')

        start_time = time.monotonic()

        while (time.monotonic() - start_time) < duration:
            # Read Raw Data
            accel, _ = self.hw.read_imu_raw()
            val = accel[fwd_axis]

            # Track peaks
            if val < min_accel: min_accel = val
            if val > max_accel: max_accel = val

            time.sleep(0.01)

        self.hw.stop()

        # Report
        delta = max_accel - min_accel
        print(f"   [CHECK] Forward Axis ({fwd_axis}) Range: [{min_accel:.2f}, {max_accel:.2f}] (Delta: {delta:.2f}g)")

        # Simple heuristic check
        if delta > 0.05:
            print(f"   [PASS] Motion Detected on {fwd_axis}-axis.")
        else:
            print(f"   [WARN] Low motion detected ({delta:.2f}g). Motors unplugged or stall?")

    def turn_90(self, desc: str):
        """
        Turn 90 degrees using Gyro integration.
        Monitors 'Off-Axis' gyros to ensure clean rotation.
        """
        print(f"-> {desc} (Target 90 deg)...")

        # Turn Right: Left Motor +, Right Motor -
        turn_speed = 40
        self.hw.set_motors(turn_speed, -turn_speed)

        target_angle = 90.0
        current_angle = 0.0

        # Identify Axes
        # Note: RobotHardware.read_imu_converted() uses accel_vertical_axis for Yaw rate.
        yaw_axis = self.hw.accel_vertical_axis
        all_axes = ["x", "y", "z"]
        off_axes = [a for a in all_axes if a != yaw_axis]

        # Track peaks for off-axes
        off_axis_peaks = {a: 0.0 for a in off_axes}

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

            # 2. Get Raw (for Off-Axis Monitoring)
            _, gyro = self.hw.read_imu_raw()
            for axis in off_axes:
                raw_rate = abs(gyro[axis])
                if raw_rate > off_axis_peaks[axis]:
                    off_axis_peaks[axis] = raw_rate

            time.sleep(0.01)

        self.hw.stop()
        print(f"   Done. Measured Turn: {current_angle:.1f} deg")

        if current_angle < 0:
            print("   [WARNING] Measured Angle was NEGATIVE. Yaw Axis might be inverted!")

        # Report Off-Axis Stats
        print(f"   [CHECK] Primary Yaw ({yaw_axis}) Integrated: {current_angle:.1f} deg")
        for axis in off_axes:
            peak = off_axis_peaks[axis]
            status = "Quiet" if peak < 20.0 else "NOISY"
            print(f"   [CHECK] Off-Axis ({axis}) Peak Rate: {peak:.1f} deg/s ({status})")

if __name__ == "__main__":
    MovementCheck().run()
