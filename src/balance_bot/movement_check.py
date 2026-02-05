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
            i2c_bus=self.config.i2c_bus,
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
        """Drive straight for a duration."""
        print(f"-> {desc} (Speed {speed}) for {duration}s...")
        self.hw.set_motors(speed, speed)
        time.sleep(duration)
        self.hw.stop()

    def turn_90(self, desc: str):
        """Turn 90 degrees using Gyro integration."""
        print(f"-> {desc} (Target 90 deg)...")

        # Turn Right: Left Motor +, Right Motor -
        # Target Yaw Rate should be POSITIVE.
        turn_speed = 40
        self.hw.set_motors(turn_speed, -turn_speed)

        target_angle = 90.0
        current_angle = 0.0

        last_time = time.monotonic()

        # Timeout safety
        start_time = time.monotonic()
        timeout = 5.0

        while current_angle < target_angle:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            if now - start_time > timeout:
                print("   [WARNING] Turn Timeout! Gyro might be unresponsive.")
                break

            reading = self.hw.read_imu_converted()

            # Integrate Yaw Rate
            # We assume turning Right generates POSITIVE Yaw Rate.
            # If wiring is inverted, this might be negative, so we take absolute to just finish the turn?
            # User wants to VERIFY wiring. So if we turn right and rate is negative, that's a FAIL condition technically.
            # But to make the test "work" locally for a square, let's just integrate.
            # Ideally, we warn if rate is negative.

            rate = reading.yaw_rate
            current_angle += rate * dt

            # Simple debug output every 10 deg
            if int(current_angle) % 10 == 0:
                 # sys.stdout.write(f"\r   Angle: {current_angle:.1f}")
                 # sys.stdout.flush()
                 pass

            time.sleep(0.01)

        self.hw.stop()
        print(f"   Done. Measured Turn: {current_angle:.1f} deg")

        if current_angle < 0:
            print("   [WARNING] Measured Angle was NEGATIVE. Yaw Axis might be inverted!")

if __name__ == "__main__":
    MovementCheck().run()
