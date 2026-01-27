import os
import math
from typing import Any


class RobotHardware:
    def __init__(
        self,
        motor_l: int,
        motor_r: int,
        invert_l: bool = False,
        invert_r: bool = False,
        gyro_axis: str = "x",
        gyro_invert: bool = False,
    ):
        self.motor_l = motor_l
        self.motor_r = motor_r
        self.invert_l = invert_l
        self.invert_r = invert_r
        self.gyro_axis = gyro_axis
        self.gyro_invert = gyro_invert

        self.pz: Any = None
        self.sensor: Any = None

        self._init_hardware()

    def _init_hardware(self) -> None:
        # Check for mock request or missing hardware
        use_mock = False
        if os.environ.get("MOCK_HARDWARE"):
            use_mock = True
        else:
            try:
                # Try importing real hardware
                from . import piconzero as pz_module
                from mpu6050 import mpu6050

                self.pz = pz_module
                self.sensor = mpu6050(0x68)
                print("Hardware initialized.")
            except (ImportError, OSError):
                print("Hardware not found or Mock requested.")
                use_mock = True

        if use_mock:
            print("Running in Mock Mode")
            from .mocks import MockPiconZero, MockMPU6050

            self.pz = MockPiconZero()
            self.sensor = MockMPU6050(0x68)

    def init(self) -> None:
        """Initialize hardware."""
        if hasattr(self.pz, "init"):
            self.pz.init()

    def read_imu_raw(self) -> tuple[dict[str, float], dict[str, float]]:
        """
        Returns raw accelerometer and gyro data.
        Returns: (accel_dict, gyro_dict)
        """
        if not self.sensor:
            return {"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 0.0, "y": 0.0, "z": 0.0}

        return self.sensor.get_accel_data(), self.sensor.get_gyro_data()

    def read_imu_processed(self) -> tuple[float, float, float]:
        """
        Read IMU and calculate pitch/rates based on config.
        Returns: (accel_angle, pitch_rate, yaw_rate)
        """
        accel, gyro = self.read_imu_raw()

        if self.gyro_axis == "y":
            # Pitch around Y axis
            raw_acc_y = accel["x"]
            raw_gyro_rate = gyro["y"]
        else:
            # Default: Pitch around X axis (Y is forward)
            raw_acc_y = accel["y"]
            raw_gyro_rate = gyro["x"]

        # Calculate Accelerometer Angle
        # atan2 returns radians, convert to degrees
        acc_angle = math.degrees(math.atan2(raw_acc_y, accel["z"]))
        gyro_rate = raw_gyro_rate
        yaw_rate = gyro["z"]

        if self.gyro_invert:
            acc_angle = -acc_angle
            gyro_rate = -gyro_rate

        return acc_angle, gyro_rate, yaw_rate

    def set_motors(self, left: float, right: float) -> None:
        """
        Set motor speeds.
        :param left: Speed -100 to 100
        :param right: Speed -100 to 100
        """
        if self.invert_l:
            left = -left
        if self.invert_r:
            right = -right

        left_val = int(max(min(left, 100), -100))
        right_val = int(max(min(right, 100), -100))

        self.pz.setMotor(self.motor_l, left_val)
        self.pz.setMotor(self.motor_r, right_val)

    def stop(self) -> None:
        """Stop all motors."""
        if hasattr(self.pz, "stop"):
            self.pz.stop()
        else:
            self.pz.setMotor(self.motor_l, 0)
            self.pz.setMotor(self.motor_r, 0)

    def cleanup(self) -> None:
        """Cleanup hardware resources."""
        if hasattr(self.pz, "cleanup"):
            self.pz.cleanup()
