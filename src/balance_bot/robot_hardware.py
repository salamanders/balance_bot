import os
from typing import Tuple, Dict, Any


class RobotHardware:
    def __init__(
        self, motor_l: int, motor_r: int, invert_l: bool = False, invert_r: bool = False
    ):
        self.motor_l = motor_l
        self.motor_r = motor_r
        self.invert_l = invert_l
        self.invert_r = invert_r
        self.pz: Any = None
        self.sensor: Any = None
        self.sensor_class: Any = None
        self.mock_mode = False

        # --- HARDWARE IMPORTS ---
        try:
            if os.environ.get("MOCK_HARDWARE"):
                raise ImportError("Mock requested")
            # Import real hardware
            from . import piconzero as pz_module
            from mpu6050 import mpu6050

            self.pz = pz_module
            self.sensor_class = mpu6050

        except (ImportError, OSError):
            print("Running in Mock Mode")
            self.mock_mode = True
            from .mocks import MockPiconZero, MockMPU6050

            self.pz = MockPiconZero()
            self.sensor_class = MockMPU6050

    def init(self) -> None:
        """Initialize hardware."""
        if hasattr(self.pz, "init"):
            self.pz.init()

        # Initialize sensor
        self.sensor = self.sensor_class(0x68)

    def read_imu_raw(self) -> Tuple[Dict[str, float], Dict[str, float]]:
        """
        Returns raw accelerometer and gyro data.
        Returns: (accel_dict, gyro_dict)
        """
        if not self.sensor:
            return {"x": 0.0, "y": 0.0, "z": 0.0}, {"x": 0.0, "y": 0.0, "z": 0.0}

        return self.sensor.get_accel_data(), self.sensor.get_gyro_data()

    def set_motors(self, left: float, right: float) -> None:
        """
        Set motor speeds.
        :param left: Speed -100 to 100
        :param right: Speed -100 to 100
        """
        # Invert if needed
        if self.invert_l:
            left = -left
        if self.invert_r:
            right = -right

        # Clamp values
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
