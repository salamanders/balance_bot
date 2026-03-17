import os
import math
import glm


class MockPiconZero:
    @staticmethod
    def init() -> None:
        print("[MockPiconZero] init")

    @staticmethod
    def stop() -> None:
        print("[MockPiconZero] stop")

    @staticmethod
    def set_retries(retries: int) -> None:
        print(f"[MockPiconZero] set_retries: {retries}")

    def set_motor(self, motor: int, value: int) -> None:
        # Mock implementation, no-op or log if needed
        pass

    def set_motors(self, motor_0_val: int, motor_1_val: int) -> None:
        # Mock implementation, no-op or log if needed
        pass

    @staticmethod
    def cleanup() -> None:
        print("[MockPiconZero] cleanup")


class MockMPU6050:
    def __init__(self, address: int):
        self.address = address
        print(f"[MockMPU6050] init at {address}")

    @staticmethod
    def get_accel_data() -> glm.vec3:
        # Default vertical
        pitch = 0.0

        # Check for external override file
        if os.path.exists("mock_pitch.txt"):
            try:
                with open("mock_pitch.txt", "r") as f:
                    content = f.read().strip()
                    if content:
                        pitch = float(content)
            except (ValueError, OSError):
                pass

        # Convert pitch (degrees) to accel vector (assuming Y is forward)
        # pitch = atan2(y, z)
        # y = sin(pitch) * 9.8
        # z = cos(pitch) * 9.8
        rad = math.radians(pitch)
        y = math.sin(rad) * 9.8
        z = math.cos(rad) * 9.8

        return glm.vec3(0.0, y, z)

    @staticmethod
    def get_gyro_data() -> glm.vec3:
        return glm.vec3(0.0)
