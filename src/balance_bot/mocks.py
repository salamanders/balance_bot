import os
import math


class MockPiconZero:
    def init(self) -> None:
        print("[MockPiconZero] init")

    def stop(self) -> None:
        print("[MockPiconZero] stop")

    def setMotor(self, motor: int, value: int) -> None:
        # Mock implementation, no-op or log if needed
        pass

    def cleanup(self) -> None:
        print("[MockPiconZero] cleanup")


class MockMPU6050:
    def __init__(self, address: int):
        self.address = address
        print(f"[MockMPU6050] init at {address}")

    def get_accel_data(self) -> dict[str, float]:
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

        return {"x": 0.0, "y": y, "z": z}

    def get_gyro_data(self) -> dict[str, float]:
        return {"x": 0.0, "y": 0.0, "z": 0.0}
