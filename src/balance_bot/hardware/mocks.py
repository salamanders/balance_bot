"""
# System Context
This module is part of the `balance_bot` application, designed to control a self-balancing
homebrew robot. It relies on a deterministic, high-frequency control loop and pessimistic hardware interactions.

# Business Rules
- Fail-fast initialization: The system must crash loudly if physical hardware is missing or unresponsive during boot.
- Fault-tolerant control loop: Once Tier 1 is running (e.g., `BalanceCore`), transient I/O errors must not collapse the system; use continuous data quality metrics instead of fatal exceptions.
- Physical pessimism: Never hardcode physical constants; rely on zero-knowledge self-discovery to deduce configuration.

# Dependency Maps
- Relies on internal configuration (`HardwareConfig`, `LearningState`).
- Interfaces with Tier 1 (`BalanceCore`), Tier 3 (`Agent`), and physical hardware abstraction (`RobotHardware`).
"""

import math
import os

from pyglm import glm


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
                with open("mock_pitch.txt") as f:
                    content = f.read().strip()
                    if content:
                        pitch = float(content)
            except (ValueError, OSError) as e:
                print(f"Warning: could not read mock_pitch.txt: {e}")

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
