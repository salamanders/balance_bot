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

from dataclasses import dataclass
from functools import cached_property

from pyglm import glm


@dataclass(slots=True)
class IMUReading:
    """
    Data structure for converted IMU readings.
    """

    pitch_angle: float
    pitch_rate: float
    yaw_rate: float
    roll_angle: float
    roll_rate: float
    error_count: int = 0
    accel_raw: glm.vec3 | None = None
    gyro_raw: glm.vec3 | None = None


@dataclass(frozen=True)
class DriveCommand:
    """
    Command parameters for a drive and measure operation.
    """

    left_power: float
    right_power: float
    duration: float
    sample_interval: float = 0.01
    wait_for_stability: bool = False
    trim_override: float | None = None


@dataclass(frozen=True)
class MeasureResult:
    """
    Result of a drive-and-measure maneuver.
    Encapsulates raw samples and derived statistics.
    """

    duration: float
    samples: list[IMUReading]

    @cached_property
    def avg_yaw_rate(self) -> float:
        if not self.samples:
            return 0.0
        return sum(s.yaw_rate for s in self.samples) / len(self.samples)

    @cached_property
    def abs_avg_yaw_rate(self) -> float:
        if not self.samples:
            return 0.0
        return sum(abs(s.yaw_rate) for s in self.samples) / len(self.samples)

    @cached_property
    def max_rate(self) -> float:
        if not self.samples:
            return 0.0
        return max((abs(s.pitch_rate) + abs(s.yaw_rate) + abs(s.roll_rate)) for s in self.samples)

    @cached_property
    def final_pitch(self) -> float:
        if not self.samples:
            return 0.0
        return self.samples[-1].pitch_angle
