from dataclasses import dataclass
from functools import cached_property
from typing import Optional

import glm


@dataclass(frozen=True)
class IMUReading:
    """
    Immutable data structure for converted IMU readings.

    :param pitch_angle: Calculated pitch angle in degrees (Zero = Upright).
    :param pitch_rate: Angular velocity around pitch axis in deg/s.
    :param yaw_rate: Angular velocity around yaw (vertical) axis in deg/s.
    :param roll_angle: Calculated roll angle in degrees.
    :param roll_rate: Angular velocity around roll axis in deg/s.
    """
    pitch_angle: float
    pitch_rate: float
    yaw_rate: float
    roll_angle: float
    roll_rate: float
    error_count: int = 0
    accel_raw: Optional[glm.vec3] = None
    gyro_raw: Optional[glm.vec3] = None


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
        return max(
            (abs(s.pitch_rate) + abs(s.yaw_rate) + abs(s.roll_rate))
            for s in self.samples
        )

    @cached_property
    def final_pitch(self) -> float:
        if not self.samples:
            return 0.0
        return self.samples[-1].pitch_angle
