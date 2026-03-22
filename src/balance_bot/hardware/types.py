from dataclasses import dataclass
from functools import cached_property

import glm


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
        return max(
            (abs(s.pitch_rate) + abs(s.yaw_rate) + abs(s.roll_rate))
            for s in self.samples
        )

    @cached_property
    def final_pitch(self) -> float:
        if not self.samples:
            return 0.0
        return self.samples[-1].pitch_angle
