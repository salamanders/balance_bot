from dataclasses import dataclass
from functools import cached_property
from typing import Optional

import glm


class IMUReading:
    """
    Immutable data structure for converted IMU readings.

    NOTE: This is intentionally implemented as a standard class with __slots__
    rather than a @dataclass(frozen=True). In high-frequency loops (e.g. 100Hz),
    instantiating frozen dataclasses introduces significant overhead due to
    attribute assignment restrictions. This slotted class provides immutability
    (by convention and lack of __dict__) with much better instantiation performance.

    :param pitch_angle: Calculated pitch angle in degrees (Zero = Upright).
    :param pitch_rate: Angular velocity around pitch axis in deg/s.
    :param yaw_rate: Angular velocity around yaw (vertical) axis in deg/s.
    :param roll_angle: Calculated roll angle in degrees.
    :param roll_rate: Angular velocity around roll axis in deg/s.
    """
    __slots__ = ['pitch_angle', 'pitch_rate', 'yaw_rate', 'roll_angle', 'roll_rate', 'error_count', 'accel_raw', 'gyro_raw']

    def __init__(self, pitch_angle: float, pitch_rate: float, yaw_rate: float, roll_angle: float, roll_rate: float, error_count: int = 0, accel_raw: Optional[glm.vec3] = None, gyro_raw: Optional[glm.vec3] = None):
        self.pitch_angle = pitch_angle
        self.pitch_rate = pitch_rate
        self.yaw_rate = yaw_rate
        self.roll_angle = roll_angle
        self.roll_rate = roll_rate
        self.error_count = error_count
        self.accel_raw = accel_raw
        self.gyro_raw = gyro_raw

    def __repr__(self):
        return f"IMUReading(pitch_angle={self.pitch_angle}, pitch_rate={self.pitch_rate}, yaw_rate={self.yaw_rate}, roll_angle={self.roll_angle}, roll_rate={self.roll_rate}, error_count={self.error_count}, accel_raw={self.accel_raw}, gyro_raw={self.gyro_raw})"

    def __eq__(self, other):
        if not isinstance(other, IMUReading):
            return NotImplemented
        return (self.pitch_angle == other.pitch_angle and
                self.pitch_rate == other.pitch_rate and
                self.yaw_rate == other.yaw_rate and
                self.roll_angle == other.roll_angle and
                self.roll_rate == other.roll_rate and
                self.error_count == other.error_count and
                self.accel_raw == other.accel_raw and
                self.gyro_raw == other.gyro_raw)



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
