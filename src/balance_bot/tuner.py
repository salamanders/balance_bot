import statistics
from itertools import pairwise
from typing import NamedTuple

from .config import TunerConfig


class TuningAdjustment(NamedTuple):
    kp: float
    ki: float
    kd: float


class ContinuousTuner:
    def __init__(self, config: TunerConfig = TunerConfig(), buffer_size: int = 100):
        """
        Initialize the ContinuousTuner.
        :param config: Tuner configuration object.
        :param buffer_size: Number of error samples to keep (100 samples @ 10ms loop = 1 sec)
        """
        self.config = config
        self.buffer_size = buffer_size
        self.errors: list[float] = []
        self.cooldown_timer = 0

    def update(self, error: float) -> TuningAdjustment:
        """
        Add a new error sample and return PID nudges.
        :param error: Current pitch error (Target - Pitch)
        :return: TuningAdjustment(kp, ki, kd)
        """
        self.errors.append(error)
        if len(self.errors) > self.buffer_size:
            self.errors.pop(0)

        # Decrement cooldown
        if self.cooldown_timer > 0:
            self.cooldown_timer -= 1
            return TuningAdjustment(0.0, 0.0, 0.0)

        # Need full buffer to analyze
        if len(self.errors) < self.buffer_size:
            return TuningAdjustment(0.0, 0.0, 0.0)

        # Analyze History
        mean_err = statistics.mean(self.errors)
        try:
            stdev_err = statistics.stdev(self.errors)
        except statistics.StatisticsError:
            stdev_err = 0.0

        zero_crossings = self._count_zero_crossings()

        kp_nudge = 0.0
        ki_nudge = 0.0
        kd_nudge = 0.0
        tuned = False

        # --- HEURISTICS ---

        # 1. OSCILLATION (High Frequency)
        # If crossing zero frequently (>15% of samples), Kp is likely too high.
        if zero_crossings > (self.buffer_size * self.config.oscillation_threshold):
            kp_nudge = self.config.kp_oscillation_penalty
            kd_nudge = self.config.kd_oscillation_boost  # More damping might help
            tuned = True

        # 2. STABILITY (Improving over time)
        # If very stable (low variance) and upright, try to tighten control (Increase Kp).
        elif (
            stdev_err < self.config.stability_std_dev
            and abs(mean_err) < self.config.stability_mean_err
        ):
            kp_nudge = self.config.kp_stability_boost
            tuned = True

        # 3. STEADY STATE ERROR
        # If consistently leaning, Ki is too low.
        if abs(mean_err) > self.config.steady_error_threshold:
            ki_nudge = self.config.ki_boost
            tuned = True

        if tuned:
            self.cooldown_timer = self.config.cooldown_reset

        return TuningAdjustment(kp_nudge, ki_nudge, kd_nudge)

    def _count_zero_crossings(self) -> int:
        crossings = 0
        for e1, e2 in pairwise(self.errors):
            if (e1 > 0 and e2 <= 0) or (e1 < 0 and e2 >= 0):
                crossings += 1
        return crossings
