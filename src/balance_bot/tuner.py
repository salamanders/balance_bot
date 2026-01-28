import statistics
from itertools import pairwise
from typing import NamedTuple

from .config import TunerConfig


class TuningAdjustment(NamedTuple):
    """
    Represents the small adjustments (nudges) to apply to the PID parameters.
    """

    kp: float
    ki: float
    kd: float


class ContinuousTuner:
    """
    Analyzes the history of error signals to automatically adjust PID parameters in real-time.

    Strategy:
    1. Detect Oscillation: If error crosses zero too frequently, Kp is too high (or Kd too low).
    2. Detect Stability: If error is consistently low and steady, we can increase Kp to stiffen response.
    3. Detect Drift: If there is a constant non-zero mean error, we need more Ki to correct it.
    """

    def __init__(self, config: TunerConfig = TunerConfig(), buffer_size: int = 100):
        """
        Initialize the ContinuousTuner.
        :param config: Tuner configuration object (thresholds, etc).
        :param buffer_size: Number of error samples to keep in memory.
                            100 samples @ 10ms loop = 1 second of history.
        """
        self.config = config
        self.buffer_size = buffer_size
        self.errors: list[float] = []
        self.cooldown_timer = 0

    def update(self, error: float) -> TuningAdjustment:
        """
        Add a new error sample to the buffer and calculate PID nudges.

        :param error: Current pitch error (Target Angle - Measured Angle). Unit: Degrees.
        :return: TuningAdjustment tuple containing (kp_delta, ki_delta, kd_delta).
        """
        self.errors.append(error)
        if len(self.errors) > self.buffer_size:
            self.errors.pop(0)

        # Decrement cooldown (prevent rapid-fire changes)
        if self.cooldown_timer > 0:
            self.cooldown_timer -= 1
            return TuningAdjustment(0.0, 0.0, 0.0)

        # Need full buffer to analyze trends reliably
        if len(self.errors) < self.buffer_size:
            return TuningAdjustment(0.0, 0.0, 0.0)

        # Calculate Statistics
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

        # 1. OSCILLATION CHECK (High Frequency Instability)
        # If the robot is shaking back and forth, it crosses the setpoint frequently.
        # If >15% of samples are crossing zero, Kp is likely too high (system is over-reactive).
        if zero_crossings > (self.buffer_size * self.config.oscillation_threshold):
            kp_nudge = self.config.kp_oscillation_penalty  # Reduce Kp
            kd_nudge = (
                self.config.kd_oscillation_boost
            )  # Increase Kd (Damping) to stop shaking
            tuned = True

        # 2. STABILITY CHECK (Improving Performance)
        # If the robot is very calm (low standard deviation) and near vertical (low mean),
        # we can try to stiffen the control by slightly increasing Kp.
        elif (
            stdev_err < self.config.stability_std_dev
            and abs(mean_err) < self.config.stability_mean_err
        ):
            kp_nudge = self.config.kp_stability_boost
            tuned = True

        # 3. STEADY STATE ERROR CHECK (Leaning)
        # If the robot is stable but consistently leaning to one side (high mean error),
        # the Integral term (Ki) is too weak to overcome gravity/imbalance.
        if abs(mean_err) > self.config.steady_error_threshold:
            ki_nudge = self.config.ki_boost
            tuned = True

        if tuned:
            self.cooldown_timer = self.config.cooldown_reset

        return TuningAdjustment(kp_nudge, ki_nudge, kd_nudge)

    def _count_zero_crossings(self) -> int:
        """Count how many times the error signal changes sign in the buffer."""
        crossings = 0
        for e1, e2 in pairwise(self.errors):
            if (e1 > 0 and e2 <= 0) or (e1 < 0 and e2 >= 0):
                crossings += 1
        return crossings
