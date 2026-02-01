import statistics
from itertools import pairwise
from typing import NamedTuple

from .config import TunerConfig, CRASH_ANGLE


class TuningAdjustment(NamedTuple):
    kp: float
    ki: float
    kd: float


class ContinuousTuner:
    """
    Background process that monitors control performance and suggests PID tweaks.

    Analyses a buffer of recent error history to detect:
     1. High Frequency Oscillation -> Reduce Kp, Increase Kd.
     2. Stable Upright Behavior -> Gently Increase Kp.
     3. Persistent Leaning -> Increase Ki.
    """

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

    def update(self, error: float, scale: float = 1.0) -> TuningAdjustment:
        """
        Add a new error sample and return PID nudges.

        :param error: Current pitch error (Target - Pitch).
        :param scale: Multiplier for the tuning adjustment (aggressiveness).
        :return: TuningAdjustment(kp, ki, kd) with additive modifiers.
        """
        # Safety: If falling/crashed, do not tune and reset history.
        # We use CRASH_ANGLE (60.0) to allow tuning while resting on training wheels (~30-40 deg)
        # during the initial startup phase.
        if abs(error) > CRASH_ANGLE:
            self.errors.clear()
            return TuningAdjustment(0.0, 0.0, 0.0)

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
            kp_nudge = self.config.kp_oscillation_penalty * scale
            kd_nudge = self.config.kd_oscillation_boost * scale  # More damping might help
            tuned = True

        # 2. STABILITY (Improving over time)
        # If very stable (low variance) and upright, try to tighten control (Increase Kp).
        elif (
            stdev_err < self.config.stability_std_dev
            and abs(mean_err) < self.config.stability_mean_err
        ):
            kp_nudge = self.config.kp_stability_boost * scale
            tuned = True

        # 3. STEADY STATE ERROR
        # If consistently leaning, Ki is too low.
        if abs(mean_err) > self.config.steady_error_threshold:
            ki_nudge = self.config.ki_boost * scale
            tuned = True

        if tuned:
            self.cooldown_timer = self.config.cooldown_reset

        return TuningAdjustment(kp_nudge, ki_nudge, kd_nudge)

    def _count_zero_crossings(self) -> int:
        """Count how many times the signal crosses zero in the buffer."""
        crossings = 0
        for e1, e2 in pairwise(self.errors):
            if (e1 > 0 and e2 <= 0) or (e1 < 0 and e2 >= 0):
                crossings += 1
        return crossings


class BalancePointFinder:
    """
    Background process that adjusts the target angle (balance point)
    based on the average motor output required to stay stationary.
    """

    def __init__(self, config: TunerConfig):
        self.config = config
        self.motor_history: list[float] = []
        self.cooldown_timer = 0

    def update(self, motor_output: float, pitch_rate: float) -> float:
        """
        Record motor output and suggest target angle adjustment.

        :param motor_output: Current average motor command.
        :param pitch_rate: Current pitch rate (deg/s) to check for stability.
        :return: Angle adjustment (additive) to apply to target_angle.
        """
        # 1. Decrement cooldown
        if self.cooldown_timer > 0:
            self.cooldown_timer -= 1
            return 0.0

        # 2. Check stability (don't learn if wobbling)
        # We only accumulate samples when the robot is relatively stable.
        if abs(pitch_rate) > self.config.balance_pitch_rate_threshold:
            return 0.0

        # 3. Add to history
        self.motor_history.append(motor_output)

        # 4. Check buffer size (wait until we have enough samples)
        if len(self.motor_history) < self.config.balance_check_interval:
            return 0.0

        # 5. Analyze
        avg_output = statistics.mean(self.motor_history)
        self.motor_history.clear()  # Reset buffer after analysis

        adjustment = 0.0

        # Logic Rule from midpoint.md:
        # If Average Motor Output > Threshold (Positive/Forward) -> Decrease Target Angle (Lean Back)
        if avg_output > self.config.balance_motor_threshold:
            adjustment = -self.config.balance_learning_rate

        # If Average Motor Output < -Threshold (Negative/Backward) -> Increase Target Angle (Lean Forward)
        elif avg_output < -self.config.balance_motor_threshold:
            adjustment = self.config.balance_learning_rate

        # If we made an adjustment, set cooldown (optional, but good practice)
        # Note: clearing the buffer already creates a delay equal to balance_check_interval samples.
        # But we might want an extra pause?
        # The logic below relies on the buffer filling up again, so no explicit cooldown needed unless we want longer.
        # But let's verify if `balance_check_interval` is enough. 5 seconds is a long time.
        # midpoint.md says "Updates should happen perhaps once every 5-10 seconds".
        # So relying on buffer refill is consistent with that.

        return adjustment
