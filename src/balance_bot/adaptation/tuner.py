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

import statistics
from collections import deque
from typing import NamedTuple

from ..configuration import TunerConfig


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
        self.errors: deque[float] = deque(maxlen=self.buffer_size)
        self.cooldown_timer = 0
        self.current_scale = self.config.start_aggression_normal
        self.tick_counter = 0
        self._zero_crossings = 0

    def reset_aggression(self, first_run: bool) -> None:
        """Reset the tuning aggression scale based on run mode."""
        if first_run:
            self.current_scale = self.config.start_aggression_first_run
        else:
            self.current_scale = self.config.start_aggression_normal

    def get_current_scale(self) -> float:
        """Return the current aggression scale."""
        return self.current_scale

    def update(self, error: float) -> TuningAdjustment:
        """
        Add a new error sample and return PID nudges.

        :param error: Current pitch error (Target - Pitch).
        :return: TuningAdjustment(kp, ki, kd) with additive modifiers.
        """
        # Safety: If falling/crashed, do not tune and reset history.
        # We use crash_angle (default 60.0) to allow tuning while resting on training wheels (~30-40 deg)
        # during the initial startup phase.
        if abs(error) > self.config.crash_angle:
            self.errors.clear()
            self._zero_crossings = 0
            return TuningAdjustment(0.0, 0.0, 0.0)

        # If buffer is full, check if we're losing a zero crossing
        if len(self.errors) == self.buffer_size and self.buffer_size >= 2:
            oldest = self.errors[0]
            second_oldest = self.errors[1]
            if (oldest > 0 >= second_oldest) or (oldest < 0 <= second_oldest):
                self._zero_crossings -= 1

        # Check if the new error creates a zero crossing with the current newest error
        if len(self.errors) > 0:
            current_newest = self.errors[-1]
            if (current_newest > 0 >= error) or (current_newest < 0 <= error):
                self._zero_crossings += 1

        self.errors.append(error)

        # Decrement cooldown
        if self.cooldown_timer > 0:
            self.cooldown_timer -= 1
            return TuningAdjustment(0.0, 0.0, 0.0)

        # Need full buffer to analyze
        if len(self.errors) < self.buffer_size:
            return TuningAdjustment(0.0, 0.0, 0.0)

        # Optimization: Downsample expensive statistical analysis
        self.tick_counter += 1
        if self.tick_counter % self.config.analysis_interval != 0:
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
            kp_nudge = self.config.kp_oscillation_penalty * self.current_scale
            kd_nudge = self.config.kd_oscillation_boost * self.current_scale  # More damping might help
            tuned = True

        # 2. STABILITY (Improving over time)
        # If very stable (low variance) and upright, try to tighten control (Increase Kp).
        elif (
                stdev_err < self.config.stability_std_dev
                and abs(mean_err) < self.config.stability_mean_err
        ):
            kp_nudge = self.config.kp_stability_boost * self.current_scale
            tuned = True

        # 3. STEADY STATE ERROR
        # If consistently leaning, Ki is too low.
        if abs(mean_err) > self.config.steady_error_threshold:
            ki_nudge = self.config.ki_boost * self.current_scale
            tuned = True

        if tuned:
            self.cooldown_timer = self.config.cooldown_reset

        # Decay Aggression
        if self.current_scale > self.config.min_aggression:
            self.current_scale *= self.config.aggression_decay

        return TuningAdjustment(kp_nudge, ki_nudge, kd_nudge)

    def _count_zero_crossings(self) -> int:
        """Count how many times the signal crosses zero in the buffer."""
        return max(0, self._zero_crossings)


class BalancePointFinder:
    """
    Background process that adjusts the target angle (balance point)
    based on the average motor output required to stay stationary.
    """

    def __init__(self, config: TunerConfig):
        self.config = config
        self.motor_history: list[float] = []

    def update(self, motor_output: float, pitch_rate: float, aggression: float = 1.0) -> float:
        """
        Record motor output and suggest target angle adjustment.

        :param motor_output: Current average motor command.
        :param pitch_rate: Current pitch rate (deg/s) to check for stability.
        :param aggression: Multiplier for learning rate (default 1.0).
        :return: Angle adjustment (additive) to apply to target_angle.
        """
        # 1. Check stability (don't learn if wobbling)
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
        rate = self.config.balance_learning_rate * aggression

        # Logic Rule from midpoint.md:
        # If Average Motor Output > Threshold (Positive/Forward) -> Decrease Target Angle (Lean Back)
        if avg_output > self.config.balance_motor_threshold:
            adjustment = -rate

        # If Average Motor Output < -Threshold (Negative/Backward) -> Increase Target Angle (Lean Forward)
        elif avg_output < -self.config.balance_motor_threshold:
            adjustment = rate

        return adjustment
