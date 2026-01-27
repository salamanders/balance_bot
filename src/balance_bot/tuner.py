import statistics


class ContinuousTuner:
    COOLDOWN_RESET = 50  # Wait 0.5s between adjustments

    # Heuristic Constants
    OSCILLATION_THRESHOLD = 0.15  # % of buffer
    KP_OSCILLATION_PENALTY = -0.1
    KD_OSCILLATION_BOOST = 0.05

    STABILITY_STD_DEV = 1.0
    STABILITY_MEAN_ERR = 1.0
    KP_STABILITY_BOOST = 0.02

    STEADY_ERROR_THRESHOLD = 3.0
    KI_BOOST = 0.005

    def __init__(self, buffer_size: int = 100):
        """
        Initialize the ContinuousTuner.
        :param buffer_size: Number of error samples to keep (100 samples @ 10ms loop = 1 sec)
        """
        self.buffer_size = buffer_size
        self.errors: list[float] = []
        self.cooldown_timer = 0

    def update(self, error: float) -> tuple[float, float, float]:
        """
        Add a new error sample and return PID nudges.
        :param error: Current pitch error (Target - Pitch)
        :return: Tuple (kp_nudge, ki_nudge, kd_nudge)
        """
        self.errors.append(error)
        if len(self.errors) > self.buffer_size:
            self.errors.pop(0)

        # Decrement cooldown
        if self.cooldown_timer > 0:
            self.cooldown_timer -= 1
            return 0.0, 0.0, 0.0

        # Need full buffer to analyze
        if len(self.errors) < self.buffer_size:
            return 0.0, 0.0, 0.0

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
        if zero_crossings > (self.buffer_size * self.OSCILLATION_THRESHOLD):
            kp_nudge = self.KP_OSCILLATION_PENALTY
            kd_nudge = self.KD_OSCILLATION_BOOST  # More damping might help
            tuned = True

        # 2. STABILITY (Improving over time)
        # If very stable (low variance) and upright, try to tighten control (Increase Kp).
        elif stdev_err < self.STABILITY_STD_DEV and abs(mean_err) < self.STABILITY_MEAN_ERR:
            kp_nudge = self.KP_STABILITY_BOOST
            tuned = True

        # 3. STEADY STATE ERROR
        # If consistently leaning, Ki is too low.
        if abs(mean_err) > self.STEADY_ERROR_THRESHOLD:
            ki_nudge = self.KI_BOOST
            tuned = True

        if tuned:
            self.cooldown_timer = self.COOLDOWN_RESET

        return kp_nudge, ki_nudge, kd_nudge

    def _count_zero_crossings(self) -> int:
        crossings = 0
        for i in range(1, len(self.errors)):
            if (self.errors[i - 1] > 0 and self.errors[i] <= 0) or (
                self.errors[i - 1] < 0 and self.errors[i] >= 0
            ):
                crossings += 1
        return crossings
