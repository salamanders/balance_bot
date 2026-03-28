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

import logging

from ..configuration import STARTUP_RAMP_SPEED, ControlConfig

logger = logging.getLogger(__name__)


class RecoveryManager:
    """
    Manages the transition from Crashed/Resting to Balanced.
    Implements a 'Soft Start' ramp to prevent violent jerks when picking up the robot.
    """

    def __init__(self, config: ControlConfig | None = None):
        self.recovering = False
        self.ramp_setpoint = 0.0
        self.config = config if config else ControlConfig()

    def update(self, is_crashed: bool, current_pitch: float, current_kp: float) -> float | None:
        """
        Update recovery state.

        :param is_crashed: True if Tier 1 reports a crash state.
        :param current_pitch: Current pitch angle.
        :param current_kp: Current P-gain (used to decide if soft start is needed).
        :return: None if normal balancing should proceed.
                 Float value (override angle) if recovering.
        """
        # 1. If currently crashed, we are not recovering yet.
        if is_crashed:
            self.recovering = False
            return None

        # 2. If we were crashed/idle, and now we are NOT crashed (Tier 1 is running),
        #    we check if we need to start a recovery sequence.
        if not self.recovering:
            # We just woke up.
            # Logic from old main.py:
            # If Kp is high (normal operation) and we are leaning significantly, assume we were just picked up.
            if current_kp >= self.config.soft_recovery_kp_threshold and abs(
                    current_pitch) > self.config.upright_threshold:
                logger.info(f"-> Starting Soft Recovery from {current_pitch:.1f} deg")
                self.recovering = True
                self.ramp_setpoint = current_pitch
            else:
                # Immediate start (Learning mode or already upright)
                return None

        # 3. Process Ramp
        if self.recovering:
            # Move setpoint towards 0
            if self.ramp_setpoint > 0:
                self.ramp_setpoint -= STARTUP_RAMP_SPEED
                if self.ramp_setpoint < 0:
                    self.ramp_setpoint = 0.0
            else:
                self.ramp_setpoint += STARTUP_RAMP_SPEED
                if self.ramp_setpoint > 0:
                    self.ramp_setpoint = 0.0

            # Check completion
            # If ramp is near zero and robot is near zero
            if abs(self.ramp_setpoint) < 0.1 and abs(current_pitch) < self.config.upright_threshold:
                logger.info("-> Recovery Complete.")
                self.recovering = False
                return None

            return self.ramp_setpoint

        return None
