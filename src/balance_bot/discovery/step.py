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

from enum import Enum, auto
from typing import Protocol, Any

from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware


class StepStatus(Enum):
    SUCCESS = auto()
    NEEDS_RETRY = auto()
    FATAL = auto()


class CalibrationStep(Protocol):
    """
    A single, isolated step in the robot's physical self-discovery pipeline.
    """

    @property
    def name(self) -> str:
        """Name of the step for logging."""
        ...

    def is_verified(self, state: LearningState) -> bool:
        """
        Check exactly ONE boolean flag in LearningState.
        """
        ...

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> tuple[
        StepStatus, dict[str, Any], dict[str, Any]]:
        """
        Execute the routine.
        Returns:
            - status: StepStatus
            - proposed_config_updates: dict to update HardwareConfig
            - proposed_state_updates: dict to update LearningState
        """
        ...
