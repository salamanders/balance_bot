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
