from typing import Protocol, Tuple
import logging
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..watchdog import SurvivalWatchdog

logger = logging.getLogger(__name__)

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

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState, watchdog: SurvivalWatchdog) -> Tuple[HardwareConfig, bool]:
        """
        Execute the routine.
        MUST return (new_config, True) on success, or (config, False) to force a retry.
        NEVER mutate `config` or `state` until the very last line before returning True.
        """
        ...
