from enum import Enum, auto
from abc import ABC, abstractmethod
from typing import Tuple, Dict, Any
import logging
import time

from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware

class StepStatus(Enum):
    SUCCESS = auto()
    NEEDS_RETRY = auto()
    FATAL = auto()

class BaseCalibrationStep(ABC):
    """
    Abstract base class for a single, isolated step in the robot's physical self-discovery pipeline.
    """

    @property
    @abstractmethod
    def name(self) -> str:
        """Name of the step for logging."""
        pass

    @abstractmethod
    def is_verified(self, state: LearningState) -> bool:
        """
        Check verification flags in LearningState.
        """
        pass

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        """
        Execute the routine.
        Returns:
            - status: StepStatus
            - proposed_config_updates: dict to update HardwareConfig
            - proposed_state_updates: dict to update LearningState
        """
        # Logging header is handled by pipeline mostly, but steps had their own ">>> Name <<<" prints.
        # I'll keep the internal print for consistency with the "interactive" feel.
        # self.log(f">>> {self.name} <<<")
        # actually pipeline.py logs "Running [Name]..."
        # But steps.py had `print(">>> Name <<<")` inside run.
        # I'll move that here.

        try:
            return self._run_impl(hw, config, state)
        except Exception as e:
            self.log(f"[ERROR] Exception in step {self.name}: {e}")
            import traceback
            traceback.print_exc()
            return StepStatus.FATAL, {}, {}

    @abstractmethod
    def _run_impl(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        pass

    def log(self, msg: str):
        # Using print for console output during calibration
        print(f"  {msg}")

    def wait_for_stability(self, hw: RobotHardware, duration: float = 1.0):
        # self.log("Waiting for stability...")
        # hw.wait_for_stability prints dots, so we might not want to spam log here.
        hw.wait_for_stability(duration)
