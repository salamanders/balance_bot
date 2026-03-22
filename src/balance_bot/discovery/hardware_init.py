import logging
from typing import Any

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware

logger = logging.getLogger(__name__)

class HardwareInitStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Initialize Hardware Drivers"

    def is_verified(self, state: LearningState) -> bool:
        return state.hardware_init_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> tuple[StepStatus, dict[str, Any], dict[str, Any]]:
        logger.info("Initializing Hardware Drivers...")
        # Force driver initialization now that buses are known
        try:
            hw.initialize_drivers()
        except Exception as e:
            logger.error(f"  [FAILURE] Driver init raised exception: {e}")
            return StepStatus.FATAL, {}, {}

        # Verify they are alive
        if hw.pz is None or hw.sensor is None:
            logger.error("  [FAILURE] Drivers failed to initialize despite known buses.")
            return StepStatus.FATAL, {}, {}

        try:
            hw.init() # Init the motor driver specifically
        except Exception as e:
            logger.error(f"  [FAILURE] Motor driver init failed: {e}")
            return StepStatus.FATAL, {}, {}

        return StepStatus.SUCCESS, {}, {'hardware_init_verified': True}
