import logging
from typing import List
from .step import CalibrationStep
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..watchdog import SurvivalWatchdog

logger = logging.getLogger(__name__)

class SelfDiscoveryPipeline:
    def __init__(self, steps: List[CalibrationStep], watchdog: SurvivalWatchdog):
        self.steps = steps
        self.watchdog = watchdog
        self.config = HardwareConfig.load()
        self.state = LearningState.load()

        # Instantiate HAL once.
        self.hw = RobotHardware(self.config, self.state, watchdog=self.watchdog)

    def run(self):
        logger.info("Starting Linear Self-Discovery Pipeline...")

        for step in self.steps:
            if step.is_verified(self.state):
                logger.info(f"Skipping [{step.name}] - Already verified.")
                continue

            logger.info(f"Running [{step.name}]...")

            new_config, success = step.run(self.hw, self.config, self.state, self.watchdog)

            if success:
                logger.info(f"[{step.name}] Succeeded. Saving atomic state.")
                self.state.save()

                if new_config != self.config:
                    new_config.save()
                    self.config = new_config
                    self.hw.apply_config(self.config)
            else:
                logger.error(f"[{step.name}] Failed. Halting pipeline.")
                raise RuntimeError(f"Pipeline halted at {step.name}")

        logger.info("Self-Discovery Pipeline Complete!")
        self.hw.stop()
