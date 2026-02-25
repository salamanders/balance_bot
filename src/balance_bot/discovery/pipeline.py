import logging
import time
from typing import List, Dict, Any

from .step import CalibrationStep, StepStatus
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
            while True: # Retry Loop
                if step.is_verified(self.state):
                    logger.info(f"Skipping [{step.name}] - Already verified.")
                    break

                logger.info(f"Running [{step.name}]...")

                # Run Step
                status, config_updates, state_updates = step.run(self.hw, self.config, self.state)

                if status == StepStatus.SUCCESS:
                    logger.info(f"[{step.name}] Succeeded.")

                    # Apply Updates to LearningState (Mutable)
                    if state_updates:
                        for k, v in state_updates.items():
                            setattr(self.state, k, v)
                        self.state.save()

                    # Apply Updates to HardwareConfig (Immutable - Replace)
                    if config_updates:
                        # Create new config instance with updates
                        new_config = self.config.model_copy(update=config_updates)
                        if new_config != self.config:
                            logger.info(f"Applying new hardware config from {step.name}")
                            new_config.save()
                            self.config = new_config
                            self.hw.apply_config(self.config)

                    break # Move to next step

                elif status == StepStatus.NEEDS_RETRY:
                    logger.warning(f"[{step.name}] Requested Retry. Stabilizing...")
                    self.hw.stop()
                    time.sleep(2.0)
                    continue # Retry same step

                elif status == StepStatus.FATAL:
                    logger.error(f"[{step.name}] FATAL ERROR. Halting pipeline.")
                    self.hw.stop()
                    raise RuntimeError(f"Pipeline halted at {step.name}")

        logger.info("Self-Discovery Pipeline Complete!")
        self.hw.stop()
