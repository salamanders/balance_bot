import logging
import time

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..watchdog import SurvivalWatchdog

logger = logging.getLogger(__name__)

class SelfDiscoveryPipeline:
    def __init__(self, steps: list[CalibrationStep], watchdog: SurvivalWatchdog) -> None:
        self.steps = steps
        self.watchdog = watchdog
        self.config = HardwareConfig.load()
        self.state = LearningState.load()

        # Instantiate HAL once.
        self.hw = RobotHardware(self.config, self.state, watchdog=self.watchdog)

    def run(self) -> None:
        logger.info("Starting Linear Self-Discovery Pipeline...")
        logger.info(f"Initial HardwareConfig: {self.config.model_dump()}")
        logger.info(f"Initial LearningState: {self.state.model_dump()}")

        for step in self.steps:
            self._run_step_with_retries(step)

        logger.info("Self-Discovery Pipeline Complete!")
        self.hw.stop()

    def _run_step_with_retries(self, step: CalibrationStep) -> None:
        max_retries = 3
        attempts = 0
        while attempts < max_retries: # Retry Loop
            attempts += 1
            if step.is_verified(self.state):
                logger.info(f"Skipping [{step.name}] - Already verified.")
                break

            logger.info(f"Running [{step.name}]...")

            # Run Step
            try:
                status, config_updates, state_updates = step.run(self.hw, self.config, self.state)
            except Exception as e:
                logger.exception(f"Unexpected error during {step.name}")
                self.hw.stop()
                raise RuntimeError(f"Pipeline failed at {step.name}: {e}")

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
                if attempts >= max_retries:
                    logger.error(f"[{step.name}] FATAL ERROR: Max retries ({max_retries}) reached. Halting pipeline.")
                    self.hw.stop()
                    raise RuntimeError(f"Pipeline halted at {step.name} after {max_retries} failed attempts")
                continue # Retry same step

            elif status == StepStatus.FATAL:
                logger.error(f"[{step.name}] FATAL ERROR. Halting pipeline.")
                self.hw.stop()
                raise RuntimeError(f"Pipeline halted at {step.name}")
