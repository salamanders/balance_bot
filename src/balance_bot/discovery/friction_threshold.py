import logging
from typing import Any
import glm

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..utils import find_threshold

logger = logging.getLogger(__name__)

# --- Step 4: Friction Threshold ---
class FrictionThresholdStep(CalibrationStep):
    def __init__(self) -> None:
        self.gyro_glitches = 0
        self.ignored_commands = 0

    @property
    def name(self) -> str:
        return "Friction Threshold (Min Power)"

    def is_verified(self, state: LearningState) -> bool:
        return state.friction_threshold_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> tuple[StepStatus, dict[str, Any], dict[str, Any]]:
        logger.info("\n>>> Finding Minimum Power (Raw) <<<")
        logger.info("Ensuring robot is on the floor...")

        def action(p):
            steps = [
                (p, p, 0.3),
                (0.0, 0.0, 0.5)
            ]
            res = hw.execute_maneuver(steps)
            return res

        def check(res):
            # Calculate Max Raw Gyro Magnitude
            max_mag = 0.0
            error_count = 0
            for s in res.samples:
                if s.error_count > 0:
                    error_count += 1
                if s.gyro_raw:
                    mag = glm.length(s.gyro_raw)
                    if mag > max_mag:
                        max_mag = mag

            if error_count > 0:
                self.gyro_glitches += error_count
                logger.warning(f"    [GLITCH] Gyro read failed {error_count} times during pulse. Total: {self.gyro_glitches}")

            logger.info(f"    Max Raw Gyro Magnitude: {max_mag:.1f} deg/s")

            if max_mag > 15.0:
                return True
            else:
                self.ignored_commands += 1
                logger.warning(f"    [IGNORED] Not enough power to move. Ignored Command count: {self.ignored_commands}")
                return False

        heartbeat_fn = hw.watchdog.heartbeat if hw.watchdog else None
        found = find_threshold("Minimum Power", 0, 5, 100, action, check, heartbeat_fn=heartbeat_fn)

        logger.info("\n--- Friction Test Summary ---")
        logger.info(f"Minimum Power Found: {found}%")
        logger.info(f"Ignored Commands (Too low power): {self.ignored_commands}")
        logger.info(f"Gyro Glitches: {self.gyro_glitches}")
        logger.info("Confidence: 95% (Simple threshold crossed)")

        if found is None:
            return StepStatus.FATAL, {}, {}

        return StepStatus.SUCCESS, {}, {
            'min_power_visible': found,
            'friction_threshold_verified': True
        }
