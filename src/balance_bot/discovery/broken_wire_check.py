import logging
from typing import Tuple, Dict, Any
import glm

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware

logger = logging.getLogger(__name__)

class BrokenWireCheckStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Broken Wire Check"

    def is_verified(self, state: LearningState) -> bool:
        return state.broken_wire_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info("\n>>> Checking for Broken Wires <<<")
        logger.info("Ensuring independent movement of left and right motors...")
        hw.wait_for_stability()

        def test_motor(name: str, power_l: float, power_r: float) -> bool:
            logger.info(f"  Pulsing {name}...")

            # Pulse the motor with a short burst of high power, then coast and record
            steps = [
                (power_l, power_r, 0.2), # Power pulse
                (0.0, 0.0, 0.5),         # Settle and record
            ]
            res = hw.execute_maneuver(steps)

            if not res.samples:
                logger.error(f"  [FAILURE] No samples collected for {name}. System Glitch.")
                return False

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
                logger.warning(f"    [GLITCH] Gyro read failed {error_count} times during pulse.")

            logger.info(f"    Max Raw Gyro Magnitude: {max_mag:.1f} deg/s")

            if max_mag > 10.0:
                return True
            else:
                logger.warning("    [IGNORED] Not enough power to move or broken wire.")
                return False

        # Test power
        power = 75.0

        # 1. Left Motor
        logger.info("  Testing Left Motor...")
        left_fwd_success = test_motor("Left Motor Forward", power, 0.0)
        hw.wait_for_stability()

        if not left_fwd_success:
            left_rev_success = test_motor("Left Motor Backward", -power, 0.0)
            hw.wait_for_stability()
            if not left_rev_success:
                logger.error("\n[FATAL] Left motor failed to produce any movement in BOTH directions.")
                logger.error("When putting a lot of power into just that side, nothing happens.")
                logger.error("Putting a lot of power into just that side backwards: also nothing happens.")
                logger.error("Please check the wiring for the left motor (loose or broken wire).")
                return StepStatus.FATAL, {}, {}

        # 2. Right Motor
        logger.info("  Testing Right Motor...")
        right_fwd_success = test_motor("Right Motor Forward", 0.0, power)
        hw.wait_for_stability()

        if not right_fwd_success:
            right_rev_success = test_motor("Right Motor Backward", 0.0, -power)
            hw.wait_for_stability()
            if not right_rev_success:
                logger.error("\n[FATAL] Right motor failed to produce any movement in BOTH directions.")
                logger.error("When putting a lot of power into just that side, nothing happens.")
                logger.error("Putting a lot of power into just that side backwards: also nothing happens.")
                logger.error("Please check the wiring for the right motor (loose or broken wire).")
                return StepStatus.FATAL, {}, {}

        logger.info("\n--- Broken Wire Check Summary ---")
        logger.info("Both motors independently produced movement.")
        logger.info("Confidence: High (Both motors confirmed working)")

        return StepStatus.SUCCESS, {}, {
            'broken_wire_verified': True
        }
