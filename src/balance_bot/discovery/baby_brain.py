import time
import logging
import traceback
from typing import List, Optional

from .knowledge_graph import DiscoveryContext
from .types import Atom, ExperimentResult
from .experiments import (
    Experiment,
    ExpPulse,
    ExpMeditation,
    ExpTwitch,
    ExpCrunch,
    ExpWiggle,
    ExpAttempt,
    ExpPirouette,
    ExpStride
)
from ..hardware.robot_hardware import RobotHardware
from ..config import RobotConfig

logger = logging.getLogger(__name__)

class DiscoveryBrain:
    """
    The "Proprioceptive Toddler" Brain.
    Dependency-Driven Discovery Engine.
    """

    def __init__(self, context: DiscoveryContext):
        self.context = context
        self.experiments: List[Experiment] = [
            ExpPulse(),
            ExpMeditation(),
            ExpTwitch(),
            ExpCrunch(),
            ExpWiggle(),
            ExpAttempt(),
            ExpPirouette(),
            ExpStride()
        ]
        self.hw: Optional[RobotHardware] = None

    def _update_hardware(self):
        """
        Re-initialize hardware with latest knowledge.
        """
        if not self.context.has_atom(Atom.HARDWARE_BUS):
            self.hw = None
            return

        try:
            c = self.context.build_config()

            # If we already have HW, we should close it first?
            # Or just update it? RobotHardware isn't easily updateable.
            # Re-creating is safer for discovery.
            if self.hw:
                try:
                    self.hw.stop()
                    # self.hw.cleanup() # Might close I2C bus?
                    # RobotHardware.cleanup just calls pz.cleanup().
                    # We can probably reuse the bus instance if we were clever, but
                    # for now let's be safe and re-init.
                except Exception:
                    pass

            # Create new instance
            self.hw = c.to_hardware(force_defaults=False)

        except ValueError:
            self.hw = None
        except Exception as e:
            logger.error(f"Failed to init hardware: {e}")
            self.hw = None

    def _ensure_upright(self):
        """
        Block until the robot is upright/stable.
        This handles 'Trauma' (Falls).
        """
        if not self.hw: return

        # Check if we have enough knowledge to know "Upright"
        if not self.context.has_atom(Atom.GRAVITY_VECTOR):
            return # Can't know.

        # We need raw data to check orientation vs Gravity Vector
        logger.info("Checking posture...")

        while True:
            try:
                # We assume Hardware is initialized enough to read raw.
                a, g = self.hw.read_imu_raw()

                # Check Stability (Gyro)
                rate = abs(g.x) + abs(g.y) + abs(g.z)

                # Check Orientation (Accel vs Gravity)
                grav = self.context.get(Atom.GRAVITY_VECTOR)

                # If we are upright, Accel ~ Gravity.
                # Cos(Theta) = (A . G) / (|A|*|G|)
                # If A ~ G, Dot Product is high.

                dot = (a.x * grav.x) + (a.y * grav.y) + (a.z * grav.z)
                # Magnitude approx 1G (9.8 or 16384).
                mag_a = (a.x**2 + a.y**2 + a.z**2)**0.5
                mag_g = (grav.x**2 + grav.y**2 + grav.z**2)**0.5

                if mag_a * mag_g == 0: cos_theta = 0
                else: cos_theta = dot / (mag_a * mag_g)

                # cos(0) = 1. cos(60) = 0.5.
                # If cos_theta < 0.5, we are crashed (>60 deg).

                is_crashed = cos_theta < 0.5
                is_moving = rate > 20.0 # Raw units? deg/s?

                if is_crashed:
                    logger.warning(f"I have fallen! (CosTheta={cos_theta:.2f}). Please stand me up.")
                    time.sleep(2.0)
                    continue

                if is_moving:
                    logger.info(f"Stabilizing... (Rate={rate:.1f})")
                    time.sleep(0.5)
                    continue

                # Upright and Stable
                return

            except Exception as e:
                logger.error(f"Error checking posture: {e}")
                time.sleep(1.0)

    def think(self):
        logger.info("Brain Online. Starting Discovery Loop.")

        while True:
            # 1. State Estimation
            # Re-init hardware with latest knowledge
            self._update_hardware()

            # Safety Check
            if self.hw:
                self._ensure_upright()

            # 2. Candidate Selection
            candidates = []
            for exp in self.experiments:
                if exp.can_run(self.context) and not exp.has_result(self.context):
                    candidates.append(exp)

            if not candidates:
                # Check if we are done?
                # Or stuck?
                # If we have all critical atoms, we are done.
                if self.context.has_atom(Atom.TRIM_CALIBRATION):
                    logger.info("All experiments complete! I have learned everything.")
                    self.graduation()
                    break
                else:
                    logger.error("I am stuck. I don't know enough to proceed, but have no candidates.")
                    # Debug: what is missing?
                    for exp in self.experiments:
                        if not exp.has_result(self.context):
                             missing = [a.name for a in exp.required_atoms if not self.context.has_atom(a)]
                             logger.info(f"  Cannot run {exp.name}: Missing {missing}")
                    break

            # 3. Prioritization
            # Simple list order is priority.
            best_exp = candidates[0]

            # 4. Execution
            logger.info(f"\n[THOUGHT] I am curious about... {best_exp.name}")
            try:
                result = best_exp.run(self.context, self.hw)
            except Exception as e:
                logger.error(f"Experiment {best_exp.name} crashed: {e}")
                traceback.print_exc()
                time.sleep(2.0)
                continue

            # 5. Integration
            if result.success:
                logger.info(f"[LEARNED] {best_exp.name} successful.")
                self.context.update(result.data)
            else:
                logger.warning(f"[FAILURE] {best_exp.name} failed: {result.error}")
                if result.retry_suggested:
                    logger.info("Retrying in 2 seconds...")
                    time.sleep(2.0)
                else:
                    # If fatal failure, abort?
                    # Or loop and try again?
                    logger.error("Fatal experiment failure. Aborting.")
                    break

            time.sleep(1.0)

    def graduation(self):
        """
        Save final config and exit.
        """
        try:
            config = self.context.build_config()
            config.save()
            logger.info("Robot Configuration Saved to pid_config.json")
            logger.info("I am ready to be an adult. Run me without --discover.")
        except Exception as e:
            logger.error(f"Failed to graduate: {e}")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    ctx = DiscoveryContext()
    brain = DiscoveryBrain(ctx)
    brain.think()
