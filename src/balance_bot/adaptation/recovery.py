import logging
from ..config import STARTUP_RAMP_SPEED

logger = logging.getLogger(__name__)

class RecoveryManager:
    """
    Manages the transition from Crashed/Resting to Balanced.
    Implements a 'Soft Start' ramp to prevent violent jerks when picking up the robot.
    """

    def __init__(self):
        self.recovering = False
        self.ramp_setpoint = 0.0

    def update(self, is_crashed: bool, current_pitch: float, current_kp: float) -> float | None:
        """
        Update recovery state.

        :param is_crashed: True if Tier 1 reports a crash state.
        :param current_pitch: Current pitch angle.
        :param current_kp: Current P-gain (used to decide if soft start is needed).
        :return: None if normal balancing should proceed.
                 Float value (override angle) if recovering.
        """
        # 1. If currently crashed, we are not recovering yet.
        if is_crashed:
            self.recovering = False
            return None

        # 2. If we were crashed/idle and now we are NOT crashed (Tier 1 is running),
        #    we check if we need to start a recovery sequence.
        if not self.recovering:
            # We just woke up.
            # Logic from old main.py:
            # If Kp is high (normal operation) and we are leaning significantly, assume we were just picked up.
            if current_kp >= 1.0 and abs(current_pitch) > 5.0:
                logger.info(f"-> Starting Soft Recovery from {current_pitch:.1f} deg")
                self.recovering = True
                self.ramp_setpoint = current_pitch
            else:
                # Immediate start (Learning mode or already upright)
                return None

        # 3. Process Ramp
        if self.recovering:
            # Move setpoint towards 0
            if self.ramp_setpoint > 0:
                self.ramp_setpoint -= STARTUP_RAMP_SPEED
                if self.ramp_setpoint < 0:
                    self.ramp_setpoint = 0.0
            else:
                self.ramp_setpoint += STARTUP_RAMP_SPEED
                if self.ramp_setpoint > 0:
                    self.ramp_setpoint = 0.0

            # Check completion
            # If ramp is near zero and robot is near zero
            if abs(self.ramp_setpoint) < 0.1 and abs(current_pitch) < 5.0:
                logger.info("-> Recovery Complete.")
                self.recovering = False
                return None

            return self.ramp_setpoint

        return None
