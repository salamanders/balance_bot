import time
import logging
from typing import Tuple, Dict, Any, Optional

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..utils import find_threshold

logger = logging.getLogger(__name__)

# --- Step 7: Kickup Dynamics ---
class KickupDynamicsStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Kick-Up Dynamics"

    def is_verified(self, state: LearningState) -> bool:
        return state.kickup_dynamics_verified

    def _force_posture(self, hw: RobotHardware, target_sign: float, base_power: float, state: LearningState) -> None:
        """Helper to force the robot into a specific posture (+1 for BACK, -1 for FRONT)."""
        target_name = "BACK" if target_sign > 0 else "FRONT"

        logger.info(f"  [Posture Check] Verifying initial physical state before trying to flop {target_name}...")
        hw.wait_for_stability(1.0)

        # Take multiple readings to be absolutely sure we're not just bouncing
        samples = []
        for _ in range(5):
            samples.append(hw.read_imu_converted().pitch_angle)
            time.sleep(0.05)

        avg_pitch = sum(samples) / len(samples)
        pitch_variance = max(samples) - min(samples)

        logger.info(f"  [Posture Check] Measured average pitch: {avg_pitch:.1f}° (Variance: {pitch_variance:.1f}°)")

        # Define a high-confidence resting threshold margin around the calibrated resting angles
        RESTING_MARGIN = 15.0

        # Check if we are already securely flopped in the correct direction
        if target_sign > 0 and abs(avg_pitch - state.rest_angle_backward) < RESTING_MARGIN and pitch_variance < 5.0:
            logger.info(f"  [Posture Check] I am REALLY sure I'm on my BACK. I know this because stable pitch is {avg_pitch:.1f}° (Target: {state.rest_angle_backward:.1f}°). No flop needed.")
            return
        elif target_sign < 0 and abs(avg_pitch - state.rest_angle_forward) < RESTING_MARGIN and pitch_variance < 5.0:
            logger.info(f"  [Posture Check] I am REALLY sure I'm on my FRONT. I know this because stable pitch is {avg_pitch:.1f}° (Target: {state.rest_angle_forward:.1f}°). No flop needed.")
            return

        logger.info(f"  [Posture Check] Current pitch is {avg_pitch:.1f}°. Need to actively flop to {target_name}. Initiating maneuver...")

        # Increment power until we reach max_power
        p = base_power
        while p <= 100:
            logger.info(f"  Flop to {target_name} (Power {p:.0f}%)...")
            motor_p = p * target_sign

            # Ramp
            steps = []
            ramp_steps = 5
            for i in range(1, ramp_steps + 1):
                factor = i / ramp_steps
                steps.append((motor_p * factor, motor_p * factor, 0.05))
            steps.append((motor_p, motor_p, 0.4 - 0.25))

            # Ramp down
            for i in range(ramp_steps - 1, -1, -1):
                factor = i / ramp_steps
                steps.append((motor_p * factor, motor_p * factor, 0.05))

            hw.execute_maneuver(steps)

            hw.wait_for_stability(1.0)

            # Post-maneuver verification
            samples_post = []
            for _ in range(3):
                samples_post.append(hw.read_imu_converted().pitch_angle)
                time.sleep(0.05)

            avg_pitch_post = sum(samples_post) / len(samples_post)

            if target_sign > 0 and abs(avg_pitch_post - state.rest_angle_backward) < RESTING_MARGIN:
                logger.info(f"  [Posture Check] Flop successful. Settled at {avg_pitch_post:.1f}°")
                return
            if target_sign < 0 and abs(avg_pitch_post - state.rest_angle_forward) < RESTING_MARGIN:
                logger.info(f"  [Posture Check] Flop successful. Settled at {avg_pitch_post:.1f}°")
                return

            # Revert movement to avoid hitting walls
            logger.info("  [Posture Check] Flop failed. Reversing to start position...")
            reverse_steps = [(-l_p, -r_p, d) for l_p, r_p, d in steps]
            hw.execute_maneuver(reverse_steps)
            hw.wait_for_stability(1.0)

            p += 10.0

        raise RuntimeError("Failed to posture")

    def _attempt_kick(self, hw: RobotHardware, start_sign: float, p: float) -> str:
        """Helper to attempt a kick-up maneuver. start_sign: +1 for BACK, -1 for FRONT."""
        kick_sign = -start_sign
        setup_sign = -kick_sign

        setup_p = p * setup_sign * 0.7
        kick_p = p * kick_sign * 1.0

        # Ramp setup power
        steps = []
        ramp_steps = 5
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps.append((setup_p * factor, setup_p * factor, 0.05))

        # Setup hold
        steps.append((setup_p, setup_p, 0.3 - 0.25))

        # Ramp kick power (fast)
        for i in range(1, 3):
            factor = i / 2.0
            steps.append((kick_p * factor, kick_p * factor, 0.02))

        # Kick hold
        steps.append((kick_p, kick_p, 0.4 - 0.04))

        # Ramp kick power down (fast)
        for i in range(1, -1, -1):
            factor = i / 2.0
            steps.append((kick_p * factor, kick_p * factor, 0.02))

        hw.execute_maneuver(steps)

        time.sleep(1.0)
        pitch = hw.read_imu_converted().pitch_angle

        # Check Success
        if start_sign > 0 and -10 < pitch < 20:
            return "SUCCESS"
        if start_sign < 0 and -20 < pitch < 10:
            return "SUCCESS"
        return "FAIL"

    def _run_kickup_test(self, hw: RobotHardware, state: LearningState, direction_sign: float) -> Optional[float]:
        """Runs the threshold search for a specific kick-up direction. direction_sign: +1 for BACK, -1 for FRONT."""
        dir_name = "BACK" if direction_sign > 0 else "FRONT"

        def action(p):
            try:
                self._force_posture(hw, direction_sign, state.min_power_visible + 10, state)
            except RuntimeError:
                return "POSTURE_FAIL"
            return self._attempt_kick(hw, direction_sign, p)

        return find_threshold(f"KickUp {dir_name}",
                               0,
                               5, 100,
                               action,
                               lambda r: r == "SUCCESS",
                               heartbeat_fn=hw.watchdog.heartbeat if hw.watchdog else None)

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info(">>> Dynamic Kick-Up Calibration <<<")

        fwd = self._run_kickup_test(hw, state, 1.0) # Kickup Forward (from Back, +1)
        if fwd is None:
            return StepStatus.NEEDS_RETRY, {}, {}

        bwd = self._run_kickup_test(hw, state, -1.0) # Kickup Backward (from Front, -1)
        if bwd is None:
            return StepStatus.NEEDS_RETRY, {}, {}

        # Construct new ControlConfig
        new_control = state.control.model_copy(update={
            'kickup_power_forward': fwd,
            'kickup_power_backward': bwd
        })

        return StepStatus.SUCCESS, {}, {
            'control': new_control,
            'kickup_dynamics_verified': True
        }
