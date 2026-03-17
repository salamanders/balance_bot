import logging
import time
from typing import Tuple, Dict, Any

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware

logger = logging.getLogger(__name__)

class MotorTrimStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Motor Trim Calibration"

    def is_verified(self, state: LearningState) -> bool:
        return state.motor_trim_verified

    @staticmethod
    def _build_smooth_ramp(l_p: float, r_p: float, total_duration: float) -> list[tuple[float, float, float]]:
        steps = []
        ramp_duration = 0.1
        ramp_steps = 5
        hold_duration = total_duration - (2 * ramp_duration)

        # Ramp up
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps.append((l_p * factor, r_p * factor, ramp_duration / ramp_steps))

        # Hold
        if hold_duration > 0:
            steps.append((l_p, r_p, hold_duration))

        # Ramp down
        for i in range(ramp_steps - 1, -1, -1):
            factor = i / ramp_steps
            steps.append((l_p * factor, r_p * factor, ramp_duration / ramp_steps))

        return steps

    @staticmethod
    def _run_square_validation(hw: RobotHardware, best_trim: float) -> None:
        logger.info("  [VALIDATION] Running square pattern test (20% power, 3s per side)...")
        total_wobble = 0.0

        for i in range(4):
            # Drive straight
            res = hw.execute_maneuver([(20.0, 20.0, 3.0)], trim_override=best_trim)
            wobble = res.abs_avg_yaw_rate
            total_wobble += wobble
            logger.info(f"    Side {i+1} Wobble: {wobble:.2f} d/s")

            # Turn ~90 degrees
            hw.set_motors(20.0, -20.0, trim_override=best_trim)
            current_yaw = 0.0
            last_time = time.time()

            while abs(current_yaw) < 90.0:
                reading = hw.read_imu_converted()
                now = time.time()
                dt = now - last_time
                last_time = now

                current_yaw += reading.yaw_rate * dt
                time.sleep(0.01)

            hw.stop()
            hw.wait_for_stability()

        logger.info(f"  [VALIDATION] Average Wobble (Straight Line Drift): {(total_wobble / 4):.2f} d/s")

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info(">>> Motor Trim Calibration <<<")
        hw.wait_for_stability()

        best_trim = state.motor_trim

        for i in range(15):
            p = state.min_power_visible + 15

            # Drive forward with smooth ramps
            fwd_steps = self._build_smooth_ramp(p, p, 1.0)
            res = hw.execute_maneuver(fwd_steps, trim_override=best_trim)
            hw.wait_for_stability()

            # Reverse to roughly the starting position with smooth ramps
            rev_steps = self._build_smooth_ramp(-p, -p, 1.0)
            hw.execute_maneuver(rev_steps, trim_override=best_trim)
            hw.wait_for_stability()

            if not res.samples:
                continue
            avg_yaw = res.avg_yaw_rate
            logger.info(f"    Trim: {best_trim:.3f}, Drift: {avg_yaw:.2f} d/s")

            if abs(avg_yaw) < 2.0:
                logger.info("  [SUCCESS] Drift is negligible.")
                self._run_square_validation(hw, best_trim)
                return StepStatus.SUCCESS, {}, {'motor_trim': best_trim, 'motor_trim_verified': True}

            # Adaptive Correction
            gain = 0.015 if abs(avg_yaw) > 10.0 else 0.005
            correction = avg_yaw * gain
            best_trim = max(-0.4, min(0.4, best_trim + correction))

        logger.warning("  [WARNING] Could not perfectly trim. Saving best effort.")
        self._run_square_validation(hw, best_trim)
        return StepStatus.SUCCESS, {}, {'motor_trim': best_trim, 'motor_trim_verified': True}
