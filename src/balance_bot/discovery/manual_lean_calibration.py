import time
import logging
from typing import Any

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..enums import Axis

logger = logging.getLogger(__name__)

class ManualLeanCalibrationStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Manual Lean Calibration (Determine Axes & Balance Range)"

    def is_verified(self, state: LearningState) -> bool:
        return state.manual_lean_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> tuple[StepStatus, dict[str, Any], dict[str, Any]]:
        logger.info("========================================")
        logger.info(">>> MANUAL LEAN CALIBRATION REQUIRED <<<")
        logger.info("Please follow the physical instructions.")
        logger.info("========================================")

        # 1. Lean Backwards (Wait for Rest)
        logger.info("1. Gently lean the robot BACKWARDS until it rests on its bumpers.")
        logger.info("   Waiting for stable backward resting position...")

        def get_stable_readings(num_samples: int = 10, delay: float = 0.1) -> dict[Axis, float]:
            samples: dict[Axis, list[float]] = {Axis.X: [], Axis.Y: [], Axis.Z: []}
            for _ in range(num_samples):
                if hw.watchdog:
                    hw.watchdog.heartbeat()
                accel, gyro = hw.read_imu_raw()
                samples[Axis.X].append(accel.x)
                samples[Axis.Y].append(accel.y)
                samples[Axis.Z].append(accel.z)
                time.sleep(delay)
            return {
                Axis.X: sum(samples[Axis.X]) / num_samples,
                Axis.Y: sum(samples[Axis.Y]) / num_samples,
                Axis.Z: sum(samples[Axis.Z]) / num_samples
            }

        def wait_for_flop(initial_readings: dict[Axis, float], threshold: float = 0.5) -> dict[Axis, float]:
            while True:
                if hw.watchdog:
                    hw.watchdog.heartbeat()
                current_accel, current_gyro = hw.read_imu_raw()
                diff_x = abs(current_accel.x - initial_readings[Axis.X])
                diff_y = abs(current_accel.y - initial_readings[Axis.Y])
                diff_z = abs(current_accel.z - initial_readings[Axis.Z])

                if diff_x > threshold or diff_y > threshold or diff_z > threshold:
                    logger.info("   Movement detected. Waiting for it to settle...")
                    time.sleep(1.0)
                    return get_stable_readings()
                time.sleep(0.1)

        backward_readings = get_stable_readings()
        logger.info("   Backward resting position locked.")

        # 2. Flop Forward
        logger.info("2. Now, manually push the robot over so it flops FORWARD onto its other bumpers.")
        logger.info("   Waiting for forward resting position...")
        forward_readings = wait_for_flop(backward_readings)
        logger.info("   Forward resting position locked.")

        # Analyze Dominance
        from ..utils import analyze_dominance

        vert_axis_str, _, vert_success = analyze_dominance(backward_readings, "Backward Rest Vertical Dominance")
        if not vert_success:
            logger.error("  [FAILURE] Could not identify a clear vertical axis.")
            return StepStatus.NEEDS_RETRY, {}, {}

        vert_axis = Axis(vert_axis_str)
        vert_dir = 1.0 if backward_readings[vert_axis] > 0 else -1.0

        # Calculate delta to find the forward axis (the axis that changed the most)
        delta_readings = {
            axis: forward_readings[axis] - backward_readings[axis]
            for axis in backward_readings
        }

        fwd_axis_str, _, fwd_success = analyze_dominance(delta_readings, "Forward/Backward Swing Dominance")
        if not fwd_success:
            logger.error("  [FAILURE] Could not identify a clear forward axis.")
            return StepStatus.NEEDS_RETRY, {}, {}

        fwd_axis = Axis(fwd_axis_str)
        fwd_dir = 1.0 if forward_readings[fwd_axis] > backward_readings[fwd_axis] else -1.0

        # The dominant vertical axis determines the range
        back_accel = backward_readings[fwd_axis] * fwd_dir
        front_accel = forward_readings[fwd_axis] * fwd_dir

        if back_accel > front_accel:
            logger.error("  [FAILURE] Kinematic sanity check failed. The forward leaning acceleration should be greater than the backward leaning acceleration along the forward axis.")
            return StepStatus.NEEDS_RETRY, {}, {}

        # Rough conversion to degrees (assuming 1g = 9.81m/s^2, sin(theta) ~ accel/9.81)
        # We don't need exact degrees here, just a logical physical range.
        import math
        try:
            # Clip to [-1, 1] to avoid math domain errors
            max_tilt_back = math.degrees(math.asin(max(-1.0, min(1.0, back_accel / 9.81))))
            max_tilt_front = math.degrees(math.asin(max(-1.0, min(1.0, front_accel / 9.81))))
        except ValueError:
             return StepStatus.NEEDS_RETRY, {}, {}

        logger.info(f"  Identified Vertical Axis: {vert_axis.name} (Dir: {vert_dir})")
        logger.info(f"  Identified Forward Axis:  {fwd_axis.name} (Dir: {fwd_dir})")
        logger.info(f"  Estimated Max Tilt: Back {max_tilt_back:.1f}°, Front {max_tilt_front:.1f}°")

        return StepStatus.SUCCESS, {
            'accel_vertical_axis': vert_axis,
            'accel_vertical_invert': vert_dir < 0,
            'accel_forward_axis': fwd_axis,
            'accel_forward_invert': fwd_dir < 0,
        }, {
            'rest_angle_backward': max_tilt_back,
            'rest_angle_forward': max_tilt_front,
            'manual_lean_verified': True
        }
