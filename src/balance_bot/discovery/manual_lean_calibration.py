"""
# System Context
This module is part of the `balance_bot` application, designed to control a self-balancing
homebrew robot. It relies on a deterministic, high-frequency control loop and pessimistic hardware interactions.

# Business Rules
- Fail-fast initialization: The system must crash loudly if physical hardware is missing or unresponsive during boot.
- Fault-tolerant control loop: Once Tier 1 is running (e.g., `BalanceCore`), transient I/O errors must not collapse the system; use continuous data quality metrics instead of fatal exceptions.
- Physical pessimism: Never hardcode physical constants; rely on zero-knowledge self-discovery to deduce configuration.

# Dependency Maps
- Relies on internal configuration (`HardwareConfig`, `LearningState`).
- Interfaces with Tier 1 (`BalanceCore`), Tier 3 (`Agent`), and physical hardware abstraction (`RobotHardware`).
"""

import logging
import time
from typing import Any

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..enums import Axis
from ..hardware.robot_hardware import RobotHardware

logger = logging.getLogger(__name__)


class ManualLeanCalibrationStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Manual Lean Calibration (Determine Axes & Balance Range)"

    def is_verified(self, state: LearningState) -> bool:
        return state.manual_lean_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> tuple[
        StepStatus, dict[str, Any], dict[str, Any]]:
        logger.info("========================================")
        logger.info(">>> MANUAL LEAN CALIBRATION REQUIRED <<<")
        logger.info("Please follow the physical instructions.")
        logger.info("========================================")

        # 1. Lean Backwards (Wait for Rest)
        logger.info("1. Gently lean the robot BACKWARDS until it rests on its bumpers.")
        logger.info("   Waiting for stable backward resting position...")

        def get_stable_readings(num_samples: int = 10, delay: float = 0.1) -> dict[Axis, float]:
            sum_x = sum_y = sum_z = 0.0
            for _ in range(num_samples):
                if hw.watchdog:
                    hw.watchdog.heartbeat()
                accel, gyro = hw.read_imu_raw()
                sum_x += accel.x
                sum_y += accel.y
                sum_z += accel.z
                time.sleep(delay)
            return {
                Axis.X: sum_x / num_samples,
                Axis.Y: sum_y / num_samples,
                Axis.Z: sum_z / num_samples
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
        fwd_dir = 1.0 if backward_readings[fwd_axis] > forward_readings[fwd_axis] else -1.0

        # We calculate the true physical pitch angle based on the two dominant axes.
        # This inherently supports any 90-degree rotated IMU mounting,
        # and guarantees the sign matches the runtime pitch calculations in RobotHardware.
        from ..utils import calculate_pitch

        # Apply the identified axis directions (inversions) to get standard Y(forward)/Z(vertical) values
        back_fwd_val = backward_readings[fwd_axis] * fwd_dir
        back_vert_val = backward_readings[vert_axis] * vert_dir

        front_fwd_val = forward_readings[fwd_axis] * fwd_dir
        front_vert_val = forward_readings[vert_axis] * vert_dir

        max_tilt_back = calculate_pitch(back_fwd_val, back_vert_val)
        max_tilt_front = calculate_pitch(front_fwd_val, front_vert_val)

        if max_tilt_front > max_tilt_back:
            logger.error(
                f"  [FAILURE] Kinematic sanity check failed. The forward resting pitch ({max_tilt_front:.1f}°) should be less than the backward resting pitch ({max_tilt_back:.1f}°)."
            )
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
