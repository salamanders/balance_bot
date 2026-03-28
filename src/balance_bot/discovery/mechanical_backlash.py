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
from ..hardware.robot_hardware import RobotHardware

logger = logging.getLogger(__name__)


# --- Step 6: Mechanical Backlash ---
class MechanicalBacklashStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Mechanical Backlash"

    def is_verified(self, state: LearningState) -> bool:
        return state.backlash_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> tuple[
        StepStatus, dict[str, Any], dict[str, Any]]:
        logger.info(">>> Measuring Mechanical Backlash <<<")
        hw.wait_for_stability(duration=1.0)

        test_power = state.min_power_visible + 10

        # Forward Ramp
        steps_fwd = []
        ramp_steps = 5
        for i in range(1, ramp_steps + 1):
            factor = i / ramp_steps
            steps_fwd.append((test_power * factor, test_power * factor, 0.05))
        steps_fwd.append((test_power, test_power, 0.3))

        # Ramp down
        for i in range(ramp_steps - 1, -1, -1):
            factor = i / ramp_steps
            steps_fwd.append((test_power * factor, test_power * factor, 0.05))

        hw.execute_maneuver(steps_fwd)
        hw.stop()
        hw.wait_for_stability(duration=1.0)

        # Start timing reverse maneuver
        start_time = time.time()

        # Execute the ramp while checking for movement to capture slop properly
        slop_time = 0.2
        moved = False

        for i in range(1, 4):
            factor = i / 3.0
            hw.set_motors(-test_power * factor, -test_power * factor)

            # Wait 0.05s while checking for movement
            step_start = time.time()
            while time.time() - step_start < 0.05:
                reading = hw.read_imu_converted()
                if abs(reading.pitch_rate) > 5.0:
                    slop_time = time.time() - start_time
                    moved = True
                    break
                time.sleep(0.005)

            if moved:
                break

        # If it hasn't moved yet during ramp, hold full power and keep timing
        if not moved:
            hw.set_motors(-test_power, -test_power)
            while time.time() - start_time < 1.0:
                reading = hw.read_imu_converted()
                if abs(reading.pitch_rate) > 5.0:
                    slop_time = time.time() - start_time
                    break
                time.sleep(0.005)

        # Ramp down from reverse
        steps_rev_down = []
        for i in range(2, -1, -1):  # Quick 3-step ramp down
            factor = i / 3.0
            steps_rev_down.append((-test_power * factor, -test_power * factor, 0.05))
        hw.execute_maneuver(steps_rev_down)

        hw.stop()
        compensated = max(0.0, slop_time - 0.02)
        logger.info(f"  Backlash: {compensated:.3f}s")

        return StepStatus.SUCCESS, {}, {
            'control': state.control.model_copy(update={'backlash_pulse_time': compensated}),
            'backlash_verified': True
        }
