import time
import logging
from typing import Tuple, Dict, Any

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

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
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
        reading = None
        _ = reading
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
            # Note: updating nested pydantic model requires full replacement or smart merge.
            # LearningState.control is ControlConfig.
            # We should provide the updated ControlConfig object.
            # But `state.control` is mutable? No, ControlConfig is a Pydantic model.
            # If `LearningState` defines `control: ControlConfig`, we need to update the field.
            # The dictionary approach requires the pipeline to handle nested updates?
            # Usually `model_copy(update=dict)` is shallow.
            # So I should pass `control` key with the NEW ControlConfig object.
            'backlash_verified': True
        }
