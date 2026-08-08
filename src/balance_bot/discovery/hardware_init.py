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
from typing import Any

from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from .step import CalibrationStep, StepStatus

logger = logging.getLogger(__name__)


class HardwareInitStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Initialize Hardware Drivers"

    def is_verified(self, state: LearningState) -> bool:
        return state.hardware_init_verified

    def run(
        self, hw: RobotHardware, config: HardwareConfig, state: LearningState
    ) -> tuple[StepStatus, dict[str, Any], dict[str, Any]]:
        logger.info("Initializing Hardware Drivers...")
        # Force driver initialization now that buses are known
        try:
            hw.initialize_drivers()
        except Exception as e:
            logger.error(f"  [FAILURE] Driver init raised exception: {e}")
            return StepStatus.FATAL, {}, {}

        # Verify they are alive
        if hw.pz is None or hw.sensor is None:
            logger.error("  [FAILURE] Drivers failed to initialize despite known buses.")
            return StepStatus.FATAL, {}, {}

        try:
            hw.init()  # Init the motor driver specifically
        except Exception as e:
            logger.error(f"  [FAILURE] Motor driver init failed: {e}")
            return StepStatus.FATAL, {}, {}

        return StepStatus.SUCCESS, {}, {"hardware_init_verified": True}
