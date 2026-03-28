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

from .step import CalibrationStep, StepStatus
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..utils import scan_i2c, make_i2c_check_fn

logger = logging.getLogger(__name__)


class DiscoverBusesStep(CalibrationStep):
    @property
    def name(self) -> str:
        return "Discover I2C Buses"

    def is_verified(self, state: LearningState) -> bool:
        return state.i2c_buses_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> tuple[
        StepStatus, dict[str, Any], dict[str, Any]]:
        logger.info("Scanning I2C Buses...")

        # 1. Find Motors (0x22)
        check_motor = make_i2c_check_fn(0x22, register=0)
        found_motor_bus = scan_i2c("PiconZero (Motors)", check_motor)
        if found_motor_bus is None:
            return StepStatus.FATAL, {}, {}

        # 2. Find IMU (0x68)
        check_imu = make_i2c_check_fn(0x68, register=0x75, expected_value=0x68)
        found_imu_bus = scan_i2c("MPU6050 (IMU)", check_imu)
        if found_imu_bus is None:
            return StepStatus.FATAL, {}, {}

        return StepStatus.SUCCESS, {
            'motor_i2c_bus': found_motor_bus,
            'imu_i2c_bus': found_imu_bus,
            'motor_l': 0,  # Bootstrap guess
            'motor_r': 1  # Bootstrap guess
        }, {'i2c_buses_verified': True}
