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

from .broken_wire_check import BrokenWireCheckStep
from .derive_kinematics import DeriveKinematicsStep
from .discover_buses import DiscoverBusesStep
from .friction_threshold import FrictionThresholdStep
from .hardware_init import HardwareInitStep
from .kickup_dynamics import KickupDynamicsStep
from .manual_lean_calibration import ManualLeanCalibrationStep
from .mechanical_backlash import MechanicalBacklashStep
from .motor_trim import MotorTrimStep
from .pipeline import SelfDiscoveryPipeline
from .step import CalibrationStep, StepStatus

__all__ = [
    "SelfDiscoveryPipeline",
    "CalibrationStep",
    "StepStatus",
    "DiscoverBusesStep",
    "HardwareInitStep",
    "ManualLeanCalibrationStep",
    "BrokenWireCheckStep",
    "FrictionThresholdStep",
    "DeriveKinematicsStep",
    "MotorTrimStep",
    "MechanicalBacklashStep",
    "KickupDynamicsStep",
]
