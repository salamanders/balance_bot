import logging
from typing import Tuple, Dict, Any

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

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
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
            'motor_r': 1   # Bootstrap guess
        }, {'i2c_buses_verified': True}
