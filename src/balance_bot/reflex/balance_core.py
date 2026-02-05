from dataclasses import dataclass
from typing import NamedTuple

from ..config import RobotConfig, PIDParams, CRASH_ANGLE
from ..hardware.robot_hardware import RobotHardware
from ..utils import ComplementaryFilter
from .pid import PIDController


@dataclass(frozen=True)
class MotionRequest:
    """
    Tier 3 -> Tier 1 Command Interface.
    """
    velocity: float = 0.0  # -1.0 to 1.0 (Forward/Backward)
    turn_rate: float = 0.0  # -1.0 to 1.0 (Left/Right)


@dataclass(frozen=True)
class BalanceTelemetry:
    """
    Tier 1 -> Tier 2/3 Data Interface.
    """
    pitch_angle: float
    pitch_rate: float
    yaw_rate: float
    motor_output: float
    crashed: bool


class TuningParams(NamedTuple):
    """
    Tier 2 -> Tier 1 Adaptation Interface.
    Allows dynamic adjustment of PID and Balance Point.
    """
    kp: float
    ki: float
    kd: float
    target_angle_offset: float


class BalanceCore:
    """
    Tier 1: The Brainstem.
    High-frequency reflex loop responsible for keeping the robot upright.

    Principles:
     - Deterministic execution (Minimize allocations/logic).
     - Statelessness (Logic depends only on inputs and physics).
     - Safety (Hard-coded limits).
    """

    # Maximum tilt angle commanded by velocity input (degrees)
    MAX_TILT_ANGLE = 10.0

    def __init__(self, config: RobotConfig):
        self.config = config

        # Hardware
        self.hw = RobotHardware(
            motor_l=config.motor_l,
            motor_r=config.motor_r,
            invert_l=config.motor_l_invert,
            invert_r=config.motor_r_invert,
            gyro_axis=config.gyro_pitch_axis,
            gyro_invert=config.gyro_pitch_invert,
            accel_vertical_axis=config.accel_vertical_axis,
            accel_vertical_invert=config.accel_vertical_invert,
            accel_forward_axis=config.accel_forward_axis,
            accel_forward_invert=config.accel_forward_invert,
            i2c_bus=config.i2c_bus,
            crash_angle=config.crash_angle,
        )
        self.hw.init()

        # Control
        self.pid = PIDController(config.pid)
        self.filter = ComplementaryFilter(config.complementary_alpha)

        # State
        self.pitch = 0.0

    def update(
        self,
        motion: MotionRequest,
        tuning: TuningParams,
        loop_delta_time: float,
        battery_compensation: float = 1.0,
    ) -> BalanceTelemetry:
        """
        Execute one reflex step.

        :param motion: Desired movement (Velocity, Turn).
        :param tuning: Current PID gains and balance offset.
        :param loop_delta_time: Time elapsed since last step.
        :param battery_compensation: Multiplier to account for voltage drop (1.0 = Full, <1.0 = Low).
        :return: Telemetry for higher tiers.
        """
        # 1. Read Physics
        reading = self.hw.read_imu_converted()

        # 2. Update State Estimation
        self.pitch = self.filter.update(
            reading.pitch_angle, reading.pitch_rate, loop_delta_time
        )

        # 3. Apply Tuning (Tier 2 Adaptation)
        # We update the PID controller's params dynamically
        # Ideally, we wouldn't mutate this every frame if it's slow,
        # but Python property assignment is fast enough.
        self.pid.params.kp = tuning.kp
        self.pid.params.ki = tuning.ki
        self.pid.params.kd = tuning.kd

        # 4. Calculate Targets
        # Map Velocity (-1 to 1) to Target Angle (-MAX to MAX)
        # Note: To move Forward (Positive Velocity), we must lean Forward (Positive Angle).
        # (Assumes Positive Pitch = Leaning Forward)
        velocity_tilt = motion.velocity * self.MAX_TILT_ANGLE

        target_angle = (
            self.config.pid.target_angle  # Base mechanical setpoint
            + tuning.target_angle_offset  # Adaptation offset
            + velocity_tilt               # Intentional tilt
        )

        # 5. Safety Cutoff
        if abs(self.pitch) > self.config.crash_angle:
            self.hw.stop()
            self.pid.reset()  # Reset integral windup on crash
            return BalanceTelemetry(
                pitch_angle=self.pitch,
                pitch_rate=reading.pitch_rate,
                yaw_rate=reading.yaw_rate,
                motor_output=0.0,
                crashed=True
            )

        # 6. Calculate Control Output
        error = self.pitch - target_angle

        pid_output = self.pid.update(
            error, loop_delta_time, measurement_rate=reading.pitch_rate
        )

        # 7. Apply Turning
        # Turn Correction: Add offset to motors to rotate.
        # We also use Yaw Rate damping to make turns smoother?
        # For now, simple differential drive.
        # We assume positive turn_rate = Right Turn.
        # To turn Right, Left Motor > Right Motor.

        # Implementation from original main.py:
        # turn_correction = -reading.yaw_rate * yaw_correction_factor
        # That was for stabilization (resist turning).
        # Here we want to CAUSE turning.

        # Let's combine Intentional Turn + Stabilization.
        # Intentional: motion.turn_rate * Gain
        # Stabilization: -reading.yaw_rate * CorrectionFactor

        turn_cmd = motion.turn_rate * 30.0  # Arbitrary gain for now
        yaw_damping = -reading.yaw_rate * self.config.control.yaw_correction_factor

        total_turn = turn_cmd + yaw_damping

        left_motor = pid_output + total_turn
        right_motor = pid_output - total_turn

        # 8. Actuate
        # Apply Battery Compensation
        if battery_compensation > 0:
            left_motor /= battery_compensation
            right_motor /= battery_compensation

        self.hw.set_motors(left_motor, right_motor)

        return BalanceTelemetry(
            pitch_angle=self.pitch,
            pitch_rate=reading.pitch_rate,
            yaw_rate=reading.yaw_rate,
            motor_output=pid_output, # Raw PID output (useful for battery estimation)
            crashed=False
        )

    def cleanup(self):
        self.hw.stop()
        self.hw.cleanup()
