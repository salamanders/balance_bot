from dataclasses import dataclass

from ..config import RobotConfig
from ..hardware.robot_hardware import RobotHardware
from ..utils import ComplementaryFilter
from ..enums import Axis
from .pid import PIDController


@dataclass(slots=True)
class MotionRequest:
    """
    Tier 3 -> Tier 1 Command Interface.
    """
    velocity: float = 0.0  # -1.0 to 1.0 (Forward/Backward)
    turn_rate: float = 0.0  # -1.0 to 1.0 (Left/Right)
    enable_control: bool = True  # If False, disables PID and sets motors to 0 (Idle/Parked)


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


class TuningParams:
    """
    Tier 2 -> Tier 1 Adaptation Interface.
    Allows dynamic adjustment of PID and Balance Point.
    """
    __slots__ = ['kp', 'ki', 'kd', 'target_angle_offset']

    def __init__(self, kp: float, ki: float, kd: float, target_angle_offset: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_angle_offset = target_angle_offset

    def __repr__(self):
        return f"TuningParams(kp={self.kp}, ki={self.ki}, kd={self.kd}, target_angle_offset={self.target_angle_offset})"


class BalanceCore:
    """
    Tier 1: The Brainstem.
    High-frequency reflex loop responsible for keeping the robot upright.

    Principles:
     - Deterministic execution (Minimize allocations/logic).
     - Statelessness (Logic depends only on inputs and physics).
     - Safety (Hard-coded limits).
    """

    def __init__(self, config: RobotConfig):
        self.config = config

        # Hardware Defaults for First Run
        # If config is incomplete, fill with standard defaults so RobotHardware can init.
        if self.config.motor_i2c_bus is None:
            self.config.motor_i2c_bus = 1
        if self.config.imu_i2c_bus is None:
            self.config.imu_i2c_bus = 1

        # Sensor Defaults (Z=Vert, Y=Fwd, X=Pitch is standard)
        if self.config.gyro_pitch_axis is None:
            self.config.gyro_pitch_axis = Axis.X
        if self.config.accel_vertical_axis is None:
             self.config.accel_vertical_axis = Axis.Z
        if self.config.accel_forward_axis is None:
             self.config.accel_forward_axis = Axis.Y

        self.hw = RobotHardware(self.config)
        self.hw.init()

        # Control
        self.pid = PIDController(config.pid)
        self.filter = ComplementaryFilter(config.complementary_alpha)

        # State
        self.pitch = 0.0

    def set_i2c_retries(self, retries: int) -> None:
        """Set the I2C retry count for the motor driver."""
        self.hw.set_motor_retries(retries)

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

        # 3. Check for Control Disable (Idle / Resting)
        if not motion.enable_control:
            # Explicitly Idle: Reset PID integrators and Stop Motors.
            self.pid.reset()
            self.hw.stop()
            return BalanceTelemetry(
                pitch_angle=self.pitch,
                pitch_rate=reading.pitch_rate,
                yaw_rate=reading.yaw_rate,
                motor_output=0.0,
                crashed=False
            )

        # 4. Apply Tuning (Tier 2 Adaptation)
        # We update the PID controller's params dynamically
        # Ideally, we wouldn't mutate this every frame if it's slow,
        # but Python property assignment is fast enough.
        self.pid.params.kp = tuning.kp
        self.pid.params.ki = tuning.ki
        self.pid.params.kd = tuning.kd

        # 5. Calculate Targets
        # Map Velocity (-1 to 1) to Target Angle (-MAX to MAX)
        # Note: To move Forward (Positive Velocity), we must lean Forward (Positive Angle).
        # (Assumes Positive Pitch = Leaning Forward)
        velocity_tilt = motion.velocity * self.config.control.max_tilt_angle

        target_angle = (
            self.config.pid.target_angle  # Base mechanical setpoint
            + tuning.target_angle_offset  # Adaptation offset
            + velocity_tilt               # Intentional tilt
        )

        # 6. Safety Cutoff
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

        # 7. Calculate Control Output
        error = self.pitch - target_angle

        pid_output = self.pid.update(
            error, loop_delta_time, measurement_rate=reading.pitch_rate
        )

        # 8. Apply Turning
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

        turn_cmd = motion.turn_rate * self.config.control.turn_gain
        yaw_damping = -reading.yaw_rate * self.config.control.yaw_correction_factor

        total_turn = turn_cmd + yaw_damping

        left_motor = pid_output + total_turn
        right_motor = pid_output - total_turn

        # 9. Actuate
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
