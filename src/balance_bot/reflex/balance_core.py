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

import math
from dataclasses import dataclass
from typing import Any

from pydantic import validate_call

from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from ..utils import ComplementaryFilter
from ..watchdog import SurvivalWatchdog
from .pid import PIDController


@dataclass(slots=True)
class MotionRequest:
    """
    Tier 3 -> Tier 1 Command Interface.
    """

    velocity: float = 0.0
    turn_rate: float = 0.0
    enable_control: bool = True


@dataclass(slots=True)
class BalanceTelemetry:
    """
    Tier 1 -> Tier 2/3 Data Interface.

    NOTE: This is intentionally implemented as a standard class with __slots__
    rather than a @dataclass(frozen=True) to avoid significant instantiation
    overhead inside the high-frequency 100Hz reflex loop.
    """

    pitch_angle: float
    pitch_rate: float
    yaw_rate: float
    error_count: int
    motor_output: float
    crashed: bool
    left_pwm: float
    right_pwm: float
    target_angle: float


@dataclass(slots=True)
class TuningParams:
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

    def __init__(
        self,
        hw_config: HardwareConfig,
        learning_state: LearningState,
        watchdog: SurvivalWatchdog | None = None,
        deadman_server: Any = None,
    ):
        self.hw_config = hw_config
        self.learning_state = learning_state

        self.hw = RobotHardware(
            self.hw_config, self.learning_state, watchdog=watchdog, deadman_server=deadman_server
        )
        self.hw.init()

        # Control
        self.pid = PIDController(learning_state.pid)
        self.filter = ComplementaryFilter(
            alpha=hw_config.complementary_alpha,
            gyro_only_alpha=hw_config.gyro_only_alpha,
            rate_trust_limit=hw_config.gyro_rate_distrust_limit,
        )

        # State
        self.pitch = 0.0

        # Pre-allocate telemetry object
        self.current_telemetry = BalanceTelemetry(
            pitch_angle=0.0,
            pitch_rate=0.0,
            yaw_rate=0.0,
            error_count=0,
            motor_output=0.0,
            crashed=False,
            left_pwm=0.0,
            right_pwm=0.0,
            target_angle=0.0,
        )

    def set_i2c_retries(self, retries: int) -> None:
        """Set the I2C retry count for the motor driver."""
        self.hw.set_motor_retries(retries)

    def seed_pitch_filter(self, samples: int = 100) -> None:
        """Sample the IMU at rest to seed the complementary filter with the true starting angle."""
        if samples <= 0:
            return
        readings = [self.hw.read_imu_converted().pitch_angle for _ in range(samples)]
        mean_pitch = sum(readings) / len(readings)
        self.filter.angle = mean_pitch
        self.pitch = mean_pitch
        self.current_telemetry.pitch_angle = mean_pitch

    @validate_call
    def pulse(
        self, left_pwm: float, right_pwm: float, loop_delta_time: float
    ) -> BalanceTelemetry:
        assert loop_delta_time > 0, "Loop delta time must be positive"
        """
        Execute one open-loop actuation step with state estimation.
        Bypasses PID and idle-stop semantics for deterministic pulse control.
        """
        # 1. Read Physics
        reading = self.hw.read_imu_converted()

        # 2. Update State Estimation
        self.pitch = self.filter.update(reading.pitch_angle, reading.pitch_rate, loop_delta_time)

        # 3. Clamp and apply motor drive
        left_cmd = max(-100.0, min(100.0, float(left_pwm)))
        right_cmd = max(-100.0, min(100.0, float(right_pwm)))
        self.hw.set_motors(left_cmd, right_cmd)

        # 4. Populate telemetry
        self.current_telemetry.pitch_angle = self.pitch
        self.current_telemetry.pitch_rate = reading.pitch_rate
        self.current_telemetry.yaw_rate = reading.yaw_rate
        self.current_telemetry.error_count = reading.error_count
        self.current_telemetry.motor_output = 0.0
        self.current_telemetry.crashed = False
        self.current_telemetry.left_pwm = left_cmd
        self.current_telemetry.right_pwm = right_cmd
        self.current_telemetry.target_angle = self.learning_state.pid.target_angle
        return self.current_telemetry

    @validate_call
    def update(
        self,
        motion: MotionRequest,
        tuning: TuningParams,
        loop_delta_time: float,
        battery_compensation: float = 1.0,
    ) -> BalanceTelemetry:
        assert loop_delta_time > 0, "Loop delta time must be positive"
        assert battery_compensation > 0, "Battery compensation must be positive"
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
        self.pitch = self.filter.update(reading.pitch_angle, reading.pitch_rate, loop_delta_time)

        # 3. Check for Control Disable (Idle / Resting)
        if not motion.enable_control:
            # Explicitly Idle: Reset PID integrators and Stop Motors.
            self.pid.reset()
            self.hw.stop()
            self.current_telemetry.pitch_angle = self.pitch
            self.current_telemetry.pitch_rate = reading.pitch_rate
            self.current_telemetry.yaw_rate = reading.yaw_rate
            self.current_telemetry.error_count = reading.error_count
            self.current_telemetry.motor_output = 0.0
            self.current_telemetry.crashed = False
            self.current_telemetry.left_pwm = 0.0
            self.current_telemetry.right_pwm = 0.0
            self.current_telemetry.target_angle = self.learning_state.pid.target_angle
            return self.current_telemetry

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
        velocity_tilt = motion.velocity * self.learning_state.control.max_tilt_angle

        target_angle = (
            self.learning_state.pid.target_angle  # Base mechanical setpoint
            + tuning.target_angle_offset  # Adaptation offset
            + velocity_tilt  # Intentional tilt
        )

        # 6. Safety Cutoff
        if abs(self.pitch) > self.learning_state.crash_angle:
            self.hw.stop()
            self.pid.reset()  # Reset integral windup on crash
            self.current_telemetry.pitch_angle = self.pitch
            self.current_telemetry.pitch_rate = reading.pitch_rate
            self.current_telemetry.yaw_rate = reading.yaw_rate
            self.current_telemetry.error_count = reading.error_count
            self.current_telemetry.motor_output = 0.0
            self.current_telemetry.crashed = True
            self.current_telemetry.left_pwm = 0.0
            self.current_telemetry.right_pwm = 0.0
            self.current_telemetry.target_angle = target_angle
            return self.current_telemetry

        # 7. Calculate Control Output
        error = -circular_difference(target_angle, self.pitch)

        pid_output = self.pid.update(error, loop_delta_time, measurement_rate=reading.pitch_rate)

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

        turn_cmd = motion.turn_rate * self.learning_state.control.turn_gain
        yaw_damping = -reading.yaw_rate * self.learning_state.control.yaw_correction_factor

        total_turn = turn_cmd + yaw_damping

        left_motor = pid_output + total_turn
        right_motor = pid_output - total_turn

        # 9. Actuate
        # Apply Battery Compensation
        if battery_compensation > 0:
            left_motor /= battery_compensation
            right_motor /= battery_compensation

        # Apply Stiction / Deadband Compensation
        min_pwr = float(self.learning_state.min_power_visible)
        if min_pwr > 0.0:
            if abs(left_motor) > 0.01:
                left_motor = math.copysign(
                    min_pwr + abs(left_motor) * (1.0 - min_pwr / 100.0), left_motor
                )
            if abs(right_motor) > 0.01:
                right_motor = math.copysign(
                    min_pwr + abs(right_motor) * (1.0 - min_pwr / 100.0), right_motor
                )

        # clamp: keep motor command within [-100, 100] hardware limit
        left_motor = max(-100.0, min(100.0, left_motor))
        right_motor = max(-100.0, min(100.0, right_motor))
        self.hw.set_motors(left_motor, right_motor)

        self.current_telemetry.pitch_angle = self.pitch
        self.current_telemetry.pitch_rate = reading.pitch_rate
        self.current_telemetry.yaw_rate = reading.yaw_rate
        self.current_telemetry.error_count = reading.error_count
        self.current_telemetry.motor_output = pid_output
        self.current_telemetry.crashed = False
        self.current_telemetry.left_pwm = left_motor
        self.current_telemetry.right_pwm = right_motor
        self.current_telemetry.target_angle = target_angle
        return self.current_telemetry

    def cleanup(self) -> None:
        self.hw.stop()
        self.hw.cleanup()
