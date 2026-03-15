import math
import time
import random
import logging
import numpy as np
import glm
from typing import Optional

from .robot_hardware import IMUReading, MeasureResult, DriveCommand
from ..configuration import HardwareConfig, LearningState
from ..watchdog import SurvivalWatchdog
from ..simulation.sim_env import BalanceBotEnv

logger = logging.getLogger(__name__)

MOTOR_MIN_OUTPUT = -100
MOTOR_MAX_OUTPUT = 100

class SimHardware:
    """
    Simulated Hardware Abstraction Layer that wraps the PyBullet Gymnasium environment
    while providing the exact same interface as RobotHardware.
    """

    def __init__(self, hw_config: HardwareConfig, learning_state: LearningState, watchdog: Optional[SurvivalWatchdog] = None, render_mode: Optional[str] = None):
        self.config = hw_config
        self.state = learning_state
        self.watchdog = watchdog

        self.env = BalanceBotEnv(render_mode=render_mode)
        self.obs, _ = self.env.reset()

        # Keep track of last action to step the environment correctly
        self.last_left_pwm = 0.0
        self.last_right_pwm = 0.0

        # Keep track of timing to simulate real-world frequency
        self.last_step_time = time.time()
        self.control_dt = 1.0 / 100.0  # 100Hz

        # Simulate I2C glitches
        self.error_count = 0
        self.last_valid_imu: Optional[IMUReading] = None

    def apply_config(self, new_config: HardwareConfig) -> None:
        self.config = new_config

    def init(self) -> None:
        logger.info("Initializing SimHardware...")
        self.obs, _ = self.env.reset()

    def cleanup(self) -> None:
        logger.info("Cleaning up SimHardware...")
        self.env.close()

    def stop(self) -> None:
        self.set_motors(0, 0)

    def read_imu_raw(self) -> tuple[glm.vec3, glm.vec3]:
        # Not strictly needed since we use read_imu_converted usually
        return glm.vec3(0,0,0), glm.vec3(0,0,0)

    def read_imu_converted(self) -> IMUReading:
        # Simulate occasional I2C read failures (10% chance)
        if random.random() < 0.10 and self.last_valid_imu is not None:
            self.error_count += 1
            # Step physics to keep time flowing, but drop the new data
            now = time.time()
            elapsed = now - self.last_step_time
            if elapsed < self.control_dt:
                time.sleep(self.control_dt - elapsed)
            action = np.array([self.last_left_pwm, self.last_right_pwm], dtype=np.float32)
            self.obs, reward, terminated, truncated, info = self.env.step(action)
            self.last_step_time = time.time()

            # Return old reading with incremented error count
            return IMUReading(
                pitch_angle=self.last_valid_imu.pitch_angle,
                pitch_rate=self.last_valid_imu.pitch_rate,
                yaw_rate=self.last_valid_imu.yaw_rate,
                roll_angle=self.last_valid_imu.roll_angle,
                roll_rate=self.last_valid_imu.roll_rate,
                error_count=self.error_count
            )

        self.error_count = 0

        # Step the environment with the last set motor values to get the latest physics state
        # The physical loop calls read_imu_converted(), computes PID, then calls set_motors().
        # In simulation, we need to enforce the loop time to emulate hardware delay.
        now = time.time()
        elapsed = now - self.last_step_time

        if elapsed < self.control_dt:
            # Sleep to match 100Hz loop rate
            time.sleep(self.control_dt - elapsed)

        action = np.array([self.last_left_pwm, self.last_right_pwm], dtype=np.float32)
        self.obs, reward, terminated, truncated, info = self.env.step(action)
        self.last_step_time = time.time()

        # PyBullet obs is in radians: [pitch, pitch_rate, yaw, yaw_rate]
        # IMUReading expects degrees
        pitch_rad = self.obs[0]
        pitch_rate_rad = self.obs[1]
        self.obs[2]
        yaw_rate_rad = self.obs[3]

        pitch_deg = math.degrees(pitch_rad)
        pitch_rate_deg = math.degrees(pitch_rate_rad)
        yaw_rate_deg = math.degrees(yaw_rate_rad)

        # Add a dummy roll value (we didn't observe roll in our simplified sim space)
        roll_deg = 0.0
        roll_rate_deg = 0.0

        # Also need error_count to simulate transient I2C glitches (we can just pass 0)
        reading = IMUReading(
            pitch_angle=pitch_deg,
            pitch_rate=pitch_rate_deg,
            yaw_rate=yaw_rate_deg,
            roll_angle=roll_deg,
            roll_rate=roll_rate_deg,
            error_count=self.error_count
        )
        self.last_valid_imu = reading
        return reading

    def set_motor_retries(self, retries: int) -> None:
        pass

    def set_motors(self, left: float, right: float, trim_override: float | None = None) -> None:
        # Simulate occasional I2C write failures to motors (10% chance)
        if random.random() < 0.10:
            logger.debug("Simulated motor write failure - ignoring command")
            return

        # Convert [-100, 100] scale to [-1.0, 1.0] expected by the Env action space
        self.last_left_pwm = max(min(left / 100.0, 1.0), -1.0)
        self.last_right_pwm = max(min(right / 100.0, 1.0), -1.0)

    def get_posture_state(self) -> str:
        # Basic proxy for physical posture state based on current pitch
        # (Resting, Balancing, Crashed, etc.)
        pitch_rad = self.obs[0]
        pitch_deg = abs(math.degrees(pitch_rad))

        # Use simple thresholds for simulation posture
        if pitch_deg < 15.0:
            return "Balancing"
        elif pitch_deg < 50.0:
            return "Resting"
        else:
            return "Crashed"

    def wait_for_stability(self, duration: float = 2.0, threshold: float = 2.0) -> None:
        # In simulation, we can just skip or loop
        logger.info("Simulated wait_for_stability")
        start = time.time()
        while time.time() - start < duration:
            self.read_imu_converted()

    def measure_gravity(self, duration: float = 1.0) -> glm.vec3:
        # Return a nominal gravity vector (Z is down in real world, but depends on accel orientation)
        return glm.vec3(0, 0, 9.81)

    # Some basic maneuver methods
    def execute_maneuver(self, steps: list[tuple[float, float, float]], sample_interval: float = 0.01, trim_override: float | None = None) -> MeasureResult:
        logger.info("Executing maneuver in simulation")
        # Step through the commands and record
        math.degrees(self.obs[0])
        yaw_sum = 0.0
        max_rate = 0.0

        for pwr_l, pwr_r, duration in steps:
            self.set_motors(pwr_l, pwr_r, trim_override)
            steps_needed = int(duration / self.control_dt)
            for _ in range(steps_needed):
                imu = self.read_imu_converted()
                yaw_sum += imu.yaw_rate * self.control_dt
                if abs(imu.pitch_rate) > max_rate:
                    max_rate = abs(imu.pitch_rate)

        self.set_motors(0, 0)
        end_pitch = math.degrees(self.obs[0])

        total_duration = sum(s[2] for s in steps)

        return MeasureResult(
            avg_yaw_rate=yaw_sum / total_duration,
            abs_avg_yaw_rate=abs(yaw_sum / total_duration),
            max_rate=max_rate,
            final_pitch=end_pitch
        )

    def drive_and_measure(self, command: DriveCommand) -> MeasureResult:
        if command.wait_for_stability:
            self.wait_for_stability()
        return self.execute_maneuver(
            [(command.left_power, command.right_power, command.duration)],
            command.sample_interval,
            command.trim_override
        )
