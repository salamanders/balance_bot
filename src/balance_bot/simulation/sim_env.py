import math
import os
import random
import numpy as np

try:
    import pybullet as pb # type: ignore
    import pybullet_data # type: ignore
    import gymnasium as gym # type: ignore
    from gymnasium import spaces # type: ignore
    _HAS_SIM = True
    _BaseEnv = gym.Env # type: ignore
except ImportError:
    pb = None  # type: ignore
    pybullet_data = None  # type: ignore
    gym = None  # type: ignore
    spaces = None  # type: ignore
    _HAS_SIM = False
    _BaseEnv = object

class BalanceBotEnv(_BaseEnv): # type: ignore[misc,valid-type]
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 100}

    def __init__(self, render_mode=None):
        self.right_joint = 2
        self.left_joint = 1
        if not _HAS_SIM:
            raise ImportError("pybullet and gymnasium are required to run the simulation environment. Install them with `uv sync --extra sim`.")

        self.render_mode = render_mode

        # 4 observations: pitch, pitch_rate, yaw, yaw_rate (all in radians)
        high = np.inf * np.ones(4, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        # 2 actions: left_motor_pwm, right_motor_pwm (-1.0 to 1.0)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        # Connect to PyBullet
        if self.render_mode == "human":
            self.client_id = pb.connect(pb.GUI)
        else:
            self.client_id = pb.connect(pb.DIRECT)

        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.plane_id = pb.loadURDF("plane.urdf", physicsClientId=self.client_id)

        # Simulation parameters
        self.physics_dt = 1.0 / 200.0  # 200 Hz internal physics
        self.control_dt = 1.0 / 100.0  # 100 Hz control loop
        self.sim_steps_per_control = int(self.control_dt / self.physics_dt)
        pb.setTimeStep(self.physics_dt, physicsClientId=self.client_id)

        # Robot parameters
        self.wheel_radius = 0.03  # 30mm
        self.max_torque = 1.0     # Maximum theoretical torque (to be scaled)

        # Reward/Termination constants (in radians)
        self.pitch_reward_limit = math.radians(15.0)
        self.pitch_terminate_limit = math.radians(45.0)

        self.robot_id = None

        # Domain Randomization placeholders
        self.left_torque_mod = 1.0
        self.right_torque_mod = 1.0
        self.imu_pitch_offset = 0.0
        self.imu_rotation_matrix = np.eye(3)

    def _get_obs(self):
        # In URDF, position and orientation
        pos, orn = pb.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client_id)

        # Multiply robot's orientation quaternion with IMU rotation offset
        # to find the "observed" orientation of the IMU in world space
        # (Actually, better to calculate angular velocity in local frame then apply IMU rotation)

        # Base velocity in WORLD frame
        linear_vel_world, angular_vel_world = pb.getBaseVelocity(self.robot_id, physicsClientId=self.client_id)

        # Convert world angular velocity to robot BODY frame
        # Get inverse of robot orientation (conjugate of quaternion)
        inv_orn = [-orn[0], -orn[1], -orn[2], orn[3]]

        # pb doesn't have a direct vector rotation function using quats, so we convert vector to point, apply transform
        ang_vel_body, _ = pb.multiplyTransforms(
            [0, 0, 0], inv_orn,
            angular_vel_world, [0, 0, 0, 1]
        )

        # Apply random IMU mounting rotation matrix to get angular velocity in IMU frame
        ang_vel_imu = np.dot(self.imu_rotation_matrix, np.array(ang_vel_body))

        # To get the "pitch" and "yaw" observed by the rotated IMU, we need to consider how the IMU calculates orientation
        # Since the real robot just uses the raw angular velocity from the IMU, we just pass the transformed rates.
        # But we also need the absolute pitch/yaw, which is derived from gravity vector on real hardware.
        # Let's find gravity in IMU frame. World gravity is [0, 0, -1].
        gravity_body, _ = pb.multiplyTransforms(
            [0, 0, 0], inv_orn,
            [0, 0, -1], [0, 0, 0, 1]
        )
        gravity_imu = np.dot(self.imu_rotation_matrix, np.array(gravity_body))

        # Calculate pitch and roll from gravity vector
        # (Standard IMU equations: pitch = atan2(-gx, sqrt(gy^2 + gz^2)), roll = atan2(gy, gz))
        obs_pitch = math.atan2(gravity_imu[0], math.sqrt(gravity_imu[1]**2 + gravity_imu[2]**2))

        # The robot uses gyro Z for yaw, so we use the transformed Z angular velocity
        obs_yaw_rate = ang_vel_imu[2]
        obs_pitch_rate = ang_vel_imu[1]

        # We also need an absolute yaw for the observation space (even though hardware uses rate)
        # Just use the original yaw to keep the environment happy, it's not strictly used by hardware
        euler = pb.getEulerFromQuaternion(orn)
        obs_yaw = euler[2]

        # Add sensor noise and mounting offset (the new orientation matrix handles offset naturally, but keeping this for legacy)
        obs_pitch += self.imu_pitch_offset

        # Add gaussian noise to pitch and pitch rate (mu=0, sigma=0.02)
        noise_pitch = random.gauss(0, 0.02)
        noise_pitch_rate = random.gauss(0, 0.02)

        obs_pitch = obs_pitch + noise_pitch
        obs_pitch_rate = obs_pitch_rate + noise_pitch_rate
        obs_yaw_rate = obs_yaw_rate

        return np.array([obs_pitch, obs_pitch_rate, obs_yaw, obs_yaw_rate], dtype=np.float32)

    def reset(self, seed=None, options=None):
        _ = options
        super().reset(seed=seed)

        pb.resetSimulation(physicsClientId=self.client_id)
        pb.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        pb.setTimeStep(self.physics_dt, physicsClientId=self.client_id)

        # Load plane

        # Load robot
        urdf_path = os.path.join(os.path.dirname(__file__), "robot.urdf")

        # Spawn robot at Z = wheel_radius + 0.01 to avoid ground clipping
        start_z = self.wheel_radius + 0.01
        start_pos = [0, 0, start_z]
        start_orn = pb.getQuaternionFromEuler([0, 0, 0])

        self.robot_id = pb.loadURDF(urdf_path, start_pos, start_orn, physicsClientId=self.client_id)

        # Joint mapping
        # 0: drag_bar_joint (fixed)
        # 1: left_wheel_joint (continuous)
        # 2: right_wheel_joint (continuous)

        # Disable default velocity control for wheels so we can use torque control
        pb.setJointMotorControl2(self.robot_id, self.left_joint, pb.VELOCITY_CONTROL, force=0, physicsClientId=self.client_id)
        pb.setJointMotorControl2(self.robot_id, self.right_joint, pb.VELOCITY_CONTROL, force=0, physicsClientId=self.client_id)

        # === Domain Randomization ===
        self.left_torque_mod = random.uniform(0.85, 1.15)
        self.right_torque_mod = random.uniform(0.85, 1.15)
        self.imu_pitch_offset = math.radians(random.uniform(-3.0, 3.0))

        # Generate random 3D orientation for IMU (mounting error, flipped upside down, random yaw, etc.)
        rpy = [random.uniform(-math.pi, math.pi) for _ in range(3)]
        quat = pb.getQuaternionFromEuler(rpy)
        rot_tuple = pb.getMatrixFromQuaternion(quat)
        self.imu_rotation_matrix = np.array(rot_tuple).reshape(3, 3)

        # Randomly invert axes to simulate backward wiring
        if random.choice([True, False]):
            self.imu_rotation_matrix[0] *= -1
        if random.choice([True, False]):
            self.imu_rotation_matrix[1] *= -1
        if random.choice([True, False]):
            self.imu_rotation_matrix[2] *= -1

        obs = self._get_obs()
        info: dict[str, float] = {}
        return obs, info

    def step(self, action):
        left_pwm, right_pwm = action

        # Apply domain randomized torque modifiers
        # Note: In PyBullet, torque control takes absolute force. PWM (-1 to 1)
        # is scaled by a max_torque multiplier and the random domain modifier.

        # Add dynamic high-frequency jitter (dodgy motors) - +/- 5% variance per step
        left_jitter = random.uniform(0.95, 1.05)
        right_jitter = random.uniform(0.95, 1.05)

        left_torque = left_pwm * self.max_torque * self.left_torque_mod * left_jitter
        right_torque = right_pwm * self.max_torque * self.right_torque_mod * right_jitter

        pb.setJointMotorControl2(
            self.robot_id,
            self.left_joint,
            controlMode=pb.TORQUE_CONTROL,
            force=left_torque,
            physicsClientId=self.client_id
        )

        pb.setJointMotorControl2(
            self.robot_id,
            self.right_joint,
            controlMode=pb.TORQUE_CONTROL,
            force=right_torque,
            physicsClientId=self.client_id
        )

        # Step physics exactly self.sim_steps_per_control times (2 times for 100Hz control at 200Hz physics)
        for _ in range(self.sim_steps_per_control):
            pb.stepSimulation(physicsClientId=self.client_id)

        obs = self._get_obs()

        pitch = obs[0]

        # Reward: 1.0 if within +/- 15 deg, else 0.0
        if abs(pitch) <= self.pitch_reward_limit:
            reward = 1.0
        else:
            reward = 0.0

        # Terminate if pitch exceeds +/- 45 deg
        terminated = bool(abs(pitch) > self.pitch_terminate_limit)
        truncated = False
        info: dict[str, float] = {}

        return obs, reward, terminated, truncated, info

    def close(self):
        if self.client_id >= 0:
            pb.disconnect(self.client_id)
            self.client_id = -1
