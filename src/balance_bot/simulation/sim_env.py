import math
import os
import random
import numpy as np
import pybullet as pb
import pybullet_data
import gymnasium as gym
from gymnasium import spaces

class BalanceBotEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 100}

    def __init__(self, render_mode=None):
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

    def _get_obs(self):
        # In URDF, position and orientation
        pos, orn = pb.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client_id)

        # Convert quaternion to euler (roll, pitch, yaw) in radians
        # Note: PyBullet returns rpy, but depending on URDF coordinate system, pitch might be Y or X.
        # Assuming robot rolls forward on X axis, wheels rotate around Y, so pitch is rotation around Y.
        euler = pb.getEulerFromQuaternion(orn)
        pitch = euler[1]
        yaw = euler[2]

        # Base velocity
        linear_vel, angular_vel = pb.getBaseVelocity(self.robot_id, physicsClientId=self.client_id)

        # angular_vel is (wx, wy, wz)
        # pitch rate is around Y, yaw rate around Z
        pitch_rate = angular_vel[1]
        yaw_rate = angular_vel[2]

        # Add sensor noise and mounting offset
        pitch += self.imu_pitch_offset

        # Add gaussian noise to pitch and pitch rate (mu=0, sigma=0.02)
        noise_pitch = random.gauss(0, 0.02)
        noise_pitch_rate = random.gauss(0, 0.02)

        obs_pitch = pitch + noise_pitch
        obs_pitch_rate = pitch_rate + noise_pitch_rate
        obs_yaw = yaw
        obs_yaw_rate = yaw_rate

        return np.array([obs_pitch, obs_pitch_rate, obs_yaw, obs_yaw_rate], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        pb.resetSimulation(physicsClientId=self.client_id)
        pb.setGravity(0, 0, -9.81, physicsClientId=self.client_id)
        pb.setTimeStep(self.physics_dt, physicsClientId=self.client_id)

        # Load plane
        self.plane_id = pb.loadURDF("plane.urdf", physicsClientId=self.client_id)

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
        self.left_joint = 1
        self.right_joint = 2

        # Disable default velocity control for wheels so we can use torque control
        pb.setJointMotorControl2(self.robot_id, self.left_joint, pb.VELOCITY_CONTROL, force=0, physicsClientId=self.client_id)
        pb.setJointMotorControl2(self.robot_id, self.right_joint, pb.VELOCITY_CONTROL, force=0, physicsClientId=self.client_id)

        # === Domain Randomization ===
        self.left_torque_mod = random.uniform(0.85, 1.15)
        self.right_torque_mod = random.uniform(0.85, 1.15)
        self.imu_pitch_offset = math.radians(random.uniform(-3.0, 3.0))

        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action):
        left_pwm, right_pwm = action

        # Apply domain randomized torque modifiers
        # Note: In PyBullet, torque control takes absolute force. PWM (-1 to 1)
        # is scaled by a max_torque multiplier and the random domain modifier.
        left_torque = left_pwm * self.max_torque * self.left_torque_mod
        right_torque = right_pwm * self.max_torque * self.right_torque_mod

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
        info = {}

        return obs, reward, terminated, truncated, info

    def close(self):
        if self.client_id >= 0:
            pb.disconnect(self.client_id)
            self.client_id = -1
