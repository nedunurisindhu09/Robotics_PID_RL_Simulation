# RL_Agent.py
"""
Reinforcement Learning agent that learns to tune PID gains on-line.

Requirements:
    pip install gym stable-baselines3 numpy matplotlib

Usage (train):
    python RL_Agent.py

This file provides:
- PIDTuningEnv: a small custom Gym environment that simulates a toy plant
  and applies a PID controller whose gains the agent adjusts.
- Training example using PPO (Stable-Baselines3).
- Save/Load example of the trained model.
"""

import gym
import numpy as np
from gym import spaces
import os
from PID_Controller import PIDController

class PIDTuningEnv(gym.Env):
    """Custom Gym environment where the RL agent adjusts PID gains to track a moving target."""
    metadata = {"render.modes": ["human"]}

    def __init__(self, episode_length=200, dt=0.02, seed=None):
        super().__init__()
        self.episode_length = episode_length
        self.dt = dt
        self.current_step = 0
        self.seed(seed)

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        obs_high = np.array([np.finfo(np.float32).max]*4, dtype=np.float32)
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)

        self.base_Kp, self.base_Ki, self.base_Kd = 2.0, 0.5, 0.05
        self.pid = PIDController(Kp=self.base_Kp, Ki=self.base_Ki, Kd=self.base_Kd)
        self.measurement, self.prev_error, self.setpoint = 0.0, 0.0, 0.0

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def _target_trajectory(self, t):
        return 0.5 * np.sin(0.5 * t)

    def step(self, action):
        self.current_step += 
