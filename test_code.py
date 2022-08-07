#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pybullet_envs.bullet import CartPoleBulletEnv
from stable_baselines3.dqn import DQN
from time import sleep
import pybullet as p

env = CartPoleBulletEnv(renders=True, discrete_actions=True)
model = DQN('MlpPolicy', env, verbose=1)
model.learn(total_timesteps=10000)
model.save("dqn_cartpole")
