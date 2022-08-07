#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pybullet_envs.bullet import CartPoleBulletEnv
from stable_baselines3.dqn import DQN
from time import sleep
import pybullet as p

# 定义回调函数，打印当前的rewards
def callback(*params):
    rewards = sum(params[0]['rewards'])
    print("rewards: ", rewards)

env = CartPoleBulletEnv(renders=False, discrete_actions=True)

model = DQN(policy="MlpPolicy", env=env)

print("开始训练，稍等片刻")
model.learn(total_timesteps=10000, callback=callback)
model.save("./model")