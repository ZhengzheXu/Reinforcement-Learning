#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pybullet as p
from time import sleep
import os
from pybullet_envs.bullet import CartPoleBulletEnv
from parl.algorithms import DQN
import numpy as np
from parl.utils import logger, ReplayMemory
# 内部依赖
from model import Model
from Agent import Agent
from Train import *

# 创建环境
env = CartPoleBulletEnv(renders=True, discrete_actions=True)
act_n = env.action_space.n
obs_n = env.observation_space.shape[0]
# 创建经验池
rpm = ReplayMemory(max_size=MEMORY_SIZE, act_dim=act_n, obs_dim=obs_n)
# 创建网络
model = Model(act_dim=act_n)
# 创建算法对象
alg = DQN(model=model, act_dim=act_n, gamma=GAMMA, lr=LEARNING_RATE)
# 创建agent
agent = Agent(
    algorithm=alg,
    obs_dim=obs_n,
    act_dim=act_n,
    e_greed=0.1,            # 随机采样的概率
    e_greed_decrement=1e-6  # 随机采样概率的衰减率
)
# 保存路径
save_path = "./3dCartPole_model.ckpt"
# 路径存在，则持久化训练
if os.path.exists(save_path):
    agent.restore(save_path)
    
# 先往经验池里存一些数据，避免最开始训练的时候样本丰富度不够
while rpm.size() < MEMORY_WARMUP_SIZE:
    run_episode(env, agent, rpm)

# 开始训练
for episode in range(12):  # 训练max_episode个回合，test部分不计算入episode数量
    # train part
    for _ in range(0, 50):
        total_reward = run_episode(env, agent, rpm)

    # test part
    eval_reward = evaluate(env, agent, render=False)  # render=True 查看显示效果
    logger.info('episode:{:4}    e_greed:{:.5f}   test_reward:{}'.format(
        episode, agent.e_greed, eval_reward))

# 训练结束，保存模型
agent.save(save_path)