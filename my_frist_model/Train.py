#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from time import sleep
# 定义一些超参
'''
LEARN_FREQ: 训练频率，不需要每一个step都learn，攒一些新增经验后再learn，提高效率
MEMORY_SIZE: eplay memory的大小，越大越占用内存
MEMORY_WARMUP_SIZE: replay_memory 里需要预存一些经验数据，再开启训练
BATCH_SIZE: 每次给agent learn的数据数量，从replay memory随机里sample一批数据出来
LEARNING_RATE: 学习率
GAMMA: reward 的衰减因子，一般取 0.9 到 0.999 不等
'''
LEARN_FREQ = 5
MEMORY_SIZE = 20000
MEMORY_WARMUP_SIZE = 200
BATCH_SIZE = 32
LEARNING_RATE = 0.001
GAMMA = 0.99

# 训练一个episode
def run_episode(env, agent, rpm):
    total_reward = 0
    obs = env.reset()
    step = 0
    while True:
        step += 1
        action = agent.sample(obs)
        next_obs, reward, done, _ = env.step(action)
        # 装入经验池，而不是立刻原地训练
        rpm.append(obs, action, reward, next_obs, done)

        # 满足两个条件即可从经验池中抽取一个batch的数据来训练：
        # 1. 经验池的大小大于预热的数值
        # 2. 每隔5步训练一次，所以需要时5的倍速
        if (rpm.size() > MEMORY_WARMUP_SIZE) and (step % LEARN_FREQ == 0):
            batch_obs, batch_action, batch_reward, batch_next_obs, batch_done = rpm.sample_batch(BATCH_SIZE)
            train_loss = agent.learn(batch_obs, batch_action, batch_reward, batch_next_obs, batch_done)

        total_reward += reward
        obs = next_obs
        if done:
            break
    return  total_reward

# 评估训练所得的agent,此处规定跑五个episode，取平均
def evaluate(env, agent, render=False):
    eval_reward = []
    for i in range(5):
        obs = env.reset()
        episode_reward = 0
        while True:
            action = agent.predict(obs)
            next_obs, reward, done, _ = env.step(action)
            episode_reward += reward
            obs = next_obs
            if render:
                env.render()
            if done:
                break
        eval_reward.append(episode_reward)
    return np.mean(eval_reward)

def show(env, agent):
    obs = env.reset()
    for _ in range(1000):
        sleep(1 / 60)
        action = agent.predict(obs)
        next_obs, reward, done, _ = env.step(action)
        obs = next_obs