#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from re import S
from tracemalloc import start
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pybullet as p
import time
import pybullet_data

# 连接物理引擎
physicsCilent = p.connect(p.GUI)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置环境重力加速度
p.setGravity(0, 0, -10)

# 加载URDF模型，此处是加载蓝白相间的陆地
planeId = p.loadURDF("plane.urdf")

# 加载机器人，并设置加载的机器人的位姿
startPos = [0, 0, 10] #机器人中心（猜测）的(x,y,z)
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r2d2.urdf", startPos, startOrientation)

# 按照位置和朝向重置机器人的位姿，由于我们之前已经初始化了机器人，所以此处加不加这句话没什么影响
p.resetBasePositionAndOrientation(boxId, startPos, startOrientation) # 尝试注释掉这句话，确实不会影响运行
height = []
time_ = []
frequency = 240 #Hz

# 仿真方法1：使用stepSimulation进行仿真
# 开始一千次迭代，也就是一千次交互，每次交互后停顿1/240
for i in range(1000):
    p.stepSimulation()
    # 获取位置与方向四元数
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    if (i%20==0):
        print(f"机器人的位置坐标为:{cubePos}\n机器人的朝向四元数为:{cubeOrn}")
        print("-" * 20)
        height.append(cubePos[2])
        time_.append(i/frequency)
    time.sleep(1 / frequency)

# 仿真方法2：使用setRealTimeSimulation进行模拟
# p.setRealTimeSimulation(1)
# duration = 8 # 运行仿真的时间
# start_time = time.time()
# while (time.time()-start_time < duration): 
#     cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#     height.append(cubePos[2])
#     time_.append(time.time()-start_time)

# 断开连接
p.disconnect()

plt.plot(time_,height)
plt.show()