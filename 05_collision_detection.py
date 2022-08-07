#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pybullet as p
import pybullet_data
from time import sleep

use_gui = True
if use_gui:
    serve_id = p.connect(p.GUI)
else:
    serve_id = p.connect(p.DIRECT)

# 配置渲染机制
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # 关闭GUI边上的一些控件
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0) # 禁用集显
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())
_ = p.loadURDF("plane.urdf", useMaximalCoordinates=True)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=True)

# 定义墙体尺寸
wall_size = [60, 5, 5]

# 创建一面墙
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=wall_size
)

collison_box_id = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=wall_size
)

wall_id = p.createMultiBody(
    baseMass=10000,
    baseCollisionShapeIndex=collison_box_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[0, 10, 5]
)

# 重新开始渲染，开启重力
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)

for i in range(p.getNumJoints(robot_id)):
    if "wheel" in p.getJointInfo(robot_id, i)[1].decode("utf-8"):           # 如果是轮子的关节，则为马达配置参数，否则禁用马达
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=30,
            force=100
        )
    else:
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i,
            controlMode=p.VELOCITY_CONTROL,
            force=0
        )

# 主循环中为r2d2机器人配备碰撞检测，一旦它与物体相撞了，
# 那么就在控制台上打印碰撞的物体的ID与名字
while True:
    p.stepSimulation()
    P_min, P_max = p.getAABB(robot_id)
    # print("P_min=",P_min," P_max=",P_max)
    id_tuple = p.getOverlappingObjects(P_min, P_max)
    if len(id_tuple) > 1: # 机器人与自己重叠，getOverlappingObjects无论如何都会返回一个值。
        for ID, _ in id_tuple:
            if ID == robot_id:
                continue
            else:
                print(f"hit happen! hit object is {p.getBodyInfo(ID)}")
    sleep(1 / 240)

p.disconnect(serve_id)