#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pybullet as p
import pybullet_data
import time

# 连接引擎
_ = p.connect(p.GUI)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)

# 载入地面模型，useMaximalCoordinates加大坐标刻度可以加快加载
p.loadURDF("plane100.urdf", useMaximalCoordinates=True)

# 创建过程中不渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
# 不展示GUI的套件
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
# 禁用 tinyrenderer 
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)

p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

### 然后我们根据pybullet_data下的duck.obj来创建鸭子的视觉模型：
shift = [0, -0.02, 0] # 偏移量
scale = [1, 1, 1] # (x,y,z)方向上的比例缩放, meshScale

# 创建视觉形状和碰撞箱形状
visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="duck.obj",
    rgbaColor=[1, 1, 1, 1],
    specularColor=shift,
    visualFramePosition=shift, # 视觉形状相对于关节坐标系的平移偏移
    meshScale=scale
)

### 再根据pybullet_data下的duck_vhacd.obj文件来创建鸭子的碰撞箱模型：
collision_shape_id = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="duck_vhacd.obj",
    collisionFramePosition=shift, # 碰撞形状相对于关节坐标系的平行偏移
    meshScale=scale
)

print("visual_shape_id = ", visual_shape_id)
print("collision_shape_id = ", collision_shape_id)

## 创建完视觉模型和碰撞箱模型后，我们需要将两者使用createMultiBody结合起来：
# 使用创建的视觉形状和碰撞箱形状使用createMultiBody将两者结合在一起
# 创建一只鸭子
p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=collision_shape_id,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=[0, 0, 3],
    useMaximalCoordinates=True
)

# 创建三只鸭子
# 使用创建的视觉形状和碰撞箱形状使用createMultiBody将两者结合在一起
# for i in range(3):
#     p.createMultiBody(
#         baseMass=1,
#         baseCollisionShapeIndex=collision_shape_id,
#         baseVisualShapeIndex=visual_shape_id,
#         basePosition=[0, 0, 2 * i],
#         useMaximalCoordinates=True
#     )

# 创建结束，重新开启渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

while (1):
    time.sleep(1./240.)