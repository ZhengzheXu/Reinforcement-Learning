#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pybullet as p
import pybullet_data
from time import sleep

use_gui = True
if use_gui:
    cid = p.connect(p.GUI)
else:
    cid = p.connect(p.DIRECT)

p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=False)
robot_id = p.loadURDF("r2d2.urdf", basePosition=[0, 0, 0.5], useMaximalCoordinates=False)


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

# print(p.KEY_WAS_TRIGGERED)  # 打印“2”
# print(p.KEY_IS_DOWN)        # 打印“1”
# print(p.KEY_WAS_RELEASED)   # 打印“4”

while True:
    p.stepSimulation()
    keys = p.getKeyboardEvents()
    
    if ord("f") in keys and keys[ord("f")] & p.KEY_WAS_TRIGGERED:
        print("f KEY_WAS_TRIGGERED")
    elif ord("f") in keys and keys[ord("f")] & p.KEY_IS_DOWN:
        print("f KEY_IS_DOWN")
    elif ord("f") in keys and keys[ord("f")] & p.KEY_WAS_RELEASED:
        print("f KEY_WAS_RELEASED")

p.disconnect(cid)