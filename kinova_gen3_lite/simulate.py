import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from objects import objects
from robot import KinovaLite

# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
# disable keyboard shortcuts so they do not interfere with keyboard control
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(
    cameraDistance=1.0,
    cameraYaw=40.0,
    cameraPitch=-30,
    cameraTargetPosition=[0.5, 0.0, 0.2]
)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", basePosition=[0, 0, -0.625])
p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.625])
cube1 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.1, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.7]))

# load the robot
kinova = KinovaLite(
    urdf_path="kinova_lite/GEN3-LITE-with-gripper.urdf",
    basePosition=[0, 0, 0],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    jointStartPositions=[0.5, -0.5, 0.9, 0.0, 0.5, 0.0],
)
kinova.open_gripper()


target = cube1.get_state()
target_position = target["position"]
ee_target_position = [
    target_position[0],
    target_position[1],
    target_position[2] + 0.15,
]
ee_target_orientation = np.array(target["euler"])
# flip to make the gripper point downward toward the cube
ee_target_orientation[0] += np.pi
# converts euler angles to quaternion: euler=(rx, ry, rz); quaternion=(qw, qx, qy, qz)
target_quaternion = p.getQuaternionFromEuler(ee_target_orientation)
target_position = ee_target_position

count = 0
while True:
    count += 1
    if count < 500:
        kinova.move_to_pose(
            ee_position=target_position,
            ee_quaternion=target_quaternion,
            positionGain=0.01,
            force=60,
        )
    else:
        kinova.close_gripper()

    p.stepSimulation()
    time.sleep(control_dt)