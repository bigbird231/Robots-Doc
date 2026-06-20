import pybullet as p
import pybullet_data
import numpy as np
import os
import time
from robot import Panda
from objects import objects
from teleop import KeyboardController

# parameters
control_dt = 1. / 240.

# create simulation and place camera
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
# disable keyboard shortcuts so they do not interfere with keyboard control
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetDebugVisualizerCamera(cameraDistance=1.0,
                             cameraYaw=40.0,
                             cameraPitch=-30.0,
                             cameraTargetPosition=[0.5, 0.0, 0.2])

# load the objects
urdfRootPath = pybullet_data.getDataPath()
plane = p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.625])
table = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.625])
cube1 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.3, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.7]))
cube2 = objects.SimpleObject("cube.urdf", basePosition=[0.4, -0.2, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, -0.3]))
cube3 = objects.SimpleObject("cube.urdf", basePosition=[0.5, -0.1, 0.025], baseOrientation=p.getQuaternionFromEuler([0, 0, 0.2]))
cabinet = objects.CollabObject("cabinet.urdf", basePosition=[0.9, -0.3, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, np.pi]))
microwave = objects.CollabObject("microwave.urdf", basePosition=[0.5, 0.3, 0.2], baseOrientation=p.getQuaternionFromEuler([0, 0, -np.pi / 2]))

# load the robot
jointStartPositions = [0.0, 0.0, 0.0, -2 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4, 0.0, 0.0, 0.04, 0.04]
panda = Panda(basePosition=[0, 0, 0],
              baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
              jointStartPositions=jointStartPositions)

teleop = KeyboardController()

robot_state = panda.get_state()
target_position = robot_state["ee-position"]
target_quaternion = robot_state["ee-quaternion"]

use_teleop = False

# main loop
while True:
    robot_state = panda.get_state()

    if use_teleop:
        # teleop control logic
        action = teleop.get_action()
        target_position = target_position + action[0:3]
        target_quaternion = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(action[3:6]), [0, 0, 0], target_quaternion)[1]
        panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion, positionGain=1)

        if action[6] == +1:
            panda.open_gripper()
        elif action[6] == -1:
            panda.close_gripper()
    else:
        target = cube1.get_state()
        ee_target_position = target["position"]
        ee_target_orientation = np.array(target["euler"])
        # flip to make the gripper point downward toward the cube
        ee_target_orientation[0] += np.pi
        # converts euler angles to quaternion: euler=(rx, ry, rz); quaternion=(qw, qx, qy, qz)
        target_quaternion = p.getQuaternionFromEuler(ee_target_orientation)
        target_position = ee_target_position
        panda.move_to_pose(ee_position=target_position, ee_quaternion=target_quaternion, positionGain=0.01)

    # step the simulation
    p.stepSimulation()
    time.sleep(control_dt)
