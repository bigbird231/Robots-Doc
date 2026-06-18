import pybullet as p
import pybullet_data
import time
from pathlib import Path

p.connect(p.GUI)

p.resetDebugVisualizerCamera(
    cameraDistance=0.9,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.4]
)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")

urdf_path = Path("./GEN3-LITE-with-gripper.urdf")

robot_id = p.loadURDF(
    str(urdf_path),
    basePosition=[0, 0, 0],
    useFixedBase=True
)

num_joints = p.getNumJoints(robot_id)
print("num_joints:", num_joints)

for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(i, info[1].decode(), "type:", info[2])

# move first 6 revolute joints to a simple pose
target_q = [0, -0.5, 0.5, 0, 0.5, 0]

joint_ids = []
for i in range(num_joints):
    joint_type = p.getJointInfo(robot_id, i)[2]
    if joint_type == p.JOINT_REVOLUTE:
        joint_ids.append(i)

for _ in range(1000):
    for j, q in zip(joint_ids[:6], target_q):
        p.setJointMotorControl2(
            robot_id,
            j,
            p.POSITION_CONTROL,
            targetPosition=q,
            force=100
        )

    p.stepSimulation()
    time.sleep(1 / 240)