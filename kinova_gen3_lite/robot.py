import pybullet as p
import numpy as np
from pathlib import Path


class KinovaLite:

    def __init__(
        self,
        urdf_path,
        basePosition,
        baseOrientation,
        jointStartPositions=None,
        ee_link_id=5,
    ):

        self.robot = p.loadURDF(
            str(Path(urdf_path)),
            basePosition=basePosition,
            baseOrientation=baseOrientation,
            useFixedBase=True,
        )

        self.num_joints = p.getNumJoints(self.robot)

        self.arm_joint_ids = []
        self.gripper_joint_ids = []

        for i in range(self.num_joints):

            info = p.getJointInfo(self.robot, i)

            joint_name = info[1].decode()
            joint_type = info[2]
            link_name = info[12].decode()

            print(
                i,
                "joint:", joint_name,
                "link:", link_name,
                "type:", joint_type
            )

            if joint_name in [
                "J0",
                "J1",
                "J2",
                "J3",
                "J4",
                "J5",
            ]:
                self.arm_joint_ids.append(i)

            if joint_name in [
                "RIGHT_BOTTOM",
                "RIGHT_TIP",
                "LEFT_BOTTOM",
                "LEFT_TIP",
            ]:
                self.gripper_joint_ids.append(i)

        self.ee_link_id = ee_link_id

        print("arm_joint_ids:", self.arm_joint_ids)
        print("gripper_joint_ids:", self.gripper_joint_ids)
        print("ee_link_id:", self.ee_link_id)

        if jointStartPositions is not None:
            self.reset(jointStartPositions)

    def reset(self, jointStartPositions):

        for joint_id, q in zip(
            self.arm_joint_ids,
            jointStartPositions
        ):
            p.resetJointState(
                self.robot,
                joint_id,
                q
            )

    def get_state(self):

        joint_states = p.getJointStates(
            self.robot,
            self.arm_joint_ids
        )

        ee_state = p.getLinkState(
            self.robot,
            self.ee_link_id
        )

        state = {}

        state["joint-position"] = [
            item[0]
            for item in joint_states
        ]

        state["joint-velocity"] = [
            item[1]
            for item in joint_states
        ]

        state["joint-torque"] = [
            item[3]
            for item in joint_states
        ]

        state["ee-position"] = ee_state[4]
        state["ee-quaternion"] = ee_state[5]

        state["ee-euler"] = p.getEulerFromQuaternion(
            state["ee-quaternion"]
        )

        return state

    def inverse_kinematics(
        self,
        ee_position,
        ee_quaternion=None
    ):

        if ee_quaternion is None:

            return p.calculateInverseKinematics(
                self.robot,
                self.ee_link_id,
                list(ee_position),
                maxNumIterations=100,
                residualThreshold=1e-4,
            )

        return p.calculateInverseKinematics(
            self.robot,
            self.ee_link_id,
            list(ee_position),
            list(ee_quaternion),
            maxNumIterations=100,
            residualThreshold=1e-4,
        )

    def move_to_pose(
        self,
        ee_position,
        ee_rotz=None,
        ee_quaternion=None,
        positionGain=0.03,
        force=80,
    ):

        if ee_rotz is not None:
            ee_quaternion = p.getQuaternionFromEuler(
                [np.pi, 0, ee_rotz]
            )

        target_positions = self.inverse_kinematics(
            ee_position,
            ee_quaternion,
        )

        p.setJointMotorControlArray(
            self.robot,
            self.arm_joint_ids,
            p.POSITION_CONTROL,
            targetPositions=target_positions[:6],
            positionGains=[positionGain] * 6,
            forces=[force] * 6,
        )

    def open_gripper(self):

        if len(self.gripper_joint_ids) == 0:
            return

        target_positions = [
            0.8,   # RIGHT_BOTTOM
            -0.4,  # RIGHT_TIP
            -0.8,  # LEFT_BOTTOM
            -0.4,   # LEFT_TIP
        ]

        p.setJointMotorControlArray(
            self.robot,
            self.gripper_joint_ids,
            p.POSITION_CONTROL,
            targetPositions=target_positions,
            positionGains=[0.04] * 4,
            forces=[200] * 4,
        )

    def close_gripper(self):

        if len(self.gripper_joint_ids) == 0:
            return

        # target_positions = [
        #     0.0,
        #     0.0,
        #     0.0,
        #     0.0,
        # ]
        target_positions = [
            0.0,  # RIGHT_BOTTOM
            -0.4,  # RIGHT_TIP
            0.0,  # LEFT_BOTTOM
            -0.4,  # LEFT_TIP
        ]

        p.setJointMotorControlArray(
            self.robot,
            self.gripper_joint_ids,
            p.POSITION_CONTROL,
            targetPositions=target_positions,
            positionGains=[0.05] * 4,
            forces=[500] * 4,
        )