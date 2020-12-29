"""Pyluos implementation of the joint reachy_ros_hal."""

from logging import Logger
from typing import Dict, List, Optional

import numpy as np

from reachy import Reachy, parts

from reachy_ros_hal.joint import JointABC


class JointPyluos(JointABC):
    """Pyluos implementation of the joint reachy_ros_hal."""

    def __init__(self, logger: Logger, port_template='/dev/ttyUSB*') -> None:
        super().__init__(logger=logger)

        self.logger.info(f'Connecting to the robot via "{port_template}"...')
        self.reachy = Reachy(
            left_arm=parts.LeftArm(io=port_template, hand='force_gripper'),
            right_arm=parts.RightArm(io=port_template, hand='force_gripper'),
        )
        self.logger.info('Connection ok!')

        self.name2mod = {
            'r_shoulder_pitch': self.reachy.right_arm.shoulder_pitch,
            'r_shoulder_roll': self.reachy.right_arm.shoulder_roll,
            'r_arm_yaw': self.reachy.right_arm.arm_yaw,
            'r_elbow_pitch': self.reachy.right_arm.elbow_pitch,
            'r_forearm_yaw': self.reachy.right_arm.hand.forearm_yaw,
            'r_wrist_pitch': self.reachy.right_arm.hand.wrist_pitch,
            'r_wrist_roll': self.reachy.right_arm.hand.wrist_roll,
            'r_gripper': self.reachy.right_arm.hand.gripper,

            'l_shoulder_pitch': self.reachy.left_arm.shoulder_pitch,
            'l_shoulder_roll': self.reachy.left_arm.shoulder_roll,
            'l_arm_yaw': self.reachy.left_arm.arm_yaw,
            'l_elbow_pitch': self.reachy.left_arm.elbow_pitch,
            'l_forearm_yaw': self.reachy.left_arm.hand.forearm_yaw,
            'l_wrist_pitch': self.reachy.left_arm.hand.wrist_pitch,
            'l_wrist_roll': self.reachy.left_arm.hand.wrist_roll,
            'l_gripper': self.reachy.left_arm.hand.gripper,
        }

    def get_all_joint_names(self) -> List[str]:
        """Return the names of all  joints."""
        return [
            'r_shoulder_pitch',
            'r_shoulder_roll',
            'r_arm_yaw',
            'r_elbow_pitch',
            'r_forearm_yaw',
            'r_wrist_pitch',
            'r_wrist_roll',
            'r_gripper',
            'l_shoulder_pitch',
            'l_shoulder_roll',
            'l_arm_yaw',
            'l_elbow_pitch',
            'l_forearm_yaw',
            'l_wrist_pitch',
            'l_wrist_roll',
            'l_gripper',
        ]

    def get_joint_positions(self, names: List[str]) -> Optional[List[float]]:
        """Return the current position (in rad) of the specified joints."""
        return list(np.deg2rad([
            self.name2mod[name].present_position
            for name in names
        ]))

    def get_joint_velocities(self, names: List[str]) -> Optional[List[float]]:
        pass

    def get_joint_efforts(self, names: List[str]) -> Optional[List[float]]:
        pass

    def get_joint_temperatures(self, names: List[str]) -> List[float]:
        return [self.name2mod[name].temperature for name in names]

    def get_goal_positions(self, names: List[str]) -> List[float]:
        goal_pos = []

        for name in names:
            mod = self.name2mod[name]
            if mod._motor.target_rot_position is None:
                goal_pos.append(np.deg2rad(mod.present_position))
            else:
                goal_pos.append(np.deg2rad(mod.goal_position))

        return goal_pos

    def set_goal_positions(self, goal_positions: Dict[str, float]) -> None:
        for name, pos in goal_positions.items():
            self.name2mod[name].goal_position = np.rad2deg(pos)

    def get_goal_velocities(self, names: List[str]) -> List[float]:
        goal_vel = []

        for name in names:
            mod = self.name2mod[name]
            if mod.moving_speed is None:
                goal_vel.append(np.deg2rad(700))
            else:
                goal_vel.append(np.deg2rad(mod.moving_speed))

        return goal_vel

    def set_goal_velocities(self, goal_velocities: Dict[str, float]) -> None:
        for name, vel in goal_velocities.items():
            self.name2mod[name].moving_speed = np.rad2deg(vel)

    def get_goal_efforts(self, names: List[str]) -> List[float]:
        goal_eff = []

        for name in names:
            mod = self.name2mod[name]
            if mod.torque_limit is None:
                goal_eff.append(100.0)
            else:
                goal_eff.append(mod.torque_limit)

        return goal_eff

    def set_goal_efforts(self, goal_efforts: Dict[str, float]) -> None:
        for name, effort in goal_efforts.items():
            self.name2mod[name].torque_limit = effort

    def get_compliant(self, names: List[str]) -> List[bool]:
        comp = []

        for name in names:
            mod = self.name2mod[name]
            if mod.compliant is None:
                comp.append(True)
            else:
                comp.append(mod.compliant)

        return comp

    def set_compliance(self, compliances: Dict[str, bool]) -> bool:
        for name, comp in compliances.items():
            self.name2mod[name].compliant = comp
        return True
