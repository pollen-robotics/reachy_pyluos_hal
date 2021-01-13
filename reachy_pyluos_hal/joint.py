"""Pyluos implementation of the joint reachy_ros_hal."""

from logging import Logger
from typing import Dict, List, Optional

import numpy as np

from reachy_ros_hal.joint import JointABC

from .reachy import Reachy


class JointPyluos(JointABC):
    """Pyluos implementation of the joint reachy_ros_hal."""

    def __init__(self, logger: Logger, port_template='/dev/ttyUSB*') -> None:
        super().__init__(logger=logger)

        # self.logger.info(f'Connecting to the robot via "{port_template}"...')
        self.reachy = Reachy(
            left_arm_port='/dev/cu.usbserial-D307RR2J',
            right_arm_port='/dev/cu.usbserial-DN05NM0W',
        )
        # self.reachy.run4ever()
        # self.logger.info('Connection ok!')

        self.name2mod = {
            'r_shoulder_pitch': self.reachy.dxls[10],
            'r_shoulder_roll': self.reachy.dxls[11],
            'r_arm_yaw': self.reachy.dxls[12],
            'r_elbow_pitch': self.reachy.dxls[13],
            'r_forearm_yaw': self.reachy.dxls[14],
            'r_wrist_pitch': self.reachy.dxls[15],
            'r_wrist_roll': self.reachy.dxls[16],
            'r_gripper': self.reachy.dxls[17],

            'l_shoulder_pitch': self.reachy.dxls[20],
            'l_shoulder_roll': self.reachy.dxls[21],
            'l_arm_yaw': self.reachy.dxls[22],
            'l_elbow_pitch': self.reachy.dxls[23],
            'l_forearm_yaw': self.reachy.dxls[24],
            'l_wrist_pitch': self.reachy.dxls[25],
            'l_wrist_roll': self.reachy.dxls[26],
            'l_gripper': self.reachy.dxls[27],
        }

    def get_all_joint_names(self) -> List[str]:
        """Return the names of all  joints."""
        return [
            'l_shoulder_pitch',
            'l_shoulder_roll',
            'l_arm_yaw',
            'l_elbow_pitch',
            'l_forearm_yaw',
            'l_wrist_pitch',
            'l_wrist_roll',
            'l_gripper',
            'r_shoulder_pitch',
            'r_shoulder_roll',
            'r_arm_yaw',
            'r_elbow_pitch',
            'r_forearm_yaw',
            'r_wrist_pitch',
            'r_wrist_roll',
            'r_gripper',
        ]

    def get_joint_positions(self, names: List[str]) -> Optional[List[float]]:
        """Return the current position (in rad) of the specified joints."""
        return [
            self.name2mod[name].get_value('present_position')
            for name in names
        ]

    def get_joint_velocities(self, names: List[str]) -> Optional[List[float]]:
        pass

    def get_joint_efforts(self, names: List[str]) -> Optional[List[float]]:
        pass

    def get_joint_temperatures(self, names: List[str]) -> List[float]:
        return [self.name2mod[name].get_value('present_temperature') for name in names]

    def get_goal_positions(self, names: List[str]) -> List[float]:
        return [
            self.name2mod[name].get_value('goal_position')
            for name in names
        ]

    def set_goal_positions(self, goal_positions: Dict[str, float]) -> None:
        from reachy_pyluos_hal.message import Message, MsgType
        from reachy_pyluos_hal.dynamixel import DXL_REGISTER
        from reachy_pyluos_hal.dxl_convert import as_bytes

        reg, nb_bytes, _, cvt = DXL_REGISTER['goal_position']
        left_arm_payload = [MsgType.MSG_TYPE_DXL_SET_MULTIPLE_REG, reg, nb_bytes]
        right_arm_payload = [MsgType.MSG_TYPE_DXL_SET_MULTIPLE_REG, reg, nb_bytes]

        for name, pos in goal_positions.items():
            dxl = self.name2mod[name]
            l = [dxl.id] + as_bytes(cvt(pos, dxl.model, dxl.offset, dxl.direct), nb_bytes)

            if dxl.gate == self.reachy.right_arm:
                right_arm_payload.extend(l)
            elif dxl.gate == self.reachy.left_arm:
                left_arm_payload.extend(l)

        if len(right_arm_payload) > 3:
            self.reachy.right_arm.send_msg(Message(right_arm_payload))
        if len(left_arm_payload) > 3:
            self.reachy.left_arm.send_msg(Message(left_arm_payload))


    def get_goal_velocities(self, names: List[str]) -> List[float]:
        return [
            self.name2mod[name].get_value('moving_speed')
            for name in names
        ]

    def set_goal_velocities(self, goal_velocities: Dict[str, float]) -> None:
        for name, vel in goal_velocities.items():
            self.name2mod[name].moving_speed = np.rad2deg(vel)

    def get_goal_efforts(self, names: List[str]) -> List[float]:
        return [
            self.name2mod[name].get_value('torque_limit')
            for name in names
        ]

    def set_goal_efforts(self, goal_efforts: Dict[str, float]) -> None:
        for name, effort in goal_efforts.items():
            self.name2mod[name].torque_limit = effort

    def get_compliant(self, names: List[str]) -> List[bool]:
        return [self.name2mod[name].get_value('torque_enable') == 0 for name in names]

    def set_compliance(self, compliances: Dict[str, bool]) -> bool:
        for name, comp in compliances.items():
            self.name2mod[name].send_value('torque_enable', 0 if comp else 1)
        return True

    def get_grip_force(self, sides: List[str]) -> List[float]:
        return [0.0 for s in sides]
