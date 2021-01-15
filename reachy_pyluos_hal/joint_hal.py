"""Implementation of the joint reachy_ros_hal via serial communication to the luos board."""

from typing import Dict, List, Optional

from logging import Logger

from reachy_ros_hal.joint import JointABC

from .reachy import Reachy


class JointLuos(JointABC):
    """Implementation of the joint JointABC via serial communication to the luos board."""

    def __init__(self, logger: Logger) -> None:
        """Create and start Reachy which wraps serial Luos GateClients."""
        self.logger = logger

        self.reachy = Reachy(self.logger)
        self.reachy.start()

    def get_all_joint_names(self) -> List[str]:
        """Return the names of all joints."""
        return list(self.reachy.joints.keys())

    def get_joint_positions(self, names: List[str]) -> Optional[List[float]]:
        """Return the current position (in rad) of the specified joints."""
        return self.reachy.get_joints_value(register='present_position', joint_names=names)

    def get_joint_velocities(self, names: List[str]) -> Optional[List[float]]:
        """Return the current velocity (in rad/s) of the specified joints."""
        pass

    def get_joint_efforts(self, names: List[str]) -> Optional[List[float]]:
        """Return the current effort of the specified joints."""
        pass

    def get_joint_temperatures(self, names: List[str]) -> List[float]:
        """Return the current temperature (in C) of the specified joints."""
        return self.reachy.get_joints_value(register='present_temperature', joint_names=names)

    def get_goal_positions(self, names: List[str]) -> List[float]:
        """Return the goal position (in rad/s) of the specified joints."""
        return self.reachy.get_joints_value(register='goal_position', joint_names=names)

    def get_goal_velocities(self, names: List[str]) -> List[float]:
        """Return the goal velocity of the specified joints."""
        return self.reachy.get_joints_value(register='moving_speed', joint_names=names)

    def get_goal_efforts(self, names: List[str]) -> List[float]:
        """Return the goal effort of the specified joints."""
        return self.reachy.get_joints_value(register='torque_limit', joint_names=names)

    def get_compliant(self, names: List[str]) -> List[bool]:
        """Return the compliance of the specified joints."""
        is_torques_enabled = self.reachy.get_joints_value('torque_enable', names)
        return [torque == 0 for torque in is_torques_enabled]

    def set_goal_positions(self, goal_positions: Dict[str, float]) -> None:
        """Set new goal positions for the specified joints."""
        self.reachy.set_joints_value('goal_position', goal_positions)

    def set_goal_velocities(self, goal_velocities: Dict[str, float]) -> None:
        """Set new goal velocities for the specified joints."""
        self.reachy.set_joints_value('moving_speed', goal_velocities)

    def set_goal_efforts(self, goal_efforts: Dict[str, float]) -> None:
        """Set new goal efforts for the specified joints."""
        self.reachy.set_joints_value('torque_limit', goal_efforts)

    def set_compliance(self, compliances: Dict[str, bool]) -> bool:
        """Set new compliances for the specified joints."""
        self.reachy.set_joints_value('torque_enable', {
            name: 0 if compliant else 1
            for name, compliant in compliances.items()
        })
        return True

    def get_grip_force(self, sides: List[str]) -> List[float]:
        """Return the current forces of the specified grip sensors."""
        return [42.0 for _ in sides]
