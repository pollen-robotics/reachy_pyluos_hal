"""Implementation of the joint reachy_ros_hal via serial communication to the luos board."""

from typing import Dict, List, Optional, Tuple
from logging import Logger

from .reachy import Reachy


class JointLuos:
    """Implementation of the joint hal via serial communication to the luos boards."""

    def __init__(self, config_filename: str, logger: Logger) -> None:
        """Create and start Reachy which wraps serial Luos GateClients."""
        self.logger = logger
        self.config_filename = config_filename

    def __enter__(self):
        """Enter context handler."""
        self.reachy = Reachy(config_filename=self.config_filename, logger=self.logger)
        self.reachy.__enter__()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Stop and close."""
        self.stop()

    def stop(self) -> None:
        """Stop and close."""
        self.reachy.stop()

    def get_all_joint_names(self) -> List[str]:
        """Return the names of all joints."""
        return self.reachy.get_all_joints_names()

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
        return self.reachy.get_joints_value(register='temperature', joint_names=names)

    def get_joint_pids(self, names: List[str]) -> List[Tuple[float, float, float]]:
        """Return the current PIDs of the specified joints.

        You should refer to the documentation of dynamixel for a better understanding of the range of values.
        The AX dynamixel motors do not have PID register so their value should be ignored.
        """
        return self.reachy.get_joints_pid(joint_names=names)

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

    def set_goal_positions(self, goal_positions: Dict[str, float]) -> bool:
        """Set new goal positions for the specified joints."""
        try:
            self.reachy.set_joints_value('goal_position', goal_positions)
            return True
        except (ValueError, TimeoutError) as e:
            self.logger.warning(f'Set_goal_positions failed with error {e}')
            return False

    def set_goal_velocities(self, goal_velocities: Dict[str, float]) -> bool:
        """Set new goal velocities for the specified joints."""
        try:
            self.reachy.set_joints_value('moving_speed', goal_velocities)
            return True
        except (ValueError, TimeoutError) as e:
            self.logger.warning(f'Set_goal_velocities failed with error {e}')
            return False

    def set_goal_efforts(self, goal_efforts: Dict[str, float]) -> bool:
        """Set new goal efforts for the specified joints."""
        try:
            self.reachy.set_joints_value('torque_limit', goal_efforts)
            return True
        except (ValueError, TimeoutError) as e:
            self.logger.warning(f'Set_goal_efforts failed with error {e}')
            return False

    def set_goal_pids(self, goal_pids: Dict[str, Tuple[float, float, float]]) -> bool:
        """Set the new PIDs to the specified joints.

        You should refer to the documentation of dynamixel for a better understanding of the range of values.
        The AX dynamixel motors do not have PID register so their value will be ignored.
        """
        try:
            self.reachy.set_joints_pid(goal_pids)
            return True
        except (ValueError, TimeoutError) as e:
            self.logger.warning(f'Set_goal_pids failed with error {e}')
            return False

    def set_compliance(self, compliances: Dict[str, bool]) -> bool:
        """Set new compliances for the specified joints."""
        try:
            self.reachy.set_joints_value('torque_enable', {
                name: 0 if compliant else 1
                for name, compliant in compliances.items()
            })
            return True
        except (ValueError, TimeoutError) as e:
            self.logger.warning(f'Set_compliance failed with error {e}')
            return False

    def get_all_force_sensor_names(self) -> List[str]:
        """Return the names of all force sensors."""
        return list(self.reachy.force_sensors.keys())

    def get_force(self, names: List[str]) -> List[float]:
        """Return the current force of the specified sensors."""
        return [self.reachy.force_sensors[name].get_force() for name in names]

    def get_all_fan_names(self) -> List[str]:
        """Return the names of all fans."""
        return list(self.reachy.fans.keys())

    def get_fans_state(self, fan_names: List[str]) -> List[bool]:
        """Get states for the specified fans."""
        return [state == 1.0 for state in self.reachy.get_fans_state(fan_names)]

    def set_fans_state(self, fan_states: Dict[str, bool]) -> bool:
        """Set states for the specified fans."""
        self.reachy.set_fans_state({name: 1 if state else 0 for name, state in fan_states.items()})
        return True
