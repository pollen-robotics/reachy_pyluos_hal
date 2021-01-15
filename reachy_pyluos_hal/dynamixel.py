"""Dynamixel implentation of a Joint."""

from typing import Union

from .joint import Joint


class DynamixelMotor(Joint):
    """Dynamixel implentation of a Joint."""

    register_config = {
        'torque_enable': (24, 1),
        'goal_position': (30, 2),
        'moving_speed': (32, 2),
        'torque_limit': (34, 2),
        'present_position': (36, 2),
        'present_temperature': (43, 1),
    }

    def __init__(self, id: int, offset: float, direct: bool) -> None:
        """Set up the dynamixel motor with its id, and an offset and direction."""
        self.id = id
        self.offset = offset
        self.direct = direct

    def convert_to_raw(self, register: str, value: float) -> bytes:
        """Convert a raw value to its USI value."""
        if register == 'torque_enable':
            return bytes([1 if value else 0])

        return bytes()

    def convert_to_usi(self, register: str, value: bytes) -> Union[float, bool]:
        """Convert a USI value to its raw representation."""
        if register == 'torque_enable':
            return value[0] == 1

        return 42.0


MX106 = MX64 = AX18 = DynamixelMotor
