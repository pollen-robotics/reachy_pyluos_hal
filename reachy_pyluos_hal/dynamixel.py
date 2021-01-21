"""Dynamixel implentation of a Joint."""

from abc import abstractproperty
from numpy import clip, deg2rad, pi
from typing import Union
from struct import pack, unpack


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
        super().__init__()

        self.id = id
        self.offset = offset
        self.direct = direct

    def __repr__(self) -> str:
        """Represent DynamixelMotor."""
        return f'<DynamixelMotor type="{self.motor_type}" id={self.id}>'

    @abstractproperty
    def max_position(self) -> int:
        """Return the max position dynamixel register value."""
        ...

    @abstractproperty
    def max_radian(self) -> float:
        """Return the max position (in rad)."""
        ...

    @abstractproperty
    def motor_type(self) -> str:
        """Return the motor type."""
        ...

    @classmethod
    def find_register(cls, addr: int) -> str:
        """Find register name by its address."""
        for reg, (reg_addr, _) in cls.register_config.items():
            if reg_addr == addr:
                return reg
        raise KeyError(addr)

    def convert_to_raw(self, register: str, value: float) -> bytes:
        """Convert a raw value to its USI value."""
        if register == 'torque_enable':
            return self.torque_enabled_to_raw(value)
        if register in ('goal_position', 'present_position'):
            return self.position_to_raw(value)
        if register == 'moving_speed':
            return self.speed_to_raw(value)
        if register == 'torque_limit':
            return self.torque_to_raw(value)
        if register == 'present_temperature':
            return self.temperature_to_raw(value)

        raise KeyError(register)

    def convert_to_usi(self, register: str, value: bytes) -> Union[float, bool]:
        """Convert a USI value to its raw value."""
        if register == 'torque_enable':
            return self.torque_enabled_to_usi(value)
        if register in ('goal_position', 'present_position'):
            return self.position_to_usi(value)
        if register == 'moving_speed':
            return self.speed_to_usi(value)
        if register == 'torque_limit':
            return self.torque_to_usi(value)
        if register == 'present_temperature':
            return self.temperature_to_usi(value)

        raise KeyError(register)

    def torque_enabled_to_raw(self, value: float) -> bytes:
        """Convert torque-enabled to raw."""
        return bytes([1 if value else 0])

    def torque_enabled_to_usi(self, value: bytes) -> float:
        """Convert torque-enabled to usi."""
        return value[0]

    def position_to_raw(self, value: float) -> bytes:
        """Convert position (in rad) to raw."""
        value = (value + self.offset) * (1 if self.direct else -1)
        pos_ratio = (value + self.max_radian / 2) / self.max_radian
        dxl_raw_pos = int(round(pos_ratio * (self.max_position - 1), 0))
        return pack('H', dxl_raw_pos)

    def position_to_usi(self, value: bytes) -> float:
        """Convert position to usi (in rad)."""
        dxl_raw_pos = unpack('H', value)[0]
        pos_ratio = dxl_raw_pos / (self.max_position - 1)
        pos = (pos_ratio * self.max_radian) - self.max_radian / 2
        return (pos if self.direct else -pos) - self.offset

    def speed_to_raw(self, value: float) -> bytes:
        """Convert speed (in rad/s) to raw."""
        assert value >= 0
        rpm = abs(value) / (2 * pi) * 60
        dxl_speed = clip(int(rpm / 0.114), 0, 1023)
        return pack('H', dxl_speed)

    def speed_to_usi(self, value: bytes) -> float:
        """Convert speed to usi (in rad/s)."""
        dxl_speed = unpack('H', value)[0]
        if dxl_speed > 1023:
            cw = True
            dxl_speed - 1024
        else:
            cw = False
        rpm = dxl_speed * 0.114
        rad_per_s = rpm / 60 * (2 * pi)
        return -rad_per_s if cw else rad_per_s

    def torque_to_raw(self, value: float) -> bytes:
        """Convert torque (in %) to raw."""
        return pack('H', clip(value, 0, 100) * 10.23)

    def torque_to_usi(self, value: bytes) -> float:
        """Convert torque to usi (in %)."""
        return unpack('H', value)[0] / 10.23

    def temperature_to_raw(self, value: float) -> bytes:
        """Convert temperature (in C) to raw."""
        return bytes([clip(value, 0, 255)])

    def temperature_to_usi(self, value: bytes) -> float:
        """Convert temperature to usi (in C)."""
        return float(value[0])


class MX(DynamixelMotor):
    """MX specific value."""

    @property
    def max_position(self) -> int:
        """Return the max position dynamixel register value."""
        return 4096

    @property
    def max_radian(self) -> float:
        """Return the max position (in rad)."""
        return deg2rad(360)


class AX18(DynamixelMotor):
    """AX specific value."""

    @property
    def max_position(self) -> int:
        """Return the max position dynamixel register value."""
        return 1024

    @property
    def max_radian(self) -> float:
        """Return the max position (in rad)."""
        return deg2rad(300)

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'AX18'


class MX106(MX):
    """MX106 impl."""

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'MX106'


class MX64(MX):
    """MX64 impl."""

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'MX64'


class MX28(MX):
    """MX28 impl."""

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'MX28'
