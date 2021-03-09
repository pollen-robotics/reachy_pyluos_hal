"""Orbita Actuator abstraction."""

import struct
from math import pi
from enum import Enum
from logging import Logger
from typing import Dict, List, Optional

import numpy as np

from .register import Register


class OrbitaRegister(Enum):
    """Enum for the available Orbita Registers."""

    angle_limit = 0
    temperature_shutdown = 1

    present_position = 10
    present_speed = 11
    present_load = 12
    absolute_position = 13

    goal_position = 20
    moving_speed = 21
    torque_limit = 22

    torque_enable = 30
    pid = 31
    temperature = 32

    zero = 40
    recalibrate = 41
    magnetic_quality = 42

    fan_state = 50
    fan_trigger_temperature_threshold = 51

    position_pub_period = 60


class OrbitaActuator:
    """Orbtia Actuator abstraction."""

    register_address: Dict[str, OrbitaRegister] = {
        reg.name: reg for reg in OrbitaRegister
    }

    reduction = 52 / 24
    resolution = 4096

    def __init__(self, id: int) -> None:
        """Create 3 disks (bottom, middle, top) with their registers."""
        self.id = id

        self.disk_bottom = OrbitaDisk('disk_bottom', self.resolution, self.reduction)
        self.disk_middle = OrbitaDisk('disk_middle', self.resolution, self.reduction)
        self.disk_top = OrbitaDisk('disk_top', self.resolution, self.reduction)
        self.disks = [self.disk_top, self.disk_middle, self.disk_bottom]

        self.logger: Optional[Logger] = None

    def __str__(self) -> str:
        """Get Orbita Actuator string representation."""
        return f'<OrbitaActuator id={self.id}>'

    def get_disks_name(self) -> List[str]:
        """Get the name of each disk."""
        return [d.name for d in self.disks]

    def get_joints_name(self) -> List[str]:
        """Get the name of each joint (disk + fake RPY joint)."""
        return self.get_disks_name() + ['roll', 'pitch', 'yaw']

    def get_id_for_disk(self, disk_name: str) -> int:
        """Get the index for a specified disk."""
        disk = getattr(self, disk_name)
        return self.disks.index(disk)

    def get_value_as_usi(self, register: OrbitaRegister) -> List[float]:
        """Get the value for each disk of the specified register."""
        return [
            getattr(disk, register.name).get_as_usi()
            for disk in self.disks
        ]

    def clear_value(self, register: OrbitaRegister):
        """Clear the value for each disk of the specified register."""
        for disk in self.disks:
            getattr(disk, register.name).reset()

    def update_value(self, register: OrbitaRegister, values: bytes):
        """Update the value for each disk of the specified register using raw values."""
        assert (len(values) % 3) == 0

        n = len(values) // 3
        disk_values = [values[i * n: (i + 1) * n] for i in range(3)]

        for disk, val in zip(self.disks, disk_values):
            getattr(disk, register.name).update(val)

    def set_offset(self, zeros: List[int], start_pos: List[int]):
        """Set the correct offset depending on the hardware zero and orbita starting position."""
        for disk, z, p in zip(self.disks, zeros, start_pos):
            disk.set_offset(z, p)


class OrbitaDisk:
    """Single Orbita disk abstraction."""

    def __init__(self, name: str, resolution: int, reduction: float) -> None:
        """Create all Orbita Register."""
        self.name = name

        self.present_position = Register(self.position_as_usi, self.position_as_raw)
        self.goal_position = Register(self.position_as_usi, self.position_as_raw)
        self.torque_limit = Register(self.max_torque_as_usi, self.max_torque_as_raw)
        self.temperature = Register(self.temperature_as_usi, self.temperature_as_raw)
        self.temperature_shutdown = Register(self.temperature_as_usi, self.temperature_as_raw)
        self.torque_enable = Register(self.torque_enable_as_usi, self.torque_enable_as_raw)
        self.angle_limit = Register(self.limits_as_usi, self.limits_as_raw)
        self.pid = Register(self.gain_as_usi, self.gain_as_raw)
        self.zero = Register(self.encoder_position_as_usi, self.encoder_position_as_raw)
        self.absolute_position = Register(self.encoder_position_as_usi, self.encoder_position_as_raw)
        self.magnetic_quality = Register(self.quality_as_usi, self.quality_as_raw)
        self.fan_state = Register(self.state_as_usi, self.state_as_raw)
        self.fan_trigger_temperature_threshold = Register(self.temperature_as_usi, self.temperature_as_raw)
        self.position_pub_period = Register(self.period_as_usi, self.period_as_raw)

        self.resolution = resolution
        self.reduction = reduction

        self.offset: int = 0

    def set_offset(self, raw_zero: int, raw_pos: int):
        """Set the correct offset depending on the hardware zero and the disk starting position."""
        possibilities = [
            raw_zero,
            raw_zero + self.resolution,
            raw_zero - self.resolution,
        ]
        distances = [abs(raw_pos - poss) for poss in possibilities]
        closest = np.argmin(distances)
        self.offset = 0
        self.offset = possibilities[closest] + self.encoder_position_as_usi(self.position_as_raw(np.deg2rad(60)))

    def encoder_position_as_raw(self, val: int) -> bytes:
        """Convert encoder value to raw."""
        return struct.pack('i', val)

    def encoder_position_as_usi(self, val: bytes) -> int:
        """Convert raw position to encoder value."""
        return struct.unpack('i', val)[0]

    def position_as_usi(self, val: bytes) -> float:
        """Convert raw position as USI."""
        encoder_value = self.encoder_position_as_usi(val)
        encoder_value -= self.offset
        rads = 2 * pi * encoder_value / self.resolution
        return rads / self.reduction

    def position_as_raw(self, val: float) -> bytes:
        """Convert USI position as raw value."""
        rads = val * self.reduction
        encoder_value = rads * self.resolution / (2 * pi)
        encoder_value += self.offset
        encoder_value = int(round(encoder_value))
        return struct.pack('i', encoder_value)

    def temperature_as_usi(self, val: bytes) -> float:
        """Convert raw temperature as USI (degree celsius)."""
        return struct.unpack('f', val)[0]

    def temperature_as_raw(self, val: float) -> bytes:
        """Convert temperature as raw value."""
        return struct.pack('f', val)

    def torque_enable_as_usi(self, val: bytes) -> float:
        """Convert torque enable as USI (0 or 1)."""
        return 0.0 if val[0] == 0 else 1.0

    def torque_enable_as_raw(self, val: float) -> bytes:
        """Convert torque enable as raw value."""
        return bytes([0]) if val == 0.0 else bytes([1])

    def max_torque_as_usi(self, val: bytes) -> float:
        """Convert max torque as USI (%)."""
        return struct.unpack('f', val)[0]

    def max_torque_as_raw(self, val: float) -> bytes:
        """Convert max torque as raw value."""
        return struct.pack('f', val)

    def gain_as_usi(self, val: bytes) -> List[float]:
        """Convert gain as USI."""
        return struct.unpack('fff', val)

    def gain_as_raw(self, val: List[float]) -> bytes:
        """Convert gain as raw value."""
        return struct.pack('fff', *val)

    def limits_as_usi(self, val: bytes) -> List[float]:
        """Convert limits angle as USI value."""
        nb_val = len(val) // 4
        return struct.unpack('i' * nb_val, val)

    def limits_as_raw(self, val: List[float]) -> bytes:
        """Convert limits angle as raw value."""
        return struct.pack('i' * len(val), *val)

    def quality_as_usi(self, val: bytes) -> float:
        """Convert magnetic quality to USI (0 = Green, 1 = Orange, 2 = red)."""
        quality = struct.unpack('B' * len(val), val)
        if sum(quality) == 0:
            return 0
        elif sum(quality) == 2:
            return 1
        else:
            return 2

    def quality_as_raw(self, val: List[float]) -> bytes:
        """Convert magnetic quality to raw (MagInc, MagDec, Linearity)."""
        raise NotImplementedError

    def period_as_usi(self, val: bytes) -> int:
        """Convert period to usi."""
        return struct.unpack('H', val)[0]

    def period_as_raw(self, val: int) -> bytes:
        """Convert period to raw."""
        return struct.pack('H', val)

    def state_as_usi(self, val: bytes) -> bool:
        """Convert state to bool."""
        return val[0] == 1

    def state_as_raw(self, state: bool) -> bytes:
        """Convert state as raw."""
        return bytes([state])
