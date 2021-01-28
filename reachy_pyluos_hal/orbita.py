"""Orbita Actuator abstraction."""

import struct
import numpy as np

from enum import Enum
from typing import Dict, List

from .register import Register


class OrbitaRegister(Enum):
    """Enum for the available Orbita Registers."""

    angle_limit = 0
    temperature_shutdown = 1

    present_position = 10
    present_speed = 11
    present_load = 12

    goal_position = 20
    max_speed = 21
    max_torque = 22

    compliant = 30
    pid = 31
    temperature = 32


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

        self.disk_bottom = OrbitaDisk(self.reduction, self.resolution)
        self.disk_middle = OrbitaDisk(self.reduction, self.resolution)
        self.disk_top = OrbitaDisk(self.reduction, self.resolution)
        self.disks = [self.disk_top, self.disk_middle, self.disk_bottom]

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


class OrbitaDisk:
    """Single Orbita disk abstraction."""

    def __init__(self, resolution, reduction) -> None:
        """Create all Orbita Register."""
        self.present_position = Register(self.position_as_usi, self.position_as_raw)
        self.goal_position = Register(self.position_as_usi, self.position_as_raw)
        self.max_torque = Register(self.max_torque_as_usi, self.max_torque_as_raw)
        self.temperature = Register(self.temperature_as_usi, self.temperature_as_raw)
        self.temperature_shutdown = Register(self.temperature_as_usi, self.temperature_as_raw)
        self.compliant = Register(self.compliant_as_usi, self.compliant_as_raw)
        self.angle_limit = Register(self.limits_as_usi, self.limits_as_raw)
        self.pid = Register(self.gain_as_usi, self.gain_as_raw)

        self.resolution = resolution
        self.reduction = reduction

    def position_as_usi(self, val: bytes) -> float:
        """Convert raw position as USI."""
        encoder_value = struct.unpack('i', val)[0]
        rads = 2 * np.pi * encoder_value / self.resolution
        return rads / self.reduction

    def position_as_raw(self, val: float) -> bytes:
        """Convert USI position as raw value."""
        rads = val * self.reduction
        encoder_value = rads * self.resolution / (2 * np.pi)
        encoder_value = round(encoder_value)
        return struct.pack('i', encoder_value)

    def temperature_as_usi(self, val: bytes) -> float:
        """Convert raw temperature as USI (degree celsius)."""
        return struct.unpack('f', val)[0]

    def temperature_as_raw(self, val: float) -> bytes:
        """Convert temperature as raw value."""
        return struct.pack('f', val)

    def compliant_as_usi(self, val: bytes) -> float:
        """Convert compliancy as USI (0 or 1)."""
        return 0.0 if val[0] == 0 else 1.0

    def compliant_as_raw(self, val: float) -> bytes:
        """Convert compliant as raw value."""
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

