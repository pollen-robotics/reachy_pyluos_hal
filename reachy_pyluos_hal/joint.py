"""Joint abstraction.

It can be any DynamixelMotor or an OrbitaMotor.
"""

from abc import ABC
from typing import List, Optional
from threading import Event

import time


class Joint(ABC):
    """Joint abstraction."""

    def __init__(self) -> None:
        """Set up internal registers."""
        self.registers = {
            reg: Register()
            for reg in self.get_requested_registers()
        }

    def get_requested_registers(self) -> List[str]:
        """Get the registers that should be implemented by all types of joint."""
        return [
            'torque_enable',
            'goal_position',
            'moving_speed',
            'torque_limit',
            'present_position',
            'present_temperature',
        ]

    def clear_value(self, register: str):
        """Clear the specified value, meaning its value should be make obsolete."""
        if register in ('present_position', 'present_temperature'):
            # Those two registers are automatically sent at a pre-defined freq.
            # So the value should never be obsolete
            return
        self.registers[register].reset()

    def get_value(self, register: str) -> bytes:
        """Get the up-to-date specified value."""
        return self.registers[register].get()

    def update_value(self, register: str, val: bytes):
        """Update the specified register with the raw value received from a gate."""
        self.registers[register].update(val)

    def convert_to_usi(self, register: str, raw_value: bytes) -> float:
        """Convert a raw value to its USI value."""
        ...

    def convert_to_raw(self, register: str, usi_value: float) -> bytes:
        """Convert a USI value to its raw representation."""
        ...


class Register:
    """Synced register object."""

    def __init__(self) -> None:
        """Set up the register with a None value by default."""
        self.val: Optional[bytes] = None
        self.timestamp = 0.0
        self.synced = Event()

    def update(self, val):
        """Update the register with a value retrieve from its associated gate."""
        self.val = val
        self.timestamp = time.time()
        self.synced.set()

    def get(self) -> bytes:
        """Wait for an updated value and returns it."""
        self.synced.wait()
        assert self.val is not None
        return self.val

    def reset(self):
        """Mark the value as obsolete."""
        self.synced.clear()
        self.val = None
