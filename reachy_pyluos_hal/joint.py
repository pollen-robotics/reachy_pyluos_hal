"""Joint abstraction.

It can be any DynamixelMotor or an OrbitaMotor.
"""

from abc import ABC
from typing import List

from .register import Register


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

    def is_value_set(self, register: str) -> bool:
        """Check if the register has been set since last reset."""
        return self.registers[register].is_set()

    def clear_value(self, register: str):
        """Clear the specified value, meaning its value should be make obsolete."""
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

