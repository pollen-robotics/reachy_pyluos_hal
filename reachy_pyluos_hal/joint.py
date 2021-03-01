"""Joint abstraction.

It can be any DynamixelMotor or an OrbitaMotor.
"""

from abc import ABC
from logging import Logger
from typing import Callable, Dict, Optional, Tuple

from .register import Register


class Joint(ABC):
    """Joint abstraction.

    Should define the following registers:
        'torque_enable'
        'goal_position'
        'moving_speed'
        'torque_limit'
        'present_position'
        'temperature'
    """

    def __init__(self, register_config: Dict[str,
                                             Tuple[
                                                 Callable[[bytes], float],
                                                 Callable[[float], bytes]
                                             ]]

                 ) -> None:
        """Set up internal registers."""
        self.registers = {
            reg: Register(cvt_as_usi, cvt_as_raw)
            for reg, (cvt_as_usi, cvt_as_raw) in register_config.items()
        }
        self.logger: Optional[Logger] = None

    def is_value_set(self, register: str) -> bool:
        """Check if the register has been set since last reset."""
        return self.registers[register].is_set()

    def clear_value(self, register: str):
        """Clear the specified value, meaning its value should be make obsolete."""
        self.registers[register].reset()

    def get_value(self, register: str) -> bytes:
        """Get the up-to-date specified value."""
        return self.registers[register].get()

    def get_value_as_usi(self, register: str) -> float:
        """Get the up-to-date specified value."""
        return self.registers[register].get_as_usi()

    def update_value(self, register: str, val: bytes):
        """Update the specified register with the raw value received from a gate."""
        self.registers[register].update(val)

    def update_value_using_usi(self, register: str, val: float):
        """Update the specified register with its USI value received from a gate."""
        self.registers[register].update_using_usi(val)
