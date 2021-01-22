"""Device type annotation."""

from typing import Union

from .dynamixel import DynamixelMotor
from .force_sensor import ForceSensor
from .joint import Joint
from .orbita import OrbitaActuator

Device = Union[Joint, DynamixelMotor, ForceSensor, OrbitaActuator]
