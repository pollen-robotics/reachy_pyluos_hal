"""Device type annotation."""

from typing import Union

from .dynamixel import DynamixelMotor
from .fan import Fan
from .force_sensor import ForceSensor
from .joint import Joint
from .orbita import OrbitaActuator


Device = Union[Fan, Joint, DynamixelMotor, ForceSensor, OrbitaActuator]
