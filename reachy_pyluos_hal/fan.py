"""Fan device abstraction."""
from logging import Logger
from typing import Optional

from .register import Register


class Fan:
    """Fan device abstraction."""

    def __init__(self, id: int) -> None:
        """Set up new Fan."""
        self.id = id
        self._state = Register(self.cvt_as_usi, self.cvt_as_raw)

        self.logger: Optional[Logger] = None

    @property
    def state(self) -> Register:
        """Get state register."""
        return self._state

    def cvt_as_usi(self, val: bytes) -> float:
        """Convert as USI (0 or 1)."""
        return val[0]

    def cvt_as_raw(self, val: float) -> bytes:
        """Convert from USI (0 or 1)."""
        return bytes([int(val)])
