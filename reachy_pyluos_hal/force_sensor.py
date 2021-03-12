"""Force Sensor abstraction."""

import struct

from logging import Logger
from typing import Optional

from .register import Register


class ForceSensor:
    """Force sensor abstraction."""

    def __init__(self, id: int) -> None:
        """Wrap a force Register."""
        self.id = id
        self.force = Register(self.cvt_as_usi, self.cvt_as_raw, timeout=1.0)
        self.logger: Optional[Logger] = None

    def __repr__(self) -> str:
        """Represent force sensor."""
        return f'<ForceSensor id={self.id}>'

    def cvt_as_usi(self, val: bytes) -> float:
        """Convert raw value as force (not really USI in this case)."""
        return struct.unpack('<f', val)[0]

    def cvt_as_raw(self, val: float) -> bytes:
        """Convert value as bytes."""
        return struct.pack('<f', val)

    def update_force(self, force: bytes):
        """Update force received from a gate update."""
        self.force.update(force)

    def get_force(self, retry=3) -> float:
        """Retrieve the last updated force."""
        for _ in range(retry - 1):
            try:
                return self.force.get_as_usi()
            except TimeoutError:
                pass
        return self.force.get_as_usi()
