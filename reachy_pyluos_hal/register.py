"""Synced register class."""

import time

from threading import Event
from typing import Callable, Union


class Register:
    """Synced register object."""

    def __init__(self,
                 cvt_as_usi: Callable[[bytes], float],
                 cvt_as_raw: Callable[[float], bytes],
                 timeout: float = 0.015,
                 ) -> None:
        """Set up the register with a None value by default."""
        self.val: Union[bytes, None] = None
        self.timestamp = 0.0
        self.synced = Event()
        self.timeout = timeout

        self.cvt_as_usi = cvt_as_usi
        self.cvt_as_raw = cvt_as_raw

    def is_set(self) -> bool:
        """Check if the register has been set since last reset."""
        return self.synced.is_set()

    def update(self, val: bytes):
        """Update the register with a raw value retrieve from its associated gate."""
        self.val = val
        self.timestamp = time.time()
        self.synced.set()

    def update_using_usi(self, val: float):
        """Update the register with a USI value retrieve from its associated gate."""
        self.update(self.cvt_as_raw(val))

    def get(self) -> bytes:
        """Wait for an updated value and returns it."""
        if not self.synced.wait(self.timeout):
            raise TimeoutError
        assert self.val is not None
        return self.val

    def get_as_usi(self) -> float:
        """Wait for an updated value and returns it converted as USI units."""
        return self.cvt_as_usi(self.get())

    def reset(self):
        """Mark the value as obsolete."""
        self.synced.clear()
        self.val = None
