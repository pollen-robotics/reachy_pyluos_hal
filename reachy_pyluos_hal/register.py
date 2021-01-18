"""Synced register class."""

import time

from typing import Optional
from threading import Event


class Register:
    """Synced register object."""

    def __init__(self) -> None:
        """Set up the register with a None value by default."""
        self.val = None
        self.timestamp = 0.0
        self.synced = Event()

    def is_set(self) -> bool:
        """Check if the register has been set since last reset."""
        return self.synced.is_set()

    def update(self, val):
        """Update the register with a value retrieve from its associated gate."""
        self.val = val
        self.timestamp = time.time()
        self.synced.set()

    def get(self):
        """Wait for an updated value and returns it."""
        self.synced.wait()
        assert self.val is not None
        return self.val

    def reset(self):
        """Mark the value as obsolete."""
        self.synced.clear()
        self.val = None
