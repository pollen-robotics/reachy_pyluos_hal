"""Serial communication protocol with Reachy Luos Gate."""

import sys
import time
import struct

from logging import Logger
from threading import Event, Thread
from typing import Dict, Iterable, List, Optional, Type

from serial import Serial
from serial.threaded import Protocol, ReaderThread


class GateProtocol(Protocol):
    """Serial communication protocol with Reachy Luos Gate."""

    MSG_TYPE_DXL_GET_REG = 10
    MSG_TYPE_DXL_SET_REG = 11
    MSG_TYPE_PUB_DATA = 15
    MSG_TYPE_LOAD_PUB_DATA = 20
    MSG_TYPE_KEEP_ALIVE = 200
    MSG_MODULE_ASSERT = 222

    logger: Optional[Logger] = None
    header = bytes([255, 255])

    def __init__(self) -> None:
        """Prepare the input buffer."""
        self.transport: Optional[ReaderThread] = None
        self.buffer = bytearray()

    def connection_made(self, transport: ReaderThread):
        """Handle connection made."""
        if self.logger is not None:
            self.logger.info(f'Connection made with {transport}')
        self.transport = transport

    def connection_lost(self, exc: Optional[Exception]):
        """Handle connection lost."""
        if isinstance(exc, Exception):
            raise exc

    def data_received(self, data: bytearray):
        """Handle new received data."""
        self.buffer.extend(data)

        if len(self.buffer) > len(self.header) + 1:
            for msg in self.pop_messages():
                self.handle_message(msg)

    def send_msg(self, payload: bytes):
        """Send message with specified payload."""
        assert (self.transport is not None)
        # TODO: this should be handle by the gate
        if not hasattr(self, 'last_send'):
            self.last_send = 0.0

        if time.time() - self.last_send < 0.001:
            time.sleep(0.001)

        data = self.header + bytes([len(payload)]) + payload
        if self.logger is not None:
            self.logger.debug(f'Sending {list(data)}')
        self.transport.write(data)
        self.last_send = time.time()

    def send_keep_alive(self):
        """Send keep alive message [MSG_TYPE_KEEP_ALIVE]."""
        self.send_msg(bytes([self.MSG_TYPE_KEEP_ALIVE]))

    def send_dxl_get(self, register: int, num_bytes: int, ids: List[int]):
        """Send a dxl get message [MSG_TYPE_DXL_GET_REG, REG, NUM_BYTES, (ID)+]."""
        self.send_msg(bytes([self.MSG_TYPE_DXL_GET_REG, register, num_bytes] + ids))

    def send_dxl_set(self, register: int, num_bytes: int, value_for_id: Dict[int, bytes]):
        """Send a dxl set message [MSG_TYPE_DXL_SET_REG, REG, NUM_BYTES, (ID, (VAL)+)+]."""
        msg = [self.MSG_TYPE_DXL_SET_REG, register, num_bytes]
        for id, val in value_for_id.items():
            msg += [id] + list(val)
        self.send_msg(bytes(msg))

    def pop_messages(self) -> Iterable[bytearray]:
        """Parse buffer and check for complete messages."""
        msgs = self.buffer.split(self.header)

        if len(msgs) == 1:
            return []

        msgs = msgs[1:]
        last_msg = msgs[-1]

        if self.check_msg(last_msg):
            self.buffer.clear()
            return msgs

        self.buffer = self.buffer[-(len(last_msg) + len(self.header) + 1):]
        return msgs[:-1]

    def check_msg(self, msg: bytes) -> bool:
        """Check if the msg is complete."""
        s = len(msg)
        return s > 1 and msg[0] + 1 == s

    def handle_message(self, msg: bytes):
        """Handle the reception of a complete message."""
        if self.logger is not None:
            self.logger.info(f'Got msg {list(msg)}')
        assert msg[0] == len(msg) - 1
        payload = msg[1:]

        if payload[0] == self.MSG_MODULE_ASSERT:
            self.handle_assert(payload[1:].decode())

        elif payload[0] == self.MSG_TYPE_PUB_DATA:
            register = payload[1]
            val_size = payload[2]
            size_per_id = 1 + 2 + val_size

            nb_ids = (len(payload) - 3) // size_per_id
            ids, errors, values = [], [], []

            for i in range(nb_ids):
                data_for_id = payload[3 + i * size_per_id: 3 + (i + 1) * size_per_id]

                ids.append(data_for_id[0])
                errors.append(struct.unpack('H', data_for_id[1:3])[0])
                values.append(data_for_id[3:])

            self.handle_dxl_pub_data(register, ids, errors, values)

        elif payload[0] == self.MSG_TYPE_LOAD_PUB_DATA:
            nb_sensors = (len(payload) - 1) // 5
            ids, loads = [], []
            for i in range(nb_sensors):
                ids.append(payload[5 * i + 1])
                loads.append(struct.unpack('<f', payload[5 * i + 2: 5 * i + 6])[0])
            self.handle_load_pub_data(ids, loads)
        
        else:
            if self.logger is not None:
                self.logger.warning(f'Got unrecognized message {list(payload)}')

    def handle_dxl_pub_data(self, register: int, ids: List[int], errors: List[int], values: List[bytes]):
        """Handle dxl update received on a gate client."""
        raise NotImplementedError

    def handle_load_pub_data(self, ids: List[int], values: List[float]):
        """Handle load update received on a gate client."""
        raise NotImplementedError

    def handle_assert(self, msg: str):
        """Handle an assertion received on a gate client."""
        raise NotImplementedError


class GateClient:
    """Gate client running a serial ReaderThread."""

    def __init__(self, port: str, protocol_factory: Type[GateProtocol]) -> None:
        """Set up the serial communication."""
        self.serial = Serial(port=port, baudrate=1000000)
        if sys.platform == 'linux':
            self.serial.set_low_latency_mode(True)

        self.protocol_factory = protocol_factory
        self.alive = Event()

    def start(self):
        """Start the ReaderThread loop and for it to really start."""
        self.t = Thread(target=self.run)
        self.t.start()
        self.alive.wait()

    def run(self):
        """Run the ReaderThread loop."""
        with ReaderThread(self.serial, self.protocol_factory) as protocol:
            self.protocol = protocol
            self.alive.set()

            while self.alive.is_set():
                protocol.send_keep_alive()
                time.sleep(1)

    def stop(self):
        """Stop the ReaderThread loop and wait for it to finish."""
        self.alive.clear()
        self.t.join()
