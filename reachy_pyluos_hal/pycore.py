from typing import Iterable, Optional

import time
import struct

from logging import Logger
from serial.threaded import Protocol, ReaderThread


class GateProtocol(Protocol):
    MSG_TYPE_DXL_GET_REG = 10
    MSG_TYPE_DXL_SET_REG = 11
    MSG_TYPE_PUB_DATA = 15
    MSG_TYPE_KEEP_ALIVE = 200
    MSG_MODULE_ASSERT = 222

    logger: Optional[Logger] = None
    header = bytes([255, 255])

    def __init__(self) -> None:
        self.transport: Optional[ReaderThread] = None
        self.buffer = bytearray()

    def connection_made(self, transport):
        print('Connection made', transport)
        self.transport = transport

    def connection_lost(self, exc):
        if isinstance(exc, Exception):
            raise exc

    def data_received(self, data):
        self.buffer.extend(data)

        if len(self.buffer) > len(self.header) + 1:
            for msg in self.pop_messages():
                self.handle_message(msg)

    def send_msg(self, payload: bytes):
        assert (self.transport is not None)
        # TODO: this should be handle by the gate
        if not hasattr(self, 'last_send'):
            self.last_send = 0.0

        if time.time() - self.last_send < 0.001:
            time.sleep(0.001)

        data = self.header + bytes([len(payload)]) + payload
        self.transport.write(data)
        self.last_send = time.time()

    def send_keep_alive(self):
        self.send_msg(bytes([self.MSG_TYPE_KEEP_ALIVE]))

    def send_dxl_get(self, register, num_bytes, ids):
        self.send_msg(bytes([self.MSG_TYPE_DXL_GET_REG, register, num_bytes] + ids))

    def send_dxl_set(self, register, num_bytes, value_for_id):
        msg = [self.MSG_TYPE_DXL_SET_REG, register, num_bytes]
        for id, val in value_for_id.items():
            msg += [id] + val
        self.send_msg(bytes(msg))

    def pop_messages(self) -> Iterable[bytearray]:
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

    def check_msg(self, msg):
        s = len(msg)
        return s > 1 and msg[0] + 1 == s

    def handle_message(self, msg: bytes):
        assert msg[0] == len(msg) - 1
        payload = msg[1:]

        if payload[0] == self.MSG_MODULE_ASSERT:
            self.handle_assert(payload[1:].decode())

        if payload[0] == self.MSG_TYPE_PUB_DATA:
            register = payload[1]
            val_size = payload[2]
            size_per_id = 1 + 2 + val_size

            nb_ids = (len(payload) - 3) // size_per_id
            ids, errors, values = [], [], []

            for i in range(nb_ids):
                data_for_id = payload[3 + i * size_per_id: 3 + (i + 1) * size_per_id]

                ids.append(data_for_id[0])
                errors.append(struct.unpack('H', data_for_id[1:3])[0])
                values.append(struct.unpack('B' * val_size, data_for_id[3:]))

            self.handle_dxl_pub_data(register, ids, errors, values)

    def handle_dxl_pub_data(self, register, ids, errors, values):
        raise NotImplementedError

    def handle_assert(self, msg: str):
        raise NotImplementedError
