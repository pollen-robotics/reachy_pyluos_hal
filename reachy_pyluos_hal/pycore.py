"""Serial communication protocol with Reachy Luos Gate."""

from reachy_pyluos_hal.orbita import OrbitaRegister
import sys
import time
import struct

from logging import Logger
from collections import defaultdict, namedtuple
from threading import Event, Thread
from typing import Dict, Iterable, List, Optional, Type, Tuple

from serial import Serial
from serial.threaded import Protocol, ReaderThread


LuosContainer = namedtuple('LuosContainer', ('id', 'alias', 'type'))


class GateProtocol(Protocol):
    """Serial communication protocol with Reachy Luos Gate."""

    MSG_TYPE_DXL_GET_REG = 10
    MSG_TYPE_DXL_SET_REG = 11
    MSG_TYPE_DXL_PUB_DATA = 15
    MSG_TYPE_LOAD_PUB_DATA = 20
    MSG_TYPE_FAN_GET_STATE = 30
    MSG_TYPE_FAN_SET_STATE = 31
    MSG_TYPE_FAN_PUB_DATA = 35
    MSG_TYPE_ORBITA_GET_REG = 50
    MSG_TYPE_ORBITA_SET_REG = 51
    MSG_TYPE_ORBITA_PUB_DATA = 55
    MSG_TYPE_KEEP_ALIVE = 200
    MSG_DETECTION_GET_NODES = 210
    MSG_DETECTION_GET_CONTAINERS = 211
    MSG_DETECTION_GET_CONTAINER_INFO = 212
    MSG_DETECTION_PUB_NODES = 215
    MSG_DETECTION_PUB_CONTAINERS = 216
    MSG_DETECTION_PUB_CONTAINER_INFO = 217
    MSG_DETECTION_RUN = 218
    MSG_MODULE_ASSERT = 222

    logger: Optional[Logger] = None
    header = bytes([255, 255])

    def __init__(self, timeout: float = 0.5) -> None:
        """Prepare the input buffer."""
        self.transport: Optional[ReaderThread] = None
        self.buffer = bytearray()
        self.timeout = timeout

        self._nodes: Dict[int, List[int]] = {}
        self._containers: Dict[int, Tuple[str, str]] = {}

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

        for msg in self.pop_messages():
            self.handle_message(msg)

    def send_msg(self, payload: bytes):
        """Send message with specified payload."""
        assert (self.transport is not None)

        data = self.header + bytes([len(payload)]) + payload
        if self.logger is not None:
            self.logger.debug(f'Sending {list(data)}')
        self.transport.write(data)

    def send_detection_run_signal(self) -> None:
        """Send request to run a Luos detection from the gate."""
        self.send_msg(bytes([self.MSG_DETECTION_RUN]))

    def send_detection_signal(self) -> Dict[int, List[LuosContainer]]:
        """Send request to the gate to retrieve all nodes/containers info."""
        self._waiting_for_nodes = Event()
        self.send_msg(bytes([self.MSG_DETECTION_GET_NODES]))

        if not self._waiting_for_nodes.wait(self.timeout):
            raise TimeoutError
        for id, evt in self._waiting_for_containers.items():
            if not evt.wait(self.timeout):
                raise TimeoutError

        devices = defaultdict(list)

        for id, (alias, type) in self._containers.items():
            for node_id, containers in self._nodes.items():
                if id in containers:
                    devices[node_id].append(LuosContainer(id, alias, type))
                    break

        return dict(devices)

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

    def send_orbita_get(self, orbita_id: int, register: int):
        """Send an orbita get message [MSG_TYPE_ORBITA_GET_REG, ORBITA_ID, REG_TYPE]."""
        self.send_msg(bytes([self.MSG_TYPE_ORBITA_GET_REG, orbita_id, register]))

    def send_orbita_set(self, orbita_id: int, register: int, value_for_id: Dict[int, bytes]):
        """Send an orbita set message [MSG_TYPE_ORBITA_SET_REG, ORBITA_ID, REG_TYPE, (MOTOR_ID, (VAL+))+]."""
        msg = [self.MSG_TYPE_ORBITA_SET_REG, orbita_id, register]
        for motor_id, value in value_for_id.items():
            msg += [motor_id] + list(value)
        self.send_msg(bytes(msg))

    def send_fan_get(self, fans: List[int]):
        """Send a fan get message [MSG_TYPE_FAN_GET_STATE, (FAN_ID)+]."""
        self.send_msg(bytes([self.MSG_TYPE_FAN_GET_STATE] + fans))

    def send_fan_set(self, state_for_fan: Dict[int, int]):
        """Send a fan set message [MSG_TYPE_FAN_SET_STATE, (FAN_ID, STATE)+]."""
        msg = [self.MSG_TYPE_FAN_SET_STATE]
        for fan_id, fan_state in state_for_fan.items():
            msg += [fan_id, fan_state]
        self.send_msg(bytes(msg))

    def pop_messages(self) -> Iterable[bytearray]:
        """Parse buffer and check for complete messages."""
        msgs = []

        while True:
            if len(self.buffer) < 3:
                break
            assert self.buffer[0] == self.buffer[1] == 255

            payload_size = self.buffer[2]
            if len(self.buffer) < 3 + payload_size:
                break

            msg = self.buffer[3: 3 + payload_size]
            msgs.append(msg)

            self.buffer = self.buffer[3 + payload_size:]

        return msgs

    def check_msg(self, msg: bytes) -> bool:
        """Check if the msg is complete."""
        s = len(msg)
        return s > 1 and msg[0] + 1 == s

    def handle_message(self, payload: bytes):
        """Handle the reception of a complete message."""
        if self.logger is not None:
            self.logger.debug(f'Got msg {list(payload)}')

        if payload[0] == self.MSG_MODULE_ASSERT:
            self.handle_assert(payload[1:])

        elif payload[0] == self.MSG_TYPE_DXL_PUB_DATA:
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
                loads.append(payload[5 * i + 2: 5 * i + 6])
            self.handle_load_pub_data(ids, loads)

        elif payload[0] == self.MSG_TYPE_ORBITA_PUB_DATA:
            orbita_id = payload[1]
            reg_type = payload[2]
            self.handle_orbita_pub_data(orbita_id, OrbitaRegister(reg_type), payload[3:])

        elif payload[0] == self.MSG_TYPE_FAN_PUB_DATA:
            data = list(payload[1:])
            ids = data[::2]
            states = data[1::2]
            self.handle_fan_pub_data(ids, states)

        elif payload[0] == self.MSG_DETECTION_PUB_NODES:
            self._nodes.clear()
            self._containers.clear()

            self._numbers_of_nodes_waiting = len(payload) - 1
            self._waiting_for_containers = {}

            if self._numbers_of_nodes_waiting == 0:
                self._waiting_for_nodes.set()

            for node_id in payload[1:]:
                self._nodes[node_id] = []
                self.send_msg(payload=bytes([self.MSG_DETECTION_GET_CONTAINERS, node_id]))

        elif payload[0] == self.MSG_DETECTION_PUB_CONTAINERS:
            node_id = payload[1]
            for container_id in payload[2:]:
                self._nodes[node_id].append(container_id)
                self.send_msg(bytes([self.MSG_DETECTION_GET_CONTAINER_INFO, container_id]))
                self._waiting_for_containers[container_id] = Event()

            self._numbers_of_nodes_waiting -= 1
            if self._numbers_of_nodes_waiting == 0:
                self._waiting_for_nodes.set()

        elif payload[0] == self.MSG_DETECTION_PUB_CONTAINER_INFO:
            container_id = payload[1]
            alias, type = payload[2:].decode().split(' ')
            self._containers[container_id] = (alias, type)
            self._waiting_for_containers[container_id].set()

        else:
            if self.logger is not None:
                self.logger.warning(f'Got unrecognized message {list(payload)}')

    def handle_dxl_pub_data(self, register: int, ids: List[int], errors: List[int], values: List[bytes]):
        """Handle dxl update received on a gate client."""
        raise NotImplementedError

    def handle_load_pub_data(self, ids: List[int], values: List[bytes]):
        """Handle load update received on a gate client."""
        raise NotImplementedError

    def handle_orbita_pub_data(self, orbita_id: int, reg_type: OrbitaRegister, values: bytes):
        """Handle orbita update received on a gate client."""
        raise NotImplementedError

    def handle_fan_pub_data(self, fan_ids: List[int], states: List[int]):
        """Handle fan state update received on a gate client."""
        raise NotImplementedError

    def handle_assert(self, msg: bytes):
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
        # Make sure all messages buffered by the gate were received.
        if hasattr(self, 't') and self.t.is_alive():
            self.t.join()
