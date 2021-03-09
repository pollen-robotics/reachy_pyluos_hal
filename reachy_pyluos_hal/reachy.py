"""Reachy wrapper around serial LUOS GateClients which handle the communication with the hardware."""

import sys

from collections import OrderedDict, defaultdict
from glob import glob
from logging import Logger
from math import pi
from operator import attrgetter
from threading import Lock
from typing import Dict, List, Tuple

from .device import Device
from .discovery import find_gate
from .dynamixel import DynamixelMotor, MX106, MX64, MX28, AX18, XL320
from .fan import DxlFan, Fan, OrbitaFan
from .force_sensor import ForceSensor
from .joint import Joint
from .orbita import OrbitaActuator, OrbitaRegister
from .pycore import GateClient, GateProtocol


class Reachy(GateProtocol):
    """Reachy wrapper around serial GateClients which handle the communication with the hardware."""

    devices: List[Dict[str, Device]] = [
        OrderedDict([
            ('r_shoulder_fan', DxlFan(id=10)),
            ('r_shoulder_pitch', MX106(id=10, offset=pi/2, direct=False)),
            ('r_shoulder_roll', MX64(id=11, offset=pi/2, direct=False)),
            ('r_arm_yaw', MX64(id=12, offset=0.0, direct=False)),
            ('r_elbow_fan', DxlFan(id=13)),
            ('r_elbow_pitch', MX64(id=13, offset=0.0, direct=False)),
            ('r_forearm_yaw', AX18(id=14, offset=0.0, direct=False)),
            ('r_wrist_fan', DxlFan(id=15)),
            ('r_wrist_pitch', MX28(id=15, offset=0.0, direct=False)),
            ('r_wrist_roll', AX18(id=16, offset=0.0, direct=False)),
            ('r_gripper', AX18(id=17, offset=0.0, direct=True)),
            ('r_force_gripper', ForceSensor(id=10)),
        ]),
        OrderedDict([
            ('l_shoulder_fan', DxlFan(id=20)),
            ('l_shoulder_pitch', MX106(id=20, offset=pi/2, direct=True)),
            ('l_shoulder_roll', MX64(id=21, offset=-pi/2, direct=False)),
            ('l_arm_yaw', MX64(id=22, offset=0.0, direct=False)),
            ('l_elbow_fan', DxlFan(id=23)),
            ('l_elbow_pitch', MX64(id=23, offset=0.0, direct=False)),
            ('l_forearm_yaw', AX18(id=24, offset=0.0, direct=False)),
            ('l_wrist_fan', DxlFan(id=25)),
            ('l_wrist_pitch', MX28(id=25, offset=0.0, direct=False)),
            ('l_wrist_roll', AX18(id=26, offset=0.0, direct=False)),
            ('l_gripper', AX18(id=27, offset=0.0, direct=True)),
            ('l_force_gripper', ForceSensor(id=20)),
        ]),
        OrderedDict([
            ('neck', OrbitaActuator(id=40)),
            ('neck_fan', OrbitaFan(id=40, orbita='neck')),
            ('l_antenna', XL320(id=30, offset=0, direct=True)),
            ('r_antenna', XL320(id=31, offset=0, direct=True)),
            ('head_left_fan', DxlFan(id=30)),
            ('head_right_fan', DxlFan(id=31)),
        ]),
    ]
    if sys.platform == 'linux':
        port_template: str = '/dev/ttyUSB*'
    elif sys.platform == 'darwin':
        port_template: str = '/dev/tty.usbserial*'
    else:
        raise OSError('Unsupported platform')

    def __init__(self, logger: Logger) -> None:
        """Create all GateClient defined in the devices class variable."""
        self.logger = logger

        class GateProtocolDelegate(GateProtocol):
            lock = Lock()

            def handle_dxl_pub_data(_self, register, ids, errors, values):
                with _self.lock:
                    return self.handle_dxl_pub_data(register, ids, errors, values)

            def handle_load_pub_data(_self, ids: List[int], values: List[bytes]):
                with _self.lock:
                    return self.handle_load_pub_data(ids, values)

            def handle_orbita_pub_data(_self, id: int, register: OrbitaRegister, values: bytes):
                with _self.lock:
                    return self.handle_orbita_pub_data(id, register, values)

            def handle_fan_pub_data(_self, fan_ids: List[int], states: List[int]):
                with _self.lock:
                    return self.handle_fan_pub_data(fan_ids, states)

            def handle_assert(_self, msg: bytes):
                with _self.lock:
                    return self.handle_assert(msg)

        self.gates: List[GateClient] = []
        self.gate4name: Dict[str, GateClient] = {}
        self.dxls: Dict[str, Joint] = OrderedDict({})
        self.dxl4id: Dict[int, DynamixelMotor] = {}

        self.fans: Dict[str, Fan] = OrderedDict({})
        self.fan4id: Dict[int, Fan] = {}

        self.orbita4id: Dict[int, OrbitaActuator] = {}
        self.orbitas: Dict[str, OrbitaActuator] = OrderedDict({})

        self.force_sensors: Dict[str, ForceSensor] = OrderedDict({})
        self.force4id: Dict[int, ForceSensor] = {}

        self.ports = glob(self.port_template)
        if len(self.ports) == 0:
            raise IOError(f'No Gate found on "{self.port_template}"')

        for devices in self.devices:
            self.logger.info(f'Looking for {list(devices.keys())} on {self.ports}.')
            port, matching, missing = find_gate(devices, self.ports, self.logger)
            if len(missing) > 0:
                raise MissingContainerError(missing)

            self.logger.info(f'Found devices on="{port}", connecting...')

            gate = GateClient(port=port, protocol_factory=GateProtocolDelegate)
            self.gates.append(gate)

            for name, dev in devices.items():
                self.gate4name[name] = gate
                if isinstance(dev, DynamixelMotor):
                    self.dxls[name] = dev
                    if dev.id in self.dxl4id:
                        raise ValueError(f'All dynamixels id should be unique ({dev})!')
                    self.dxl4id[dev.id] = dev
                if isinstance(dev, ForceSensor):
                    self.force_sensors[name] = dev
                    if dev.id in self.force4id:
                        raise ValueError(f'All force sensors id should be unique ({dev})!')
                    self.force4id[dev.id] = dev
                if isinstance(dev, OrbitaActuator):
                    if dev.id in self.orbita4id:
                        raise ValueError(f'All orbitas id should be unique ({dev})!')
                    self.orbita4id[dev.id] = dev
                    self.orbitas[name] = dev
                if isinstance(dev, Fan):
                    self.fans[name] = dev
                    if dev.id in self.fan4id:
                        raise ValueError(f'All fans id should be unique ({dev})!')

                    self.fan4id[dev.id] = dev

                dev.logger = self.logger

    def __enter__(self):
        """Enter context handler."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Stop and close."""
        self.stop()

    def start(self):
        """Start all GateClients (start sending/receiving data with hardware)."""
        for gate in self.gates:
            gate.start()
            gate.protocol.logger = self.logger
        self.setup()

    def stop(self):
        """Stop all GateClients (start sending/receiving data with hardware)."""
        for gate in self.gates:
            gate.stop()

    def setup(self):
        """Set up everything before actually using (eg. offset for instance)."""
        for name, orbita in self.orbitas.items():
            zero = [int(x) for x in self.get_orbita_values('zero', name, clear_value=True, retry=3)]
            pos = [int(x) for x in self.get_orbita_values('absolute_position', name, clear_value=True, retry=3)]
            orbita.set_offset(zero, pos)

    def get_all_joints_names(self) -> List[str]:
        """Return the names of all joints."""
        dxl_names = list(self.dxls.keys())
        orbita_disk_names = []

        for name, orbita in self.orbitas.items():
            for disk_name in orbita.get_joints_name():
                orbita_disk_names.append(f'{name}_{disk_name}')

        return dxl_names + orbita_disk_names

    def get_joints_value(self, register: str, joint_names: List[str], retry: int = 3) -> List[float]:
        """Return the value of the specified joints."""
        # TODO: both get (dxl and orbita) should run in parallel (via asyncio?)
        clear_value = False if register in ('present_position', 'temperature') else True

        dxl_names = [name for name in joint_names if name in self.dxls]
        dxl_values = dict(zip(dxl_names, self.get_dxls_value(register, dxl_names, clear_value, retry)))

        orbitas_values = {}

        if register in ['moving_speed']:
            for name in joint_names:
                orbita_name = name.partition('_')[0]
                if orbita_name in self.orbitas:
                    orbita = self.orbitas[orbita_name]
                    for disk in orbita.get_joints_name():
                        orbitas_values[f'{orbita_name}_{disk}'] = 0.0
        else:
            for name in joint_names:
                orbita_name = name.partition('_')[0]
                if orbita_name in self.orbitas:
                    disk_values = self.get_orbita_values(register, orbita_name, clear_value, retry)
                    for disk, val in zip(self.orbitas[orbita_name].get_disks_name(), disk_values):
                        orbitas_values[f'{orbita_name}_{disk}'] = val
                    orbitas_values[f'{orbita_name}_roll'] = 0.0
                    orbitas_values[f'{orbita_name}_pitch'] = 0.0
                    orbitas_values[f'{orbita_name}_yaw'] = 0.0

        values = {}
        values.update(dxl_values)
        values.update(orbitas_values)
        return [values[joint] for joint in joint_names]

    def set_joints_value(self, register: str, value_for_joint: Dict[str, float]):
        """Set the value for the specified joints."""
        dxl_values: Dict[str, float] = {}
        orbita_values: Dict[str, Dict[str, float]] = {}

        for name, value in value_for_joint.items():
            orbita_name, _, disk_name = name.partition('_')

            if name in self.dxls:
                dxl_values[name] = value

            elif orbita_name in self.orbitas:
                if disk_name not in self.orbitas[orbita_name].get_disks_name():
                    continue

                if orbita_name not in orbita_values:
                    orbita_values[orbita_name] = {}

                orbita_values[orbita_name][disk_name] = value
            else:
                raise ValueError(f'"{name}" is an unknown joints!')

        if dxl_values:
            self.set_dxls_value(register, dxl_values)
        if orbita_values:
            if register == 'moving_speed':
                if self.logger is not None:
                    self.logger.warning('Speed for orbita not handled!!!')
                return
            for orbita, values in orbita_values.items():
                self.set_orbita_values(register, orbita, values)

    def get_dxls_value(self, register: str, dxl_names: List[str], clear_value: bool, retry: int) -> List[float]:
        """Retrieve register value on the specified dynamixels.

        The process is done as follows.
        First, clear any cached value for the register, we want to make sure we get an updated one.
        Then, split joints among their respective gate and send a single get request per gate (multiple ids per request).
        Finally, wait for all joints to received the updated value, converts it and returns it.
        """
        dxl_ids_per_gate: Dict[GateClient, List[int]] = defaultdict(list)
        dxl_reg_per_gate: Dict[GateClient, Tuple[int, int]] = {}

        for name in dxl_names:
            dxl = self.dxls[name]
            if clear_value:
                dxl.clear_value(register)

            if clear_value or (not dxl.is_value_set(register)):
                if isinstance(dxl, DynamixelMotor):
                    gate = self.gate4name[name]
                    dxl_ids_per_gate[gate].append(dxl.id)
                    dxl_reg_per_gate[gate] = dxl.get_register_config(register)

        for gate, ids in dxl_ids_per_gate.items():
            addr, num_bytes = dxl_reg_per_gate[gate]
            gate.protocol.send_dxl_get(addr, num_bytes, ids)

        try:
            return [
                self.dxls[name].get_value_as_usi(register)
                for name in dxl_names
            ]
        except TimeoutError as e:
            missing_dxls = [
                name for name in dxl_names
                if not self.dxls[name].is_value_set(register)
            ]
            if self.logger is not None:
                self.logger.warning(f'Timeout occurs after GET cmd: dev="{missing_dxls}" reg="{register}"!')
            if retry == 0:
                raise e
            return self.get_dxls_value(register, dxl_names, clear_value, retry - 1)

    def set_dxls_value(self, register: str, values_for_dxls: Dict[str, float]):
        """Set new value for register on the specified dynamixels.

        The values are splitted among the gates corresponding to the joints.
        One set request per gate is sent (with possible multiple ids).
        """
        dxl_data_per_gate: Dict[GateClient, Dict[int, bytes]] = defaultdict(dict)
        dxl_reg_per_gate: Dict[GateClient, Tuple[int, int]] = {}

        for name, dxl_value in values_for_dxls.items():
            dxl = self.dxls[name]

            if isinstance(dxl, DynamixelMotor):
                self.dxl4id[dxl.id].update_value_using_usi(register, dxl_value)

                if self._is_torque_enable(name) or register not in ['goal_position', 'moving_speed']:
                    gate = self.gate4name[name]
                    dxl_data_per_gate[gate][dxl.id] = self.dxl4id[dxl.id].get_value(register)
                    dxl_reg_per_gate[gate] = self.dxl4id[dxl.id].get_register_config(register)

        for gate, value_for_id in dxl_data_per_gate.items():
            addr, num_bytes = dxl_reg_per_gate[gate]
            gate.protocol.send_dxl_set(addr, num_bytes, value_for_id)

        if register == 'torque_enable':
            names = [name for name, value in values_for_dxls.items() if value == 1]
            cached_speed = dict(zip(names, self.get_dxls_value('moving_speed', names, clear_value=False, retry=3)))
            self.set_dxls_value('moving_speed', cached_speed)
            self.get_dxls_value('goal_position', names, clear_value=True, retry=3)

    def get_orbita_values(self, register_name: str, orbita_name: str, clear_value: bool, retry: int) -> List[float]:
        """Retrieve register value on the specified orbita actuator."""
        orbita = self.orbitas[orbita_name]
        register = OrbitaActuator.register_address[register_name]
        gate = self.gate4name[orbita_name]

        if clear_value:
            orbita.clear_value(register)

            gate.protocol.send_orbita_get(
                orbita_id=orbita.id,
                register=register.value,
            )

        try:
            return orbita.get_value_as_usi(register)
        except TimeoutError as e:
            if self.logger is not None:
                self.logger.warning(f'Timeout occurs after GET cmd: dev="{orbita_name}" reg="{register_name}"!')
            if retry == 0:
                raise e
            return self.get_orbita_values(register_name, orbita_name, clear_value, retry - 1)

    def set_orbita_values(self, register_name: str, orbita_name: str, value_for_disks: Dict[str, float]):
        """Set new value for register on the specified disks."""
        orbita = self.orbitas[orbita_name]
        register = OrbitaActuator.register_address[register_name]
        gate = self.gate4name[orbita_name]

        for disk_name, value in value_for_disks.items():
            attrgetter(f'{disk_name}.{register_name}')(orbita).update_using_usi(value)

        value_for_id = {
            orbita.get_id_for_disk(disk_name): attrgetter(f'{disk_name}.{register_name}')(orbita).get()
            for disk_name in value_for_disks.keys()
        }
        gate.protocol.send_orbita_set(orbita.id, register.value, value_for_id)

    def get_fans_state(self, fan_names: List[str], retry=3) -> List[float]:
        """Retrieve state for the specified fans."""
        dxl_fans_per_gate: Dict[GateClient, List[int]] = defaultdict(list)
        dxl_fans: List[str] = []
        orbita_fans: List[Tuple[str, str]] = []

        for name in fan_names:
            fan = self.fans[name]

            if isinstance(fan, DxlFan):
                fan.state.reset()
                dxl_fans_per_gate[self.gate4name[name]].append(fan.id)
                dxl_fans.append(name)
            elif isinstance(fan, OrbitaFan):
                orbita_fans.append((name, fan.orbita))

        for gate, ids in dxl_fans_per_gate.items():
            gate.protocol.send_dxl_fan_get(ids)

        try:
            fans_state = {}
            for name in dxl_fans:
                fans_state[name] = self.fans[name].state.get_as_usi()

            for fan_name, orbita_name in orbita_fans:
                fans_state[fan_name] = self.get_orbita_values('fan_state', orbita_name, clear_value=True, retry=retry)[0]

            return [fans_state[name] for name in fan_names]

        except TimeoutError:
            if retry > 0:
                return self.get_fans_state(fan_names, retry - 1)
            raise

    def set_fans_state(self, state_for_fan: Dict[str, float]):
        """Set state for the specified fans."""
        fans_per_gate: Dict[GateClient, Dict[int, float]] = defaultdict(dict)

        for name, state in state_for_fan.items():
            fan = self.fans[name]
            if isinstance(fan, DxlFan):
                fan.state.update_using_usi(state)
                fans_per_gate[self.gate4name[name]][fan.id] = state

        for gate, values in fans_per_gate.items():
            gate.protocol.send_dxl_fan_set(values)

    def _is_torque_enable(self, name: str) -> bool:
        return self.get_dxls_value('torque_enable', [name], clear_value=False, retry=3)[0] == 1

    def handle_dxl_pub_data(self, addr: int, ids: List[int], errors: List[int], values: List[bytes]):
        """Handle dxl update received on a gate client."""
        for id, err, val in zip(ids, errors, values):
            if (err != 0) and self.logger is not None:
                self.logger.warning(f'Dynamixel error {err} on motor id={id}!')

            m = self.dxl4id[id]
            m.update_value(m.find_register_by_addr(addr), val)

    def handle_load_pub_data(self, ids: List[int], values: List[bytes]):
        """Handle load update received on a gate client."""
        for id, val in zip(ids, values):
            self.force4id[id].update_force(val)

    def handle_orbita_pub_data(self, orbita_id: int, reg_type: OrbitaRegister, values: bytes):
        """Handle orbita update received on a gate client."""
        self.orbita4id[orbita_id].update_value(reg_type, values)

    def handle_fan_pub_data(self, fan_ids: List[int], states: List[int]):
        """Handle fan state update received on a gate client."""
        for id, state in zip(fan_ids, states):
            self.fan4id[id].state.update_using_usi(state)

    def handle_assert(self, msg: bytes):
        """Handle an assertion received on a gate client."""
        raise AssertionError(msg)


class MissingContainerError(Exception):
    """Custom exception for missing container."""

    def __init__(self, missing: List[Device]):
        """Set up the missing container execption."""
        super().__init__(f'Could not find given devices {missing}!')
