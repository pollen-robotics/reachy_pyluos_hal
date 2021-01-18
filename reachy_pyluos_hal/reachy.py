"""Reachy wrapper around serial LUOS GateClients which handle the communication with the hardware."""

from typing import Dict, List
from logging import Logger

from threading import Lock
from collections import OrderedDict, defaultdict

from .joint import Joint
from .pycore import GateClient, GateProtocol
from .dynamixel import DynamixelMotor, MX106, MX64, AX18


class Reachy(GateProtocol):
    """Reachy wrapper around serial LUOS GateClients which handle the communication with the hardware."""

    devices: Dict[str, Dict[str, Joint]] = OrderedDict([
        ('/dev/tty.usbserial-DN05NM0W', OrderedDict([
            ('r_shoulder_pitch', MX106(id=10, offset=90.0, direct=False)),
            ('r_shoulder_roll', MX64(id=11, offset=90.0, direct=False)),
            ('r_arm_yaw', MX64(id=12, offset=0.0, direct=False)),
            ('r_elbow_pitch', MX64(id=13, offset=0.0, direct=False)),
        ])),
        ('/dev/tty.usbserial-D307RR2E', OrderedDict([
            ('r_forearm_yaw', AX18(id=14, offset=0.0, direct=False)),
            ('r_wrist_pitch', AX18(id=15, offset=0.0, direct=False)),
            ('r_wrist_roll', AX18(id=16, offset=0.0, direct=False)),
            ('r_gripper', AX18(id=17, offset=0.0, direct=True)),
        ])),
    ])

    def __init__(self, logger: Logger) -> None:
        """Create all GateClient defined in the devices class variable."""
        self.logger = logger

        class GateProtocolDelegate(GateProtocol):
            lock = Lock()

            def handle_dxl_pub_data(_self, register, ids, errors, values):
                with _self.lock:
                    return self.handle_dxl_pub_data(register, ids, errors, values)

            def handle_assert(_self, msg: str):
                with _self.lock:
                    return self.handle_assert(msg)

        self.gates: List[GateClient] = []
        self.gate4name: Dict[str, GateClient] = {}
        self.joints: Dict[str, Joint] = {}
        self.dxl4id: Dict[int, DynamixelMotor] = {}

        for port, devices in self.devices.items():
            self.logger.info(f'Create GateClient on="{port}" with {list(devices.keys())} attached.')
            gate = GateClient(port=port, protocol_factory=GateProtocolDelegate)
            self.gates.append(gate)

            for name, dev in devices.items():
                self.gate4name[name] = gate
                self.joints[name] = dev

                if isinstance(dev, DynamixelMotor):
                    self.dxl4id[dev.id] = dev

    def start(self):
        """Start all GateClients (start sending/receiving data with hardware)."""
        for gate in self.gates:
            gate.start()

    def stop(self):
        """Stop all GateClients (start sending/receiving data with hardware)."""
        for gate in self.gates:
            gate.stop()

    def get_joints_value(self, register: str, joint_names: List[str], clear_value: bool) -> List[float]:
        """Retrieve register value on the specified joints.

        The process is done as follows.
        First, clear any cached value for the register, we want to make sure we get an updated one.
        Then, split joints among their respective gate and send a single get request per gate (multiple ids per request).
        Finally, wait for all joints to received the updated value, converts it and returns it.
        """
        dxl_ids_per_gate: Dict[GateClient, List[int]] = defaultdict(list)
        for name in joint_names:
            joint = self.joints[name]
            if clear_value:
                joint.clear_value(register)

            if clear_value or (not joint.is_value_set(register)):
                if isinstance(joint, DynamixelMotor):
                    gate = self.gate4name[name]
                    dxl_ids_per_gate[gate].append(joint.id)

        addr, num_bytes = DynamixelMotor.register_config[register]
        for gate, ids in dxl_ids_per_gate.items():
            gate.protocol.send_dxl_get(addr, num_bytes, ids)

        return [
            self.joints[name].convert_to_usi(
                register,
                self.joints[name].get_value(register),
            )
            for name in joint_names
        ]

    def set_joints_value(self, register: str, values_for_joints: Dict[str, float]):
        """Set new value for register on the specified joints.

        The values are splitted among the gates corresponding to the joints.
        One set request per gate is sent (with possible multiple ids).
        """
        dxl_data_per_gate: Dict[GateClient, Dict[int, bytes]] = defaultdict(dict)
        for name, value in values_for_joints.items():
            joint = self.joints[name]

            if isinstance(joint, DynamixelMotor):
                dxl_raw_value = joint.convert_to_raw(register, value)

                if self._is_torque_enable(name) or register not in ['goal_position', 'moving_speed']:
                    gate = self.gate4name[name]
                    dxl_data_per_gate[gate][joint.id] = dxl_raw_value

                self.dxl4id[joint.id].update_value(register, dxl_raw_value)

        addr, num_bytes = DynamixelMotor.register_config[register]
        for gate, value_for_id in dxl_data_per_gate.items():
            gate.protocol.send_dxl_set(addr, num_bytes, value_for_id)

        if register == 'torque_enable':
            names = list(values_for_joints.keys())
            cached_speed = dict(zip(names, self.get_joints_value('moving_speed', names, clear_value=False)))
            self.set_joints_value('moving_speed', cached_speed)
            self.get_joints_value('goal_position', names, clear_value=True)

    def _is_torque_enable(self, name: str) -> bool:
        return self.get_joints_value('torque_enable', [name], clear_value=False)[0] == 1

    def handle_dxl_pub_data(self, addr: int, ids: List[int], errors: List[int], values: List[bytes]):
        """Handle dxl update received on a gate client."""
        for id, err, val in zip(ids, errors, values):
            if (err != 0) and self.logger is not None:
                self.logger.warning(f'Dynamixel error {err} on motor id={id}!')
            self.dxl4id[id].update_value(DynamixelMotor.find_register(addr), val)

    def handle_assert(self, msg: str):
        """Handle an assertion received on a gate client."""
        raise AssertionError(msg)
