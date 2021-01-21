"""Discover utility functions to find the correct serial port where the given devices are connected."""

from typing import Dict, List, Tuple

from serial import Serial
from serial.threaded import ReaderThread

from .device import Device
from .dynamixel import DynamixelMotor
from .force_sensor import ForceSensor
from .pycore import GateProtocol, LuosContainer


def find_gate(devices: Dict[str, Device], ports: List[str]) -> Tuple[str, List[Device], List[Device]]:
    """Try to identify the correct gate among possible serial ports based on the identified luos device."""
    solutions = {}

    for port in ports:
        containers: List[LuosContainer] = sum(identify_luos_containers(port).values(), [])

        matching, missing = corresponding_containers(devices, containers)
        solutions[port] = (matching, missing)
        if len(missing) == 0:
            return (port, matching, missing)

    best_solution = sorted(solutions.items(), key=lambda item: len(item[1][1]))[0]
    port, (matching, missing) = best_solution
    return (port, matching, missing)


def identify_luos_containers(port: str) -> Dict[int, List[LuosContainer]]:
    """Found which luos containers are connected to the serial port."""
    class GateHandler(GateProtocol):
        def handle_assert(self, msg):
            raise AssertionError(msg)

    with Serial(port, baudrate=1000000) as s:
        with ReaderThread(s, GateHandler) as p:
            return p.send_detection_signal()


def corresponding_containers(
        devices: Dict[str, Device],
        luos_containers: List[LuosContainer],
        ) -> Tuple[List[Device], List[Device]]:
    """Find the matching and missing containers."""
    matching: List[Device] = []
    missing: List[Device] = []

    for dev in devices.values():
        if isinstance(dev, DynamixelMotor):
            container_type = 'DynamixelMotor'
            basename = 'dxl'
        elif isinstance(dev, ForceSensor):
            container_type = 'Load'
            basename = 'load'
        else:
            missing.append(dev)
            continue

        if find_container(dev.id, container_type, basename, luos_containers):
            matching.append(dev)
        else:
            missing.append(dev)

    return matching, missing


def find_container(container_id: int, container_type: str, basename: str, luos_containers: List[LuosContainer]) -> bool:
    """Find a specific container give its type, id and alias."""
    for c in luos_containers:
        if c.type == container_type and c.alias == f'{basename}_{container_id}':
            return True
    return False
