"""Discover utility functions to find the correct serial port where the given devices are connected."""

import time

from logging import Logger
from typing import Dict, List, Optional, Tuple

from serial import Serial
from serial.threaded import ReaderThread

from .device import Device
from .dynamixel import DynamixelMotor
from .fan import Fan
from .force_sensor import ForceSensor
from .orbita import OrbitaActuator
from .pycore import GateProtocol, LuosContainer


def find_gate(devices: Dict[str, Device], ports: List[str], logger: Optional[Logger], retry: int = 10) -> Tuple[str, List[Device], List[Device]]:
    """Try to identify the correct gate among possible serial ports based on the identified luos device."""
    solutions = {}

    for port in ports:
        containers: List[LuosContainer] = sum(identify_luos_containers(port, logger).values(), [])

        matching, missing = corresponding_containers(devices, containers)
        solutions[port] = (matching, missing)
        if len(missing) == 0:
            return (port, matching, missing)

    if retry > 0:
        return find_gate(devices, ports, logger, retry - 1)

    best_solution = sorted(solutions.items(), key=lambda item: len(item[1][1]))[0]
    port, (matching, missing) = best_solution
    return (port, matching, missing)


def identify_luos_containers(port: str, logger: Optional[Logger] = None) -> Dict[int, List[LuosContainer]]:
    """Found which luos containers are connected to the serial port."""
    class GateHandler(GateProtocol):
        def handle_assert(self, msg):
            raise AssertionError(msg)

    GateHandler.logger = logger
    with Serial(port, baudrate=1000000) as s:
        with ReaderThread(s, GateHandler) as p:
            p.send_detection_run_signal()
            containers = p.send_detection_signal()
            time.sleep(p.timeout)
            return containers


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
        elif isinstance(dev, OrbitaActuator):
            container_type = 'ControllerMotor'
            basename = 'orbita'
        elif isinstance(dev, Fan):
            continue
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


if __name__ == '__main__':
    import sys
    from glob import glob

    if sys.platform == 'linux':
        port_template = '/dev/gate*'
    elif sys.platform == 'darwin':
        port_template = '/dev/tty.usbserial*'
    else:
        raise SystemError

    for port in glob(port_template):
        print(port)
        print(identify_luos_containers(port))
        print()
