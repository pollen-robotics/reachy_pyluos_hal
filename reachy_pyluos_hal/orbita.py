from enum import Enum


from .register import Register


class OrbitaRegister(Enum):
    ANGLE_LIMIT = 0
    TEMPERATURE_LIMIT = 1

    PRESENT_POSITION = 10
    PRESENT_SPEED = 11
    PRESENT_LOAD = 12

    GOAL_POSITION = 20
    MAX_SPEED = 21
    MAX_TORQUE = 22

    COMPLIANT = 30
    PID = 31
    TEMPERATURE = 32


class OrbitaActuator:
    def __init__(self, id: int) -> None:
        self.id = id

        self.disk_bottom = OrbitaDisk()
        self.disk_middle = OrbitaDisk()
        self.disk_top = OrbitaDisk()
        self.disks = [self.disk_top, self.disk_middle, self.disk_bottom]

    def update_value(self, register: OrbitaRegister, values: bytes):
        assert (len(values) % 3) == 0

        n = len(values) // 3
        disk_values = [values[i * n: (i + 1) * n] for i in range(3)]

        if register == OrbitaRegister.PRESENT_POSITION:
            for disk, val in zip(self.disks, disk_values):
                disk.present_position.update(val)


class OrbitaDisk:
    def __init__(self) -> None:
        self.present_position = Register()

