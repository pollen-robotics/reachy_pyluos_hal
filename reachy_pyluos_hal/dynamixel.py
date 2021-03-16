"""Dynamixel implentation of a Joint."""

from abc import abstractproperty
from enum import Enum
from typing import Dict, List, Tuple, Type
from struct import pack, unpack

import numpy as np
from numpy import clip, deg2rad, pi

from .joint import Joint


class DynamixelModelNumber(Enum):
    """Enum representing the different Dynamixel models."""

    AX18 = 18
    MX28 = 29
    MX64 = 310
    MX106 = 320
    XL320 = 350


class DynamixelError(Enum):
    """Enum representing the different Dynamixel errors."""

    InstructionError = 6
    OverloadError = 5
    ChecksumError = 4
    RangeError = 3
    OverheatingError = 2
    AngleLimitError = 1
    InputVoltageError = 0


class DynamixelMotor(Joint):
    """Dynamixel implentation of a Joint."""

    def __init__(self, id: int, 
                 offset: float, direct: bool, 
                 cw_angle_limit: float, ccw_angle_limit: float,
                 reduction: float,
                 ) -> None:
        """Set up the dynamixel motor with its id, and an offset and direction."""
        self.id = id
        self.offset = offset
        self.direct = direct
        self.cw_angle_limit = cw_angle_limit
        self.ccw_angle_limit = ccw_angle_limit
        self.reduction = reduction

        super().__init__({
            'model_number': (self.model_to_usi, self.model_to_raw),
            'id': (self.id_to_usi, self.id_to_raw),
            'baudrate': (self.baudrate_to_usi, self.baudrate_to_raw),
            'return_delay_time': (self.return_delay_time_to_usi, self.return_delay_time_to_raw),
            'cw_angle_limit': (self.position_to_usi, self.position_to_raw),
            'ccw_angle_limit': (self.position_to_usi, self.position_to_raw),
            'temperature_limit': (self.temperature_to_usi, self.temperature_to_raw),
            'alarm_shutdown': (self.alarm_to_usi, self.alarm_to_raw),

            'torque_enable': (self.torque_enabled_to_usi, self.torque_enabled_to_raw),
            'd_gain': (self.gain_to_usi, self.gain_to_raw),
            'i_gain': (self.gain_to_usi, self.gain_to_raw),
            'p_gain': (self.gain_to_usi, self.gain_to_raw),

            'cw_compliance_margin': (self.gain_to_usi, self.gain_to_raw),
            'ccw_compliance_margin': (self.gain_to_usi, self.gain_to_raw),
            'cw_compliance_slope': (self.gain_to_usi, self.gain_to_raw),
            'ccw_compliance_slope': (self.gain_to_usi, self.gain_to_raw),

            'goal_position': (self.position_to_usi, self.position_to_raw),
            'moving_speed': (self.speed_to_usi, self.speed_to_raw),
            'torque_limit': (self.torque_to_usi, self.torque_to_raw),
            'present_position': (self.position_to_usi, self.position_to_raw),
            'temperature': (self.temperature_to_usi, self.temperature_to_raw),
        })

    @abstractproperty
    def dxl_config(self) -> Dict[str, Tuple[int, int]]:
        """Get registers config: Dict[reg_name, (reg_addr, reglength)]."""
        ...

    def __repr__(self) -> str:
        """Represent DynamixelMotor."""
        return f'<DynamixelMotor type="{self.motor_type}" id={self.id} limits={self.cw_angle_limit, self.ccw_angle_limit}>'

    @abstractproperty
    def max_position(self) -> int:
        """Return the max position dynamixel register value."""
        ...

    @abstractproperty
    def max_radian(self) -> float:
        """Return the max position (in rad)."""
        ...

    @abstractproperty
    def motor_type(self) -> str:
        """Return the motor type."""
        ...

    def find_register_by_addr(self, addr: int) -> str:
        """Find register name by its address."""
        for reg, (reg_addr, _) in self.dxl_config.items():
            if reg_addr == addr:
                return reg
        raise KeyError(addr)

    def get_register_config(self, register: str) -> Tuple[int, int]:
        """Get register addr and length by its name."""
        return self.dxl_config[register]

    def model_to_usi(self, value: bytes) -> DynamixelModelNumber:
        """Convert model."""
        model_number = unpack('H', value)[0]
        return DynamixelModelNumber(value=model_number)

    def model_to_raw(self, model: DynamixelModelNumber) -> bytes:
        """Convert model."""
        return pack('H', model.value)

    def id_to_usi(self, value: bytes) -> int:
        """Convert id to USI."""
        return value[0]

    def id_to_raw(self, id: int) -> bytes:
        """Convert id to raw."""
        if not (1 <= id <= 253):
            raise ValueError(f'Id should be with range (1, 253): {id}!')

        return bytes([id])

    def baudrate_to_usi(self, value: bytes) -> int:
        """Convert baudrate dynamixel id to bps."""
        return {
            0: 2000000,
            1: 1000000,
            3: 500000,
            4: 400000,
            7: 250000,
            9: 200000,
            16: 115200,
            34: 57600,
            103: 19200,
            207: 9600,
        }[value[0]]

    def baudrate_to_raw(self, baudrate: int) -> bytes:
        """Convert bps to dynamixel baudrate id."""
        return bytes([{
            2000000: 0,
            1000000: 1,
            500000: 3,
            400000: 4,
            250000: 7,
            200000: 9,
            115200: 16,
            57600: 34,
            19200: 103,
            9600: 207
        }[baudrate]])

    def return_delay_time_to_usi(self, value: bytes) -> int:
        """Convert return delay time to USI (µs)."""
        return value[0] * 2

    def return_delay_time_to_raw(self, rdt: int) -> bytes:
        """Convert return delay time to raw."""
        if not (0 <= rdt <= 508):
            raise ValueError(f'Id should be with range (0, 508): {id}!')

        return bytes([rdt // 2])

    def alarm_to_usi(self, value: bytes) -> List[DynamixelError]:
        """Convert raw error bytes to list of DynamixelError."""
        error_indices = np.where(np.unpackbits(np.asarray(value[0], dtype=np.uint8))[::-1])[0]
        return [DynamixelError(err) for err in error_indices]

    def alarm_to_raw(self, errors: List[DynamixelError]) -> bytes:
        """Convert dynamixel errors to raw coded bytes."""
        value = 0

        for err in errors:
            value += 2 ** err.value

        return bytes([value])

    def torque_enabled_to_raw(self, value: float) -> bytes:
        """Convert torque-enabled to raw."""
        return bytes([1 if value else 0])

    def torque_enabled_to_usi(self, value: bytes) -> float:
        """Convert torque-enabled to usi."""
        return value[0]

    def position_to_raw(self, value: float) -> bytes:
        """Convert position (in rad) to raw."""
        value *= self.reduction
        value = (value + self.offset) * (1 if self.direct else -1)
        pos_ratio = (value + self.max_radian / 2) / self.max_radian
        pos_ratio = np.clip(pos_ratio, 0, 1)
        dxl_raw_pos = int(round(pos_ratio * (self.max_position - 1), 0))
        return pack('H', dxl_raw_pos)

    def position_to_usi(self, value: bytes) -> float:
        """Convert position to usi (in rad)."""
        dxl_raw_pos = unpack('H', value)[0]
        if not (0 <= dxl_raw_pos < self.max_position):
            if self.logger is not None:
                self.logger.warning(f'Corrupted dynamixel position received ({dxl_raw_pos} should be in (0, {self.max_position}))!')
                dxl_raw_pos = np.clip(dxl_raw_pos, 0, self.max_position)
        pos_ratio = dxl_raw_pos / (self.max_position - 1)
        pos = (pos_ratio * self.max_radian) - self.max_radian / 2
        pos /= self.reduction
        return (pos if self.direct else -pos) - self.offset

    def speed_to_raw(self, value: float) -> bytes:
        """Convert speed (in rad/s) to raw."""
        assert value >= 0
        rpm = abs(value) / (2 * pi) * 60
        dxl_speed = clip(int(rpm / 0.114), 0, 1023)
        return pack('H', dxl_speed)

    def speed_to_usi(self, value: bytes) -> float:
        """Convert speed to usi (in rad/s)."""
        dxl_speed = unpack('H', value)[0]
        assert 0 <= dxl_speed < 2048
        if dxl_speed > 1023:
            cw = True
            dxl_speed -= 1024
        else:
            cw = False
        rpm = dxl_speed * 0.114
        rad_per_s = rpm / 60 * (2 * pi)
        return -rad_per_s if cw else rad_per_s

    def torque_to_raw(self, value: float) -> bytes:
        """Convert torque (in %) to raw."""
        return pack('H', int(round(clip(value, 0, 100) * 10.23)))

    def torque_to_usi(self, value: bytes) -> float:
        """Convert torque to usi (in %)."""
        raw_torque = unpack('H', value)[0]
        assert 0 <= raw_torque < 1024
        return raw_torque / 10.23

    def temperature_to_raw(self, value: int) -> bytes:
        """Convert temperature (in C) to raw."""
        return bytes([clip(value, 0, 255)])

    def temperature_to_usi(self, value: bytes) -> int:
        """Convert temperature to usi (in C)."""
        return int(value[0])

    def gain_to_usi(self, value: bytes) -> int:
        """Convert gain to USI."""
        return unpack('B', value)[0]

    def gain_to_raw(self, gain: int) -> bytes:
        """Convert gain to raw."""
        return pack('B', gain)


class DynamixelMotorV1(DynamixelMotor):
    """Specific motor using protocol V1 registers."""

    dxl_config = {
        'model_number': (0, 2),
        'id': (3, 1),
        'baudrate': (4, 1),
        'return_delay_time': (5, 1),
        'cw_angle_limit': (6, 2),
        'ccw_angle_limit': (8, 2),
        'temperature_limit': (11, 1),
        'alarm_shutdown': (18, 1),

        'torque_enable': (24, 1),
        'd_gain': (26, 1),
        'i_gain': (27, 1),
        'p_gain': (28, 1),
        'goal_position': (30, 2),
        'moving_speed': (32, 2),
        'torque_limit': (34, 2),
        'present_position': (36, 2),
        'temperature': (43, 1),
    }


class DynamixelMotorV2(DynamixelMotor):
    """Specific motor using protocol V2 registers."""

    dxl_config = {
        'model_number': (0, 2),
        'id': (3, 1),
        'baudrate': (4, 1),
        'return_delay_time': (5, 1),
        'cw_angle_limit': (6, 2),
        'ccw_angle_limit': (8, 2),
        'temperature_limit': (12, 1),
        'alarm_shutdown': (18, 1),

        'torque_enable': (24, 1),
        'd_gain': (27, 1),
        'i_gain': (28, 1),
        'p_gain': (29, 1),
        'goal_position': (30, 2),
        'moving_speed': (32, 2),
        'torque_limit': (35, 2),
        'present_position': (37, 2),
        'temperature': (46, 1),
    }


class MX(DynamixelMotorV1):
    """MX specific value."""

    @property
    def max_position(self) -> int:
        """Return the max position dynamixel register value."""
        return 4096

    @property
    def max_radian(self) -> float:
        """Return the max position (in rad)."""
        return deg2rad(360)


class AX18(DynamixelMotorV1):
    """AX specific value."""

    dxl_config = {
        'model_number': (0, 2),
        'id': (3, 1),
        'baudrate': (4, 1),
        'return_delay_time': (5, 1),
        'cw_angle_limit': (6, 2),
        'ccw_angle_limit': (8, 2),
        'temperature_limit': (11, 1),
        'alarm_shutdown': (18, 1),

        'torque_enable': (24, 1),
        'cw_compliance_margin': (26, 1),
        'ccw_compliance_margin': (27, 1),
        'cw_compliance_slope': (28, 1),
        'ccw_compliance_slope': (29, 1),
        'goal_position': (30, 2),
        'moving_speed': (32, 2),
        'torque_limit': (34, 2),
        'present_position': (36, 2),
        'temperature': (43, 1),
    }

    @property
    def max_position(self) -> int:
        """Return the max position dynamixel register value."""
        return 1024

    @property
    def max_radian(self) -> float:
        """Return the max position (in rad)."""
        return deg2rad(300)

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'AX18'


class MX106(MX):
    """MX106 impl."""

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'MX106'


class MX64(MX):
    """MX64 impl."""

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'MX64'


class MX28(MX):
    """MX28 impl."""

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'MX28'


class XL320(DynamixelMotorV2):
    """XL320 specific value."""

    @property
    def max_position(self) -> int:
        """Return the max position dynamixel register value."""
        return 1024

    @property
    def max_radian(self) -> float:
        """Return the max position (in rad)."""
        return deg2rad(300)

    @property
    def motor_type(self):
        """Return the motor type."""
        return 'XL320'


def get_motor_from_model(model: DynamixelModelNumber) -> Type[DynamixelMotor]:
    """Get the motor class corresponding to the specified model number."""
    return {
        DynamixelModelNumber.AX18: AX18,
        DynamixelModelNumber.MX28: MX28,
        DynamixelModelNumber.MX64: MX64,
        DynamixelModelNumber.MX106: MX106,
        DynamixelModelNumber.XL320: XL320,
    }[model]
