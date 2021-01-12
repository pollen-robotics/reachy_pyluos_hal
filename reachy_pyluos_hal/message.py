import struct

from collections import namedtuple


class MsgType:
    MSG_TYPE_DXL_GET_REG = 10  # [MSG_TYPE_DXL_GET_REG, DXL_ID, DXL_REG, NB_BYTES]
    MSG_TYPE_DXL_SET_REG = 11  # [MSG_TYPE_DXL_SET_REG, DXL_ID, DXL_REG, (VAL)+]
    # From Dxl --> PC
    MSG_TYPE_DXL_PUB_DATA = 15  # [MSG_TYPE_DXL_PUB_DATA, DXL_ID, DXL_REG, ERR1, ERR2, (VAL)+]

    # Orbita

    # FAN
    # From PC -> Fan
    MSG_TYPE_FAN_GET_STATE = 30  # [MSG_TYPE_FAN_GET_STATE, FAN_ID]
    MSG_TYPE_FAN_SET_STATE = 31  # [MSG_TYPE_FAN_SET_STATE, FAN_ID, STATE]
    # From Fan --> PC
    MSG_TYPE_FAN_PUB_DATA = 35  # [MSG_TYPE_FAN_PUB_DATA, FAN_ID, STATE]

    # Force Sensor
    MSG_TYPE_FORCE_SENSOR_PUB = 40  # [MSG_TYPE_FORCE_SENSOR_GET_VAL, SENSOR_ID, VAL]

    # SYSTEM
    MSG_TYPE_READY = 200
    MSG_MODULE_ASSERT = 222  # [MSG_MODULE_ASSERT, CHAR1, CHAR2, ...]


class Message(list):
    def __init__(self, payload) -> None:
        self.extend([255, 255, len(payload)] + payload)

    @property
    def bytes(self):
        return bytes(self)

    @classmethod
    def ready(cls):
        return Message(payload=[MsgType.MSG_TYPE_READY])


DynamixelUpdate = namedtuple('DynamixelUpdate', ['id', 'register', 'error', 'value'])
AssertMessage = namedtuple('AssertMessage', ['msg'])

def parse(data: bytes):
    if data[0] == MsgType.MSG_MODULE_ASSERT:
        return AssertMessage(data[1:].decode())


    elif data[0] == MsgType.MSG_TYPE_DXL_PUB_DATA:
        dxl_id, dxl_reg, dxl_err = struct.unpack('BBH', data[1:5])
        val_size = len(data) - 5
        if val_size == 1:
            dxl_val = struct.unpack('B', data[5:6])[0]
        elif val_size == 2:
            dxl_val = struct.unpack('H', data[5:7])[0]
        else:
            dxl_val = struct.unpack('B' * val_size, data[5:])

        return DynamixelUpdate(dxl_id, dxl_reg, dxl_err, dxl_val)
