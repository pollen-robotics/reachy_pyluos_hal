from reachy_pyluos_hal.dxl_convert import any_from_raw, any_to_raw, load_from_raw, load_to_raw, pos_from_raw, pos_to_raw, as_bytes
from threading import Event

from .message import Message, MsgType


DXL_REGISTER = {
    'torque_enable': (24, 1, any_from_raw, any_to_raw),
    'goal_position': (30, 2, pos_from_raw, pos_to_raw),
    'moving_speed': (32, 2, any_from_raw, any_to_raw),
    'torque_limit': (34, 2, load_from_raw, load_to_raw),
    'present_position': (36, 2, pos_from_raw, pos_to_raw),
    'present_temperature': (43, 1, any_from_raw, any_to_raw),
}

DXL_ADDR_2_REG = {
    addr: name
    for name, (addr, _, _, _) in DXL_REGISTER.items()
}


class Dynamixel:
    def __init__(self, gate, id, model, offset=0, direct=True) -> None:
        self.gate = gate
        self.id = id
        self.model = model
        self.offset = offset
        self.direct = direct

        self.fields = {
            field: DxlField(field)
            for field in DXL_REGISTER.keys()
        }

    def get_value(self, register):
        if register not in ('present_position', 'present_temperature'):
            self.ask_register_value(register)
        self.fields[register].sync.wait()
        val = self.fields[register].value
        if register in ('present_position', 'goal_position'):
            val = DXL_REGISTER[register][2](val, self.model, self.offset, self.direct)
        else:
            val = DXL_REGISTER[register][2](val, self.model)
        return val            

    def send_value(self, register, value):
        addr, nb_bytes, _, cvt = DXL_REGISTER[register]

        if register in ('present_position', 'goal_position'):
            value = cvt(value, self.model, self.offset, self.direct)
        else:
            value = cvt(value, self.model)
        values = as_bytes(value, nb_bytes)
        self.gate.send_msg(Message([MsgType.MSG_TYPE_DXL_SET_REG, self.id, addr] + values))

    def wait_for_sync(self):
        for name, field in self.fields.items():
            self.ask_register_value(name)
            field.sync.wait()

    def ask_register_value(self, register):
        self.fields[register].sync.clear()

        addr, length, _, _ = DXL_REGISTER[register]
        self.gate.send_msg(Message([MsgType.MSG_TYPE_DXL_GET_REG, self.id, addr, length]))
        
    def set_register_value(self, addr, value):
        register = DXL_ADDR_2_REG[addr]

        self.fields[register].value = value
        self.fields[register].sync.set()


class DxlField:
    def __init__(self, name) -> None:
        self.sync = Event()
        self.value = None
