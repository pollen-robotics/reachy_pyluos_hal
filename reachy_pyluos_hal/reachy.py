from queue import Queue
from threading import Thread

from .gate_client import GateClient
from .message import AssertMessage, DynamixelUpdate, parse
from .dynamixel import Dynamixel
from .dxl_convert import DxlModel


class Reachy:
    def __init__(self, left_arm_port, right_arm_port) -> None:
        self.msg_queue: Queue = Queue()
        self.left_arm = GateClient(port=left_arm_port, msg_queue=self.msg_queue)
        self.right_arm = GateClient(port=right_arm_port, msg_queue=self.msg_queue)

        self.dxls = {
            10: Dynamixel(gate=self.right_arm, id=10, model=DxlModel.MX106, offset=90, direct=False),
            11: Dynamixel(gate=self.right_arm, id=11, model=DxlModel.MX64, offset=90, direct=False),
            12: Dynamixel(gate=self.right_arm, id=12, model=DxlModel.MX64, direct=False),
            13: Dynamixel(gate=self.right_arm, id=13, model=DxlModel.MX64, direct=False),
            14: Dynamixel(gate=self.right_arm, id=14, model=DxlModel.AX18, direct=False),
            15: Dynamixel(gate=self.right_arm, id=15, model=DxlModel.MX28, direct=False),
            16: Dynamixel(gate=self.right_arm, id=16, model=DxlModel.AX18, direct=False),
            17: Dynamixel(gate=self.right_arm, id=17, model=DxlModel.AX18),
            20: Dynamixel(gate=self.left_arm, id=20, model=DxlModel.MX106, offset=90),
            21: Dynamixel(gate=self.left_arm, id=21, model=DxlModel.MX64, offset=-90, direct=False),
            22: Dynamixel(gate=self.left_arm, id=22, model=DxlModel.MX64, direct=False),
            23: Dynamixel(gate=self.left_arm, id=23, model=DxlModel.MX64, direct=False),
            24: Dynamixel(gate=self.left_arm, id=24, model=DxlModel.AX18, direct=False),
            25: Dynamixel(gate=self.left_arm, id=25, model=DxlModel.MX28, direct=False),
            26: Dynamixel(gate=self.left_arm, id=26, model=DxlModel.AX18, direct=False),
            27: Dynamixel(gate=self.left_arm, id=27, model=DxlModel.AX18),
        }

        t = Thread(target=self.poll_messages)
        t.daemon = True
        t.start()

    def handle_message(self, msg: bytes):
        msg = parse(msg)
        
        if isinstance(msg, AssertMessage):
            print(f'Error "{msg.msg}"')

        if isinstance(msg, DynamixelUpdate):
            if msg.error:
                print(msg)
            else:
                self.dxls[msg.id].set_register_value(msg.register, msg.value)

    def poll_messages(self):
        while True:
            self.handle_message(self.msg_queue.get())
