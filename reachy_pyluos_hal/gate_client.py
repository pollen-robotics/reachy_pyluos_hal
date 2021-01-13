import time

from queue import Queue
from serial import Serial
from threading import Event, Lock, Thread

from .message import Message


class GateClient:
    def __init__(self, port: str, msg_queue: Queue) -> None:
        self.serial = Serial(port=port, baudrate=1000000)
        self.msg_queue = msg_queue

        self.send_lock = Lock()
        self.poll_msgs_sync = Event()
        self.keep_alive_sync = Event()
        self.last_send = 0

        t = Thread(target=self.poll_msgs)
        t.daemon = True
        t.start()

        t = Thread(target=self.keep_alive)
        t.daemon = True
        t.start()

        self.poll_msgs_sync.wait()
        self.keep_alive_sync.wait()

    def send_msg(self, msg):
        if (time.time() - self.last_send) < 0.001:
            time.sleep(0.001)

        with self.send_lock:
            self.serial.write(msg.bytes)
            self.last_send = time.time()

    def poll_msgs(self):
        self.poll_msgs_sync.set()

        while True:
            header = self.serial.read(3)
            assert header[0] == header[1] == 255
            payload_size = header[2]

            payload = self.serial.read(payload_size)
            self.msg_queue.put(payload)

    def keep_alive(self):
        self.keep_alive_sync.set()

        while True:
            self.send_msg(Message.ready())
            time.sleep(1)


