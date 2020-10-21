import serial
import numpy as np
from threading import Thread, Event
from queue import Queue


FLOAT_SIZE = 4
MSG_SIZE = FLOAT_SIZE*(3+3+4)+3
QUEUE_SIZE = 128  # 0 for no limit


def to_euler_angles(q):
    x, y, z, w = q.swapaxes(0, 1)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    yaw = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1, 1)
    pitch = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    roll = np.arctan2(t3, t4)
    return np.array((roll, pitch, yaw))


class SensorFusion:
    def __init__(self, baudrate=115200, parity=serial.PARITY_NONE, stop=serial.STOPBITS_ONE, port='COM3'):
        self.q = Queue(maxsize=QUEUE_SIZE)
        self.end = Event()
        self.t = Thread(target=self.uart_sync)
        self.config(baudrate, parity, stop, port)
        self.ser = None

    def config(self, baudrate=115200, parity=serial.PARITY_NONE, stop=serial.STOPBITS_ONE, port='COM3'):
        self.baudrate = baudrate
        self.parity = parity
        self.stopbits = stop
        self.port = port

    def get_data(self):
        acc = np.empty((0, 3))
        gyro = np.empty((0, 3))
        quat = np.empty((0, 4))

        while not self.q.empty():
            msg = self.q.get()
            if not isinstance(msg, serial.SerialException):
                a, g, q = self.msg_decode(msg)
                acc = np.vstack((acc, a))
                gyro = np.vstack((gyro, g))
                quat = np.vstack((quat, q))
            else:
                raise msg

        return {'acc': acc, 'gyro': gyro, 'quat': quat}

    def is_connected(self):
        return self.ser is not None and self.ser.isOpen()

    def is_running(self):
        return self.is_connected() and self.t.is_alive()

    def start(self):
        if self.is_connected() and not self.is_running():
            self.t = Thread(target=self.uart_sync)
            self.t.start()

    def stop(self):
        if self.is_running():
            self.end.set()

    def connect(self):
        if self.is_connected():
            self.disconnect()

        self.ser = serial.Serial(port=self.port, baudrate=self.baudrate, stopbits=self.stopbits, parity=self.parity)

    def disconnect(self):
        if self.is_running():
            self.stop()

        if self.is_connected():
            self.ser.close()

    def uart_sync(self):
        try:
            self.ser.flush()
            buffer = self.ser.read(MSG_SIZE)
            while not self.end.is_set():
                agq = buffer[0::FLOAT_SIZE * 3 + 1][:3]
                try:
                    # check if the buffer fits the 'AfffGfffQffff' format (where f represets a float)
                    agq = agq.decode(encoding='ascii')
                except UnicodeDecodeError:
                    pass

                if agq == 'AGQ':
                    self.q.put(buffer)
                    buffer = self.ser.read(MSG_SIZE)

                else:
                    buffer = buffer[1:] + self.ser.read(1)

        except (serial.SerialException, serial.SerialTimeoutException) as e:
            self.q.put(e)
        else:
            self.q = Queue(maxsize=QUEUE_SIZE)
        finally:
            self.end.clear()

    @staticmethod
    def msg_decode(buffer):
        acc = np.frombuffer(buffer[1:1+3*FLOAT_SIZE], dtype=np.float32)
        gyro = np.frombuffer(buffer[2+3*FLOAT_SIZE:2+6*FLOAT_SIZE], dtype=np.float32)
        quat = np.frombuffer(buffer[3+6*FLOAT_SIZE:], dtype=np.float32)
        return acc, gyro, quat

    @staticmethod
    def get_angles(data):
        quat = data['quat']
        roll, pitch, _ = 180 / 3.14 * to_euler_angles(quat)
        return roll, pitch



