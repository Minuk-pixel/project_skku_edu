# utils/control_utils.py

import serial
import time


class CarController:
    def __init__(self, port="COM4", baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # 아두이노 초기화 대기

    def send_command(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def forward(self):
        self.send_command("F")

    def backward(self):
        self.send_command("B")

    def stop(self):
        self.send_command("S")

    def steer_left(self):
        self.send_command("L")

    def steer_right(self):
        self.send_command("R")

    def steer_center(self):
        self.send_command("C")

    def steer_angle(self, angle):
        self.send_command(f"A{angle}")  # 예: A-15, A0, A20

    def close(self):
        self.ser.close()
