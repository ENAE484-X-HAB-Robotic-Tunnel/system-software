import serial
import time
import numpy as np
from StewartPlatform import StewartPlatform

ser = serial.Serial()
ser.port = 'COM4'
ser.baudrate = 115200
ser.dtr = False      # Disables arduino reset
ser.open()
time.sleep(0.5)      # short sleep
ser.reset_input_buffer()   # clear any garbage bytes
ser.reset_output_buffer()

# x, y, z, roll, pitch, yaw
states0 = np.array([
    [0.45, 0, 0, 0, 0, 0]
])

states1 = np.array([
    [0.45,  0, 0, 0, 0, 0],
    [0.5, 0, 0, 0, 0, 0],
    [0.4, 0, 0, 0, 0, 0],
    [0.45,  0, 0, 0, 0, 0],
    [0.45,  0.08, 0, 0, 0, 0],
    [0.45, -0.08, 0, 0, 0, 0],
    [0.45,  0, 0, 0, 0, 0],
    [0.45, 0,  0.08, 0, 0, 0],
    [0.45, 0, -0.08, 0, 0, 0],
    [0.45,  0, 0, 0, 0, 0],
    [0.45, 0, 0,  0.5, 0, 0],
    [0.45, 0, 0, -0.5, 0, 0],
    [0.45,  0, 0, 0, 0, 0],
    [0.45, 0, 0, 0,  0.5, 0],
    [0.45, 0, 0, 0, -0.5, 0],
    [0.45,  0, 0, 0, 0, 0],
    [0.45, 0, 0, 0, 0,  0.5],
    [0.45, 0, 0, 0, 0, -0.5],
    [0.45,  0, 0, 0, 0, 0]
])


if __name__ == '__main__':
    states = states0 #_lissajous

    for X in states:
        print("State: ", X)
        SP = StewartPlatform(X, 12.5 * 0.0254, 12 * 0.0254)

        msg = ",".join(f"{x:.8f}" for x in SP.l_legs) + "\n"
        ser.write(msg.encode('ascii'))  # force pure ASCII
        # print(msg.encode('ascii'))

        print("Sent...")

        line = ser.readline().decode('utf-8', errors='replace').strip()
        print(line)
        line = ser.readline().decode('utf-8', errors='replace').strip()
        print(line)

        # time.sleep(0.5)

# class SerialInterface:
#     def __init__(self, port='COM4', baud=115200):
#         self.ser = serial.Serial(port, baudrate=baud)
#         self.ser.dtr = False
#         time.sleep(0.5)
#         self.ser.reset_input_buffer()
#         self.ser.reset_output_buffer()

#     def send_legs(self, legs):
#         msg = ",".join(f"{x:.6f}" for x in legs) + "\n"
#         self.ser.write(msg.encode('ascii'))

#     def read_lines(self, n=1):
#         lines = []
#         for _ in range(n):
#             line = self.ser.readline().decode('utf-8', errors='replace').strip()
#             lines.append(line)
#         return lines