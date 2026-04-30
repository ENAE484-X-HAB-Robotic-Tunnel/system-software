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
states = np.array([
    [0.5, 0, 0, 0, 0, 0],
    [0.5, 0, 0, 0.5, 0, 0],
    [0.5, 0, 0, -0.5, 0, 0],
    [0.5, 0, 0, 0, 0, 0]
])

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

    time.sleep(0.5)