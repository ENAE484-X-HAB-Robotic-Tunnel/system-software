import serial

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

message = "10"

ser.write(message.encode('utf-8'))