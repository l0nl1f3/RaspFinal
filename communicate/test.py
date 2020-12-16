import serial
from time import sleep 

ser = serial.Serial('/dev/ttyUSB0', 38400, timeout = 1)
ser.close()
ser.open()
print(ser.name)
try:
    while True:
        response = ser.readline();
        print(response, '!!')
        sleep(0.1)
except KeyboardInterrupt:
    ser.close()
