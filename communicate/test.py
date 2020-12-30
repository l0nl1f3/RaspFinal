import serial
from time import sleep 

ser = serial.Serial('/dev/ttyUSB0', 38400, timeout = 1)

ser.close()
ser.open()
sleep(3)
try:
    while True:
        sleep(0.1)
        print('A')
        ser.write(str.encode('A'));
except KeyboardInterrupt:
    print("!!")
    ser.write(str.encode('I'));
    ser.close()
