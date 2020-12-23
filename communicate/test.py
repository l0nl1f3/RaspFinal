import serial
from time import sleep 

ser = serial.Serial('/dev/ttyUSB0', 38400, timeout = 1)

ser.close()
ser.open()
sleep(3)
try:
    while True:
   #     o = str.encode(input())
   #     ser.write(o)
   #     sleep(0.1)
        ser.write(str.encode('A'));
        sleep(0.10)
        ser.write(str.encode('Z'));
        sleep(0.35)
        break
except KeyboardInterrupt:
    print("!!")
    ser.write(str.encode('I'));
    ser.close()
