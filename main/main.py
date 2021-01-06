import cv2
import copy
import serial
from time import sleep

ser = serial.Serial('/dev/ttyUSB0', 38400, timeout = 1)
ser.close()
ser.open()
sleep(3)

cam = cv2.VideoCapture(0) # 打开摄像头
cam.set(3, 960)
cam.set(4, 720)

def detect(pos): 
    ret , frame = cam.read() # 获取摄像头数据
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # 色彩变换
    blur = cv2.blur(grey,(5,5)) # 过滤噪声
    circles = cv2.HoughCircles(blur, # 识别圆形
    method = cv2.HOUGH_GRADIENT,dp = 1,minDist = 100,
    param1 = 100,param2 = 33,minRadius = 35,maxRadius = 170)
    if circles is not None: # 识别到圆形
        n = circles.shape[1]
        tpos = pos
        for i in circles [0,:]: # 画出识别的结果
            if n == 1:
                cv2.circle(frame, (i[0], i[1]), i[2], (0, 255 ,0), 2)
                cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
                # print(abs(pos - i))
                # print(pos, i)
                pos[0] = i[0]; pos[1] = i[1]; pos[2] = i[2]
                # print(pos, i)
            else:
                d = abs(pos - i)
                if max(d[0], d[1]) < 55:
                    tpos = i
                    cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
                # print(d)
        #cv2.imshow('Detected', frame)
        if n != 1:
            pos[0] = tpos[0]
            pos[1] = tpos[1]
            pos[2] = tpos[2]
        return True
    else:
        return False

def sw(string):
    ser.write(str.encode(string))

def _f():
    sw('A')

def _rf():
    sw('B')

def _r():
    sw('C')

def _rb():
    sw('D')

def _b():
    sw('E')

def _lb():
    sw('F')

def _l():
    sw('G')

def _lf():
    sw('H')

def _stop():
    sw('Z')

def _op_motor():
    sw('I')

hi_thres = 1800 * 4
lo_thres = 1000 * 4
pos = [0, 0, 0]
pre_pos = [0, 0, 0]

try:
    while (True):
        miss = 0
        if (detect(pos)):
            miss = 0
            area = pos[2] * pos[2]
            print(pos, area)
            x_shift = pos[0] - 480
            lef = x_shift < -140
            rig = x_shift > +140
            fwd = area < lo_thres
            bwd = area > hi_thres
            if (not lef and not rig and not fwd and not bwd):
                _stop()
            elif (not fwd and not bwd):
                if lef:
                    _l()
                else:
                    _r()
            elif (not lef and not rig):
                if fwd:
                    _f()
                else:
                    _b()
            else:
                t = lef * 2 + fwd
                if (t == 0):
                    _lb()
                if (t == 1):
                    _rf()
                if (t == 2):
                    _rb()
                if (t == 3): 
                    _lf()
        else:
            miss += 1
            if miss > 5:
                _stop()
                sleep(0.005)
            print('miss')    
        if cv2.waitKey(1) == ord("q"): # 等待按键
            break
        sleep(0.002)
except KeyboardInterrupt:
    ser.write(str.encode('Z'))
    sleep(0.2)
    ser.close()
    cv2.destroyAllWindows()
    cam.release()  # 关闭窗口
