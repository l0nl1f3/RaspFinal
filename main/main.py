import cv2
import copy
import serial
from time import sleep

ser = serial.Serial('/dev/ttyUSB0', 38400, timeout = 1)
ser.close()
ser.open()
sleep(3)

cam = cv2.VideoCapture(0) # 打开摄像头
cam.set(3, 480)
cam.set(4, 360)

def detect(pos): 
    ret , frame = cam.read() # 获取摄像头数据
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # 色彩变换
    blur = cv2.blur(grey,(5,5)) # 过滤噪声
    circles = cv2.HoughCircles(blur, # 识别圆形
    method = cv2.HOUGH_GRADIENT,dp = 1,minDist = 100,
    param1 = 100,param2 = 33,minRadius = 15,maxRadius = 85)
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
        cv2.imshow('Detected', frame)
        if n != 1:
            pos[0] = tpos[0]
            pos[1] = tpos[1]
            pos[2] = tpos[2]
        return True
    else:
        return False

def step_forward():
    ser.write(str.encode('A'))
    sleep(0.08)
    ser.write(str.encode('Z'))
    sleep(0.08)

def step_backward():
    ser.write(str.encode('E'))
    sleep(0.08)
    ser.write(str.encode('Z'))
    sleep(0.20)

th1 = 1.5
pos = [0, 0, 0]
pre_pos = [0, 0, 0]
while (True):
    if (detect(pos)):
        if (pre_pos[2] == 0):
            pre_pos = copy.deepcopy(pos)
        else:
            print(pre_pos, pos)
            if (abs(pos[2] - pre_pos[2]) > th1):
                T = 0
                while (T <= 10 and abs(pos[2] - pre_pos[2]) > th1):
                    T += 1
                    if (pos[2] < pre_pos[2] + 1.):
                        print("forward")
                        step_forward() 
                    else:
                        step_backward()
                        print("backward")
                    while (not detect(pos)):
                        pass
                sleep(0.2)
                print('end adjust !!')
                pre_pos = [0, 0, 0]
                # detect(pos)
               # pre_pos = copy.deepcopy(pos)
               # print(pos, pre_pos)
                    
    else:
        print('miss')    
    if cv2.waitKey(1) == ord("q"): # 等待按键
        break
    sleep(0.005)

ser.write(str.encode('I'))
ser.close()
cv2.destroyAllWindows()
cam.release()  # 关闭窗口
