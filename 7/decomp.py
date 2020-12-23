import numpy as np
import cv2
cam = cv2.VideoCapture(0) # 初始化摄像头
while (True):
    ret, frame = cam.read() # 读取摄像头数据
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) # 转换颜色空间
    # 通过颜色设计模板
    image_mask=cv2.inRange(hsv,np.array([40,50,50]), np.array([80,255,255]))
    # 计算输出图像
    output=cv2.bitwise_and(frame,frame,mask=image_mask)
    cv2.imshow('Original',frame) # 显示原始图像
    cv2.imshow('Output',output) # 显示输出图像
    if cv2.waitKey(1) == ord("q"): # 等待按键
        break
cv2.destroyAllWindows()
cam.release()
