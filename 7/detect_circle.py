import cv2
cam = cv2.VideoCapture(0) # 打开摄像头

pos = [0, 0, 0]

while (True):
    ret , frame = cam.read() # 获取摄像头数据
    grey = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) # 色彩变换
    blur = cv2.blur(grey,(5,5)) # 过滤噪声
    circles = cv2.HoughCircles(blur, # 识别圆形
    method = cv2.HOUGH_GRADIENT,dp = 1,minDist = 200,
    param1 = 100,param2 = 33,minRadius = 30,maxRadius = 175)
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
    if cv2.waitKey(1) == ord("q"): # 等待按键
        break
cv2.destroyAllWindows() # 关闭窗口
cam.release() # 释放摄像头
