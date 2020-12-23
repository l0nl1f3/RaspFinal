import cv2 # 加载 OpenCV 库
cam = cv2.VideoCapture(0) # 打开摄像头
cam.set(3, 1024) # 设置图像宽度
cam.set(4, 768) # 设置图像高度
while(True):
    ret, frame = cam.read() # 读入一帧图像
    cv2.namedWindow("Test", 0)
    cv2.resizeWindow("Test", 320, 160)
    cv2.imshow('Test',frame) # 显示图像
    if cv2.waitKey(1) == ord("q") : # 等待按键
        break

cam.release()
cv2.destroyAllWindows()
