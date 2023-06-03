# 调用对应的库
import cv2
import numpy as np
# 视频参数设置
cap = cv2.VideoCapture(0)
while True:
   # 读取视频
   ret, frame = cap.read()
   gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
   # 阈值设置
   thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)[1]
   # 寻找轮廓
   contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   x,y,w,h = cv2.boundingRect(contours[0])
   print(x,y,w,h)
   # 画出轮廓，-1,表示所有轮廓，颜色为(0, 255, 0)，即Green，粗细为2
   cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
   brcnt = np.array([[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]])
   cv2.drawContours(frame, [brcnt], -1, (255, 255, 255), 2)
   # 弹出显示视频
   cv2.imshow('Capture', frame)
   cv2.imshow('Capture_gray', thresh)
   # Q键退出
   if cv2.waitKey(1) & 0xFF == ord('q'):
       break

cap.release()
cv2.destroyAllWindows()