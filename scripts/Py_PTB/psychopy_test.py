#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from psychopy import visual
from psychopy.hardware import camera
import cv2
import numpy as np
def _updata(x,y,w,h):#将opencv中坐标和长度转换为psychopy的坐标
    global a,rect
    center_x=((x+w/2)-a.size[0])/a.size[0]/2
    center_y=(-(y+h/2)+a.size[1])/a.size[1]/2
    rect_w=w/a.size[0]/2
    rect_h=h/a.size[1]/2
    rect=visual.Rect(
                    win=win,
                    width=rect_w,height=rect_h,
                    lineColor='red',
                    fillColor=None,
                    pos=(center_x,center_y),
                    )
    print(center_x,center_y,rect_w,rect_h)

win=visual.Window(
                  size=(1280,720),
                  color=(-1,-1,-1)
                  )
#选择相机的设备号，以及显示窗口
camera=camera.Camera(device=0,win=win)
#打开相机
camera.open()
#开始录制
camera.record()
print(camera)
movie=visual.ImageStim(win,camera,size= (1,1),colorSpace='rgb')
while camera.recordingTime < 5 :
    #获取当前视频帧所有信息
    a=camera.getVideoFrame()
    #a.size输出相机分辨率，reshape将一维rgb数组转换为Mat数组以便后续opencv图像处理
    frame=np.reshape(a.colorData,(a.size[0],a.size[1],3))
    #将rgb图像文件转换为BGR
    frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
    #将BGR灰度处理
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #对处理后进行二值化
    thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)[1]
    # 寻找轮廓
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #第一个轮廓
    cnt = contours[0]
    #边界矩形
    x,y,w,h = cv2.boundingRect(cnt)
    _updata(x,y,w,h)
    print(x,y,w,h)
    movie.draw()
    rect.draw()
    win.flip()
camera.save('1.mp4',useThreads=False)
camera.stop() # stop
camera.close()#关闭相机
