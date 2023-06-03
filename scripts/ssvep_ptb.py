#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import random
import numpy as np
import rospy
import scipy.signal as signal
from psychopy import visual, event, core
import scipy.signal as signal
import cv2
import sys
import math
from std_msgs.msg import String, Bool, Float64MultiArray
from threading import Thread, current_thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

sig = 0
image = []
Pos = []

def mstoframe(ms, fps):
    return np.round(ms * fps / 1000).astype(int)

def StateResult_Callback(msg):
    global sig
    if msg.data == True:
        sig =  1

def Picture_Callback(msg):
    global image
    image = CvBridge.imgmsg_to_cv2(msg, "bgr8")

def PositionLength_Callback(msg):
    global Pos
    ReceivePosLen = np.array(msg.data);
    # 将 ReceivePosLen 重构为 (length(ReceivePosLen)/3,3) 的新的矩阵 Pos
    # len(ReceivePosLen)/3为物块个数，3表示位置x、y和矩形边长
    Pos = ReceivePosLen.reshape(3, ReceivePosLen.shape[0] / 3)
    Pos = Pos.T
    
def ssvep_pre():
    global sig, image, Pos
    bg_colour = [-1, -1, -0.25]
    win = visual.Window(
    size=[800, 600],
    units="pix",
    fullscr=True,
    color=[-1,-1,-1],
    )
    screenXpixels, screenYpixels = win.size   # Get the size of the on screen window
    Msperframe = win.getMsPerFrame()  # 刷新间隔
    fps = np.round(1000 / Msperframe[0]).astype(int)  # 每秒刷新率
    print("your screen's fps is ", fps)
    path = '/home/edifier/code/catkin_ws/src/bci_grip/scripts/' 
    theImage_PseudoKey = path + 'PseudoKey.jpg'
    Image_PseudoKey = cv2.imread(theImage_PseudoKey, cv2.IMREAD_UNCHANGED)
    Image_PseudoKey = cv2.resize(Image_PseudoKey, [200,200], cv2.INTER_CUBIC)    # 双三次样条插值
    ImageSize_u_PseudoKey = Image_PseudoKey.shape[1] # 200 -x
    ImageSize_v_PseudoKey = Image_PseudoKey.shape[0] # 200 -y
    # Position and size of the images [x, y, width, height] 
    xCenter, yCenter = 0, 0 # 屏幕中心
    
    Position_pseudo_LU = 0.5 * np.array([-screenXpixels, screenYpixels, 0, 0]) + 0.5 * np.array([ImageSize_u_PseudoKey, -ImageSize_v_PseudoKey, 0, 0]) + np.array([0, 0, ImageSize_u_PseudoKey, ImageSize_v_PseudoKey])   # 左上
    Position_pseudo_RU = 0.5 * np.array([screenXpixels, screenYpixels, 0, 0]) + 0.5 * np.array([-ImageSize_u_PseudoKey, -ImageSize_v_PseudoKey, 0, 0]) + np.array([0, 0, ImageSize_u_PseudoKey, ImageSize_v_PseudoKey])  # 右上
    Position_pseudo_RD = 0.5 * np.array([screenXpixels, -screenYpixels, 0, 0]) + 0.5 * np.array([-ImageSize_u_PseudoKey, ImageSize_v_PseudoKey, 0, 0]) + np.array([0, 0, ImageSize_u_PseudoKey, ImageSize_v_PseudoKey])   # 右下
    Position_pseudo_LD = 0.5 * np.array([-screenXpixels, -screenYpixels, 0, 0]) + 0.5 * np.array([ImageSize_u_PseudoKey, ImageSize_v_PseudoKey, 0, 0]) + np.array([0, 0, ImageSize_u_PseudoKey, ImageSize_v_PseudoKey])   # 左下
    Flash_Position_Pseudo = np.zeros((4, 4))
    Flash_Position_Pseudo[0][:] = Position_pseudo_LU
    Flash_Position_Pseudo[1][:] = Position_pseudo_RU
    Flash_Position_Pseudo[2][:] = Position_pseudo_RD
    Flash_Position_Pseudo[3][:] = Position_pseudo_LD
    print(Flash_Position_Pseudo[0][:])
    amplitude = 0.5 # 振幅
    freq_pre = 17
    staP_pre = 0.0
    angF_pre = 2 * np.pi * freq_pre
    wait_text_1=visual.TextStim(
                            win=win,
                            text='Press anykey to continue.',
                            color=[1, 1, 1],
                            colorSpace='rgb',
                            pos=(0,0),
                            height=60
                            )
    wait_text_2=visual.TextStim(
                            win=win,
                            text='Wait for picture to come.',
                            color=[1, 1, 1],
                            colorSpace='rgb',
                            pos=(0,0),
                            height=60
                            )
    block = [0 for i in range(4)]  # 初始化
    for i in range(4):
        block[i] = visual.ImageStim(
                                            win=win, 
                                            image=theImage_PseudoKey, 
                                            pos=[Flash_Position_Pseudo[i][0], Flash_Position_Pseudo[i][1]],
                                            size=[Flash_Position_Pseudo[i][2], Flash_Position_Pseudo[i][3]]
                                            )
    seq = [[0 for i in range(fps)] for i in range(5)]  # 创建对于帧数的数组
    n = np.arange(0, fps)
    for i in range(4):
        seq[i] = [(math.sin(angF_pre * (j / fps)) * 0.5 + 0.5) for j in range(n.shape[0])]  # 利用math.sin转换成正弦波，并且利用offset将幅值转换到0～1,进而利用对比度实现闪烁。
    while 1:
        wait_text_1.draw()
        win.flip()
        if event.getKeys(keyList=['space', 'escape']):
            break
    # 伪密钥阶段
    while 1:
        if event.getKeys(keyList=['space', 'escape']) or sig == 1:
            break
        for num in range(4):
            block[num].opacity = seq[num][i % fps]
            block[num].draw()
        i += 1
        win.flip()
    # 伪密钥检测后，进入了下一个场景图片接收与显示的阶段
    # After the detection of the pseudo key,
    # the next stage of picture reception and display is entered
    win.flip()  # 将屏幕win涂成black
    # 等待图片何位置信息的订阅
    duration_s = 3.0  
    clock = core.Clock()
    while clock.getTime() < duration_s:
        wait_text_2.draw()
        win.flip()
        if event.getKeys(keyList=['space', 'escape']):
            break
    
    event.waitKeys()
    win.close()
    rospy.signal_shutdown("Congratulation! The experiment is end!")
    

if __name__ == '__main__':
    try:
        # Python 语言初始化 ROS 节点
        rospy.init_node('ssvep_ptb', anonymous=True)
        rospy.Subscriber("/StateResultNode", Bool, StateResult_Callback)
        rospy.Subscriber("/PictureNode", Image, Picture_Callback)   # 开启图像订阅节点
        rospy.Subscriber("/PositionLengthNode", Float64MultiArray, PositionLength_Callback)
        thread_ssvep = Thread(target=ssvep_pre, args="", name="线程1")
        thread_ssvep.setDaemon(True)
        thread_ssvep.start()
        rospy.spin()
            
        # event.waitKeys()
        # win.close()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)