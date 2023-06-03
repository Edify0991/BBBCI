#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import random
import numpy as np
import scipy.signal as signal
from psychopy import visual, event


def mstoframe(ms,fps):
    return np.round(ms * fps / 1000).astype(int)

random.seed()
test_num = int(random.random() * 1000 % 12)  # 实验编号
base_time = 1000  # 基础时间1000ms
test_count = 10  # 实验次数
test_time = 6000        # 实验时长
wait_time = 3000        #准备时间
sensor_time=2000        #预热时间
win = visual.Window(size=(960, 960), color=(255, 255, 255),fullscr=True)  # 窗口
Msperframe = win.getMsPerFrame()  # 刷新间隔

fps = np.round(1000 / Msperframe[0]).astype(int)  # 每秒刷新率
print("your screen's fps is ", fps)
seq = [[0 for i in range(fps)] for i in range(12)]  # 创建对于帧数的数组
block = [0 for i in range(12)]  # 初始化
path = '/home/edifier/code/catkin_ws/src/bci_grip/scripts/Py_PTB/'
filename = [path + '0.png', path + '1.png', path + '2.png', path + '3.png', path + '4.png', path + '5.png', path + '6.png', path + '7.png', path + '8.png', path + '9.png', path + '10.png',
            path + '11.png']
for i in range(12):
    block[i] = visual.ImageStim(win, image=filename[i], pos=(((i % 3) - 1) * 0.66, (0.75 - 0.5 * (i // 3))),
                                size=(0.5, 0.375))
    # 创建图片信息
rectangle = visual.Rect(win,
                        width=0.6, height=0.45,
                        lineWidth=2.5, lineColor='red', lineColorSpace='rgb',
                        fillColor=None,
                        pos=(((test_num % 3) - 1) * 0.66, (0.75 - 0.5 * (test_num // 3)))
                        )
rate = np.asarray([7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])  # 对应数字的刷新频率
n = np.arange(0, fps)
for i in range(12):
    seq[i] = signal.square(2 * np.pi * rate[i] * (n / fps)) * 0.5 + 0.5  # 利用signal.square转换成周期性方波，取0或1，进而实现闪烁。

#========================================================================================================================
test_text=visual.TextStim(win,
                          'Please stare at the picture in red rectangle.',
                          color=(0,0,1),
                          colorSpace='rgb',
                          pos=(0,0),
                          height=0.04
                          )
wait_text=visual.TextStim(win,
                          'Press anykey to continue.',
                          color=(0,0,1),
                          colorSpace='rgb',
                          pos=(0,0),
                          height=0.06
                          )
finish_text=visual.TextBox2(win,
                            'End or re experiment',
                            color=(0,0,1),
                            colorSpace='rgb',
                            pos=(0,0),
                            letterHeight=0.06,
                            )
edit_text=visual.TextBox2(win,
                           text=None,
                           color=(0,0,1),
                           colorSpace='rgb',
                           pos=(0,-0.3),
                           letterHeight=0.08,
                           )
edit_text.editable=True
edit_text.placeholder=True
# -----------------------------------------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------------------------------------
while 1 :
    edit_text.clear()

    for frame in range(mstoframe(wait_time,fps)):
        test_text.draw()
        win.flip()
    
    for frame in range(mstoframe(sensor_time,fps)):
        block[test_num].draw()
        rectangle.draw()
        win.flip()
    
    
    while 1:
        if event.getKeys():
            break
        wait_text.draw()
        win.flip()
    
    for frame in range(mstoframe(2000,fps)):
        for i in range(12):
            block[i].draw()
        rectangle.draw()
        win.flip()
    frame = 0
    while 1:
        if frame >= mstoframe(test_time, fps):
            break
        for num in range(fps):
            if event.getKeys(keyList=['space', 'escape']):
                win.close()
            for i in range(12):
                if seq[i][num]:
                    block[i].draw()
            rectangle.draw()
            win.flip()
        frame = frame + fps

    edit_text.clear()
    while 1:
        finish_text.draw()
        edit_text.draw()
        if(edit_text.getText()=='end'):
            exit(0)
        if(edit_text.getText()=='re'):
            break
        win.flip()
