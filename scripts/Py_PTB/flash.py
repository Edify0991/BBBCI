#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
from psychopy import visual, core,event
import numpy as np

win = visual.Window(size=(720, 960), pos=(500, 243), color=(0, 0, 0))
timer = core.Clock()              #获取当前时间刻
timer.reset()                     #当前时间刻归零
path = '/home/edifier/code/catkin_ws/src/bci_grip/scripts/Py_PTB/';
block_1= visual.ImageStim(win,image= path + '1.png', pos=(-0.66, 0.75), size=(0.5, 0.5))
block_2= visual.ImageStim(win,image= path + '2.png', pos=(0, 0.75), size=(0.5, 0.5))
block_3= visual.ImageStim(win,image= path + '3.png', pos=(0.66, 0.75), size=(0.5, 0.5))
block_4= visual.ImageStim(win,image= path + '4.png', pos=(-0.66, 0.25), size=(0.5, 0.5))
block_5= visual.ImageStim(win,image= path + '5.png', pos=(0, 0.25), size=(0.5, 0.5))
block_6= visual.ImageStim(win,image= path + '6.png', pos=(0.66, 0.25), size=(0.5, 0.5))
block_7= visual.ImageStim(win,image= path + '7.png', pos=(-0.66, -0.25), size=(0.5, 0.5))
block_8= visual.ImageStim(win,image= path + '8.png', pos=(0, -0.25), size=(0.5, 0.5))
block_9= visual.ImageStim(win,image= path + '9.png', pos=(0.66, -0.25), size=(0.5, 0.5))
block_0= visual.ImageStim(win,image= path + '0.png', pos=(0, -0.75), size=(0.5, 0.5))

frq =np.asarray([7,8,9,10,11,12,13,14,15,16])         #频率
block_1.autoDraw = True
block_2.autoDraw = True
block_3.autoDraw = True
block_4.autoDraw = True
block_5.autoDraw = True
block_6.autoDraw = True
block_7.autoDraw = True
block_8.autoDraw = True
block_9.autoDraw = True
block_0.autoDraw = True
while 1:
    if event.getKeys(keyList=['space','escape']):
        win.close()
    now_time = timer.getTime()
#    block_1.opacity =  np.sin(2 * np.pi * now_time* frq[1])/2+0.5     #靠透明度实现闪烁
    block_1.contrast = np.sin(2 * np.pi * now_time * frq[1]) / 2 + 0.5   #靠对比度实现闪烁
    block_2.contrast = np.sin(2 * np.pi * now_time * frq[2]) / 2 + 0.5
    block_3.contrast = np.sin(2 * np.pi * now_time * frq[3]) / 2 + 0.5
    block_4.contrast = np.sin(2 * np.pi * now_time * frq[4]) / 2 + 0.5
    block_5.contrast = np.sin(2 * np.pi * now_time * frq[5]) / 2 + 0.5
    block_6.contrast = np.sin(2 * np.pi * now_time * frq[6]) / 2 + 0.5
    block_7.contrast = np.sin(2 * np.pi * now_time * frq[7]) / 2 + 0.5
    block_8.contrast = np.sin(2 * np.pi * now_time * frq[8]) / 2 + 0.5
    block_9.contrast = np.sin(2 * np.pi * now_time * frq[9]) / 2 + 0.5
    block_0.contrast = np.sin(2 * np.pi * now_time * frq[0]) / 2 + 0.5
    win.flip()
