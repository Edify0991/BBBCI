from psychopy import visual,event
import numpy as np
import scipy.signal as signal

fps=60                     # 每秒刷新率
base_time=1000               # 基础时间1000ms
test_count=10                # 实验次数
test_time=base_time*test_count     #实验总时长
seq=[[0 for i in range(fps)] for i in range(10)]        #创建对于帧数的数组
win = visual.Window(size=(960, 1280), color=(0, 0, 0))
block=[0 for i in range(10)]
block[1]= visual.ImageStim(win, image='1.png', pos=(-0.66, 0.75), size=(0.5, 0.375))
block[2]= visual.ImageStim(win, image='2.png', pos=(0, 0.75), size=(0.5, 0.375))
block[3]= visual.ImageStim(win, image='3.png', pos=(0.66, 0.75), size=(0.5, 0.375))
block[4]= visual.ImageStim(win, image='4.png', pos=(-0.66, 0.25), size=(0.5, 0.375))
block[5]= visual.ImageStim(win, image='5.png', pos=(0, 0.25), size=(0.5, 0.375))
block[6]= visual.ImageStim(win, image='6.png', pos=(0.66, 0.25), size=(0.5, 0.375))
block[7]= visual.ImageStim(win, image='7.png', pos=(-0.66, -0.25), size=(0.5, 0.375))
block[8]= visual.ImageStim(win, image='8.png', pos=(0, -0.25), size=(0.5, 0.375))
block[9]= visual.ImageStim(win, image='9.png', pos=(0.66, -0.25), size=(0.5, 0.375))
block[0]= visual.ImageStim(win, image='0.png', pos=(0, -0.75), size=(0.5, 0.375))
rate=np.asarray([9,10,11,12,13,14,15,16,17,18])         #对应数字的刷新频率
n=np.arange(0,fps)
for i in range(10):
    seq[i] = signal.square(np.sin(2*np.pi*rate[i]*(n/fps)))*0.5+0.5      #利用signal.square将sin转换成周期性方波，取0或1，进而实现闪烁。
#for n in range(test_count):
while 1:
    for num in range(fps):
        if event.getKeys(keyList=['space','escape']):
            win.close()
        for i in range(10):
            if seq[i][num]:
                block[i].draw()
        win.flip()
win.close()
