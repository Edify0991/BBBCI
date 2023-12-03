import random
import numpy as np
import scipy.signal as signal
from psychopy import visual, event
import psychopy.tools.filetools
from psychopy import gui

base_time = 1000  # 基础时间1000ms
test_count = 10  # 实验次数
test_time = 2000  # 实验时长
wait_time = 3000  # 准备时间
sensor_time = 2000  # 预热时间
fps=0
filename= ['-.png', '1.png', '2.png', '3.png', '4.png', '5.png', '6.png', '7.png', '8.png', '9.png', '+.png']
filename2= ['-_-.png', '1_1.png', '2_2.png', '3_3.png', '4_4.png', '5_5.png', '6_6.png', '7_7.png', '8_8.png', '9_9.png', '+_+.png']
pos=[(0.75,0.66),(-0.75,-0.66),(-0.25,-0.66),(0.25,-0.66),(-0.75,0),(-0.25,0),(0.25 ,0),(-0.75,0.66),(-0.25,0.66),(0.25,0.66),(0.75,-0.66)]
rate = np.asarray([18, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17])  # 对应数字的刷新频率
block = [0 for i in range(11)]  # 初始化
block1 = [0 for i in range(11)]
#=================================================================================================================================
def mstoframe(ms, fps):
    return np.round(ms * fps / 1000).astype(int)
# ================================================================================================================================

def value_init():
    random.seed()
    global fps,seq,n,seq,win
    win = visual.Window(
                        size=(800, 600),
                        color=(255, 255, 255)
                        )  # 窗口
    Msperframe = win.getMsPerFrame()  # 刷新间隔
    fps = np.round(1000 / Msperframe[0]).astype(int)  # 每秒刷新率
    print("your screen's fps is ", fps)
    # 创建对于帧数的数组
    seq = [[0 for i in range(fps)] for i in range(12)]
    n = np.arange(0, fps)
    # 利用signal.square转换成周期性方波，取0或1，进而实现闪烁。
    for i in range(11):
        seq[i] = signal.square(2 * np.pi * rate[i] * (n / fps) + (i % 4) * 0.5 * np.pi) * 0.5 + 0.5

# ========================================================================================================================
def image_init():
    global block
    for i in range(11):
        block[i] = visual.ImageStim(win,
                                    image=filename[i],
                                    pos=pos[i],
                                    size=(0.375, 0.5),
                                    )  # 创建图片信息
        block1[i] = visual.ImageStim(win,
                                    image=filename2[i],
                                    pos=pos[i],
                                    size=(0.375, 0.5),
                                    )  # 创建图片信息

def rect(test_num):
    global rectangle
    rectangle = visual.Rect(win,
                            width=0.45, height=0.6,
                            lineWidth=2.5, lineColor='red', lineColorSpace='rgb',
                            fillColor=None,
                            pos=pos[test_num]
                            )

def enter_to_test(ch):
    if ch[len(ch)-1] == 't':
        return 0
    elif ch[len(ch)-1] == 'd':
        return 10
    elif ord(ch[len(ch)-1])>=ord('0')and ord(ch[len(ch)-1])<=ord('9'):
        return ord(ch[len(ch)-1])-ord('0')
    else:
        print('Unexpected key')
        exit(0)

# =============================================================================================================================
value_init()
while 1:
#============================================================================================================================
    image_init()
    for frame in range(mstoframe(2000,fps)):
        for i in range(11):
            block[i].draw()
        win.flip()
    for frame in range(mstoframe(14,fps)):
        for i in range(11):
            block1[i].draw()
        win.flip()
    enter_num = event.waitKeys()[0]
    rect(enter_to_test(enter_num))
    for i in range(11):
        block[i].opacity=1
    for frame in range(mstoframe(1000, fps)):
        for i in range(11):
            block[i].draw()
        rectangle.draw()
        win.flip()
#================================================================================================================================
    frame = 0
    #6s刺激时间
    while frame <= mstoframe(test_time, fps):
        for num in range(fps):
            if event.getKeys(keyList=['space', 'escape']):
                win.close()
            for i in range(11):
                if seq[i][num]:
                    block[i].draw()
            rectangle.draw()
            win.flip()
        frame = frame + fps
    #================================================================================================================================
    #数据输出
