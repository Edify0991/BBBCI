import numpy as np
import scipy.signal as signal
from psychopy import visual, event

base_time = 1000  # 基础时间1000ms
test_count = 10  # 实验次数
test_time = base_time * test_count  # 实验总时长
win = visual.Window(size=(960, 1280), color=(0, 0, 0))
Msperframe = win.getMsPerFrame()
fps = np.round(1000 / Msperframe[0]).astype(int)  # 每秒刷新率
print("your screen's fps is ", fps)
seq = [[0 for i in range(fps)] for i in range(10)]  # 创建对于帧数的数组
block = [0 for i in range(10)]
filename = ['0.png', '1.png', '2.png', '3.png', '4.png', '5.png', '6.png', '7.png', '8.png', '9.png', '0.png']
for i in range(9):
    block[i] = visual.ImageStim(win, image=filename[i], pos=(((i % 3) - 1) * 0.66, (0.75 - 0.5 * (i // 3))), size=(0.5, 0.375))
block[9] = visual.ImageStim(win, image=filename[9], pos=(0, -0.75), size=(0.5, 0.375))
rate = np.asarray([9, 10, 11, 12, 13, 14, 15, 16, 17, 18])  # 对应数字的刷新频率
n = np.arange(0, fps)
for i in range(10):
    seq[i] = signal.square(2 * np.pi * rate[i] * (n / fps)) * 0.5 + 0.5  # 利用signvl.square转换成周期性方波，取0或1，进而实现闪烁。
# for n in range(test_count):
while 1:
    for num in range(fps):
        if event.getKeys(keyList=['space', 'escape']):
            win.close()
        for i in range(10):
            if seq[i][num]:
                block[i].draw()
        win.flip()
win.close()
