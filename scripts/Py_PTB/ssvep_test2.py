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
test_num=0
fps=0
filename= ['0.png', '1.png', '2.png', '3.png', '4.png', '5.png', '6.png', '7.png', '8.png', '9.png', '10.png','11.png']
rate = np.asarray([7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])  # 对应数字的刷新频率
block = [0 for i in range(12)]  # 初始化
#=================================================================================================================================

def mstoframe(ms, fps):
    return np.round(ms * fps / 1000).astype(int)

# ================================================================================================================================

def value_init():
    random.seed()
    global test_num,fps,seq,n,tex,seq,win
    test_num = int(random.random() * 1000 % 12)  # 实验编号
    win = visual.Window(
                        size=(600, 800),
                        color=(255, 255, 255)
                        )  # 窗口
    Msperframe = win.getMsPerFrame()  # 刷新间隔
    fps = np.round(1000 / Msperframe[0]).astype(int)  # 每秒刷新率
    print("your screen's fps is ", fps)
    # 创建对于帧数的数组
    seq = [[0 for i in range(fps)] for i in range(12)]
    n = np.arange(0, fps)
    # 利用signal.square转换成周期性方波，取0或1，进而实现闪烁。
    for i in range(12):
        seq[i] = signal.square(2 * np.pi * rate[i] * (n / fps) + (i % 4) * 0.5 * np.pi) * 0.5 + 0.5
    #读取输出文件
    text = psychopy.tools.filetools.fromFile('a.psydat')
    #提取数据
    tex = text.getAllEntries()

# ========================================================================================================================
def image_init():
    global  rectangle,test_text,myDlg,block
    for i in range(12):
        block[i] = visual.ImageStim(win,
                                    image=filename[i],
                                    pos=(((i % 3) - 1) * 0.66, (0.75 - 0.5 * (i // 3))),
                                    size=(0.5, 0.375)
                                    )  # 创建图片信息
    rectangle = visual.Rect(win,
                            width=0.6, height=0.45,
                            lineWidth=2.5, lineColor='red', lineColorSpace='rgb',
                            fillColor=None,
                            pos=(((test_num % 3) - 1) * 0.66, (0.75 - 0.5 * (test_num // 3)))
                            )
    test_text = visual.TextStim(win,
                                'Please stare at the picture in red rectangle.',
                                color=(0, 0, 1),
                                colorSpace='rgb',
                                pos=(0, 0),
                                height=0.04
                                )
    #创建对话框显示数据
    myDlg = gui.Dlg(title='dataOut',labelButtonOK='Retest',labelButtonCancel='Cancel')

# =============================================================================================================================
while 1:
#============================================================================================================================
    value_init()
    image_init()
#================================================================================================================================

    #提示阶段
    for frame in range(mstoframe(wait_time,fps)):
        test_text.draw()
        win.flip()

    #标记阶段
    for frame in range(mstoframe(sensor_time,fps)):
        block[test_num].draw()
        rectangle.draw()
        win.flip()

    #2秒等待
    for frame in range(mstoframe(2000,fps)):
        for i in range(12):
            block[i].draw()
        rectangle.draw()
        win.flip()
    frame = 0

    #6s刺激时间
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

    win.close()
    #================================================================================================================================
    #数据输出
    for i in range(len(tex)):
        for data in tex[i].items():
            myDlg.addField(data[0],data[1])
    ok_data=myDlg.show()
    if not myDlg.OK:
        exit(0)
