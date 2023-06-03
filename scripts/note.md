1.fullscr为true时，会显示全屏，在测试时置为false即可以小窗口形式检查代码正确性
2.坐标关系：右上为正，左下为负
3.psychopy.misc.pol2cart函数可以将极坐标转换为笛卡尔坐标
4.keys = psychopy.event.waitKeys()

print keys
可使用该命令查看所按键的string是什么
5.wrapWidth 设置文本行一行最多输入的像素数