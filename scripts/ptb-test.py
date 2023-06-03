import psychopy.visual
import psychopy.event

win = psychopy.visual.Window(
    size=[400, 400],
    units="pix",
    fullscr=False
)

rect = psychopy.visual.Rect(
    win=win,
    width=200,
    height=100,
    pos=[0,-50],
    fillColor=[1] * 3
)

rect.draw()

img = psychopy.visual.ImageStim(
    win=win,
    image="UNSW.png",
    units="pix"
)

img.opacity = 0.5

img.draw()

win.flip()

psychopy.event.waitKeys()