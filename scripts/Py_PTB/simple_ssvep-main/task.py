# Simple SSVEP offline paradigm
#
# Created........: 16Mar2023 [ollie-d]
# Last Modified..: 16Mar2023 [ollie-d]

from math import atan2, degrees
import scipy.signal as signal
import psychopy.visual
import psychopy.event
import time
import datetime
import numpy as np
import pylsl
import random

# Global variables
win = None # Global variable for window (Initialized in main)
mrkstream = None # Global variable for LSL marker stream (Initialized in main)
photosensor = None # Global variable for photosensor (Initialized in main)
triangle = None # Global variable for stimulus (Initialized in main)
fixation = None # Global variable for fixation cross (Initialized in main)
bg_color = [-1, -1, -1]
win_w = 1920
win_h = 1080
refresh_rate = 165. # Monitor refresh rate (CRITICAL FOR TIMING)


#========================================================
# High Level Functions
#========================================================
def Paradigm(rate=10.0, flash_len=5000, num_trials=5):
    '''
        rate: SSVEP rate 
        flash_len: length in ms flashes should occur
        num_trials: number of repetitions
    '''
    
    # Initialize our main flashing hex
    size = 50
    hex = psychopy.visual.Circle(
            win=win,
            units="pix",
            radius=size*2,
            fillColor=[1, 1, 1],
            lineColor=bg_color,
            lineWidth = 1,
            edges = 6,
            pos = (0, 0)
        )
       
    # Initialize photosensor region
    photo = psychopy.visual.Circle(
            win=win,
            units="pix",
            radius=size,
            fillColor=[1, 1, 1],
            lineColor=bg_color,
            lineWidth = 1,
            edges = 32,
            pos = ((win_w / 2) - size, -((win_h / 2) - size))
        )
   
    # Create our flashing sequence
    seq = ssvep_sequence(rate, fs=refresh_rate)
    
    # Determine how many frames our flash_len is
    num_frames = MsToFrames(flash_len, refresh_rate)
    
    # Repeat our sequence int(num_frames / fs) times and only keep num_frames
    long_seq = listFlatten([seq]*np.ceil(num_frames/refresh_rate).astype(int))
   
   
    # 2000ms darkness before trials start
    for frame in range(MsToFrames(2000, refresh_rate)):
        if frame == 0:
            mrkstream.push_sample(pylsl.vectorstr(['10']));
        win.flip()
        
    # 2000ms photosensor on before trials start
    for frame in range(MsToFrames(2000, refresh_rate)):
        if frame == 0:
            mrkstream.push_sample(pylsl.vectorstr(['11']));
        photo.draw()
        win.flip()
        
    # 3000 ms darkness
    for frame in range(MsToFrames(2000, refresh_rate)):
        win.flip()
    
    # Loop through trials
    for trial in range(num_trials):
        # Flash SSVEP at rate
        start_time = datetime.datetime.now()
        for frame, state in enumerate(long_seq):
        #for frame, state in enumerate(long_seq):
        #for frame in range(MsToFrames(500, refresh_rate)):
            # Send LSL marker on first frame
            if frame == 0:
                mrkstream.push_sample(pylsl.vectorstr(['1']));
            if state:
                hex.draw()
            photo.draw()
            win.flip()
        
        end_time = datetime.datetime.now()
        time_diff = (end_time - start_time)
        execution_time = time_diff.total_seconds() * 1000
        print(execution_time)
            
        # 750-1000ms darkness
        for frame in range(MsToFrames(random.randint(750, 1000), refresh_rate)):
            if frame == 0:
                mrkstream.push_sample(pylsl.vectorstr(['0']));
            win.flip()

#========================================================
# Low Level Functions
#========================================================
def ssvep_sequence(freq, fs=refresh_rate):
    indices = np.arange(0, fs)
    y = signal.square(2 * np.pi * freq * (indices / fs))
    return (y + 1)/2

def MsToFrames(ms, fs):
    dt = 1000 / fs;
    return np.round(ms / dt).astype(int);

def DegToPix(h, d, r, deg):
    # Source: https://osdoc.cogsci.nl/3.2/visualangle/
    deg_per_px = degrees(atan2(.5*h, d)) / (.5*r)
    size_in_px = deg / deg_per_px
    return size_in_px

def listFlatten(df):
    t = []
    for i in range(len(df)):
        for j in range(len(df[i])):
            t.append(df[i][j])
    return t

def CreateMrkStream(name):
    info = pylsl.stream_info(name, 'Markers', 1, 0, pylsl.cf_string, 'unsampledStream');
    outlet = pylsl.stream_outlet(info, 1, 1)
    return outlet;

if __name__ == "__main__":
    random.seed()
    mrkstream = CreateMrkStream('SSVEP_Markers');
    print('Marker stream created, go to LabRecorder')
    time.sleep(10) # give time to synchronize LSL streams with lab recorder

    # Create PsychoPy window
    win = psychopy.visual.Window(
        screen = 0,
        size=[win_w, win_h],
        units="pix",
        fullscr=False,
        color=bg_color,
        gammaErrorPolicy = "ignore"
    );

    # Run through paradigm
    Paradigm(rate=10.0, flash_len=5000, num_trials=5)
    