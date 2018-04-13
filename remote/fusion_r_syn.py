# fusion_r_syn.py Test for sensor fusion remote device
# simulated by captured data mpudata
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2018 Peter Hinch
# Run under MicroPython on Unix or other target, or CPython 3.4 or later

try:
    import utime as time
except ImportError:
    import time
try:
    import ujson as json
except ImportError:
    import json
from fusion import Fusion
from deltat import is_micropython

# Generator produces synthetic remote data from a file.
# Initially we're in cal mode
calibrate = True
def gdata():
    global calibrate
    with open('mpudata', 'r') as f:
        line = f.readline()
        while line:
            if line.strip() == 'cal_end':
                calibrate = False
            else:
                yield json.loads(line)
            line = f.readline()

get_data = gdata()

# Test of supplying a timediff
if is_micropython:
    def TimeDiff(start, end):
        return time.ticks_diff(start, end)/1000000
else:  # Cpython cheat: test data does not roll over
    def TimeDiff(start, end):
        return (start - end)/1000000

# Expect a timestamp. Use supplied differencing function
fuse = Fusion(True, TimeDiff)

def getmag():  # Return (x, y, z) tuple of magnetometer
    imudata = next(get_data)
    return imudata[2]

print("Calibrating. Takes 8 secs.")
fuse.calibrate(getmag, lambda : not calibrate, lambda : time.sleep(0.01))
print('Calibration done. Bias:', fuse.magbias)

count = 0
while True:
    try:
        data = next(get_data)
    except StopIteration:
        break
    fuse.update(*data)
    if count % 25 == 0:
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
    time.sleep(0.02)
    count += 1
