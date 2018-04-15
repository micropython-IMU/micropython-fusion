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

intro = '''
This demo reads data from a file created by recording IMU data from a Pyboard
equipped with an IMU device. The data (in JSON format) contains the 3 IMU
vectors followed by a timestamp in μs.

Initially the board was rotated around each orthogonal axis to calibrate the
magnetometer. A special line was inserted into the file to signal the end of
cal.

The device was then partially rotated in heading, then pitch, then roll: this
data is displayed when calibration is complete.

Calibration takes 8 seconds.
'''

# gdata produces synthetic remote data from a file.
# In an application this would do a blocking data read from a remote device.
# The change in calibration mode might be returned from the remote as here or
# handled at the fusion device by user input.

# Initially we're in cal mode
calibrate = True
def gdata():
    global calibrate
    # Return [[ax, ay, az], [gx, gy, gz], [mx, my, mz], timestamp]
    # from whatever the device supplies (in this case JSON)
    with open('mpudata', 'r') as f:
        line = f.readline()  # An app would do a blocking read of remote data
        while line:
            if line.strip() == 'cal_end':
                calibrate = False
            else:
                yield json.loads(line)  # Convert foreign data format.
            line = f.readline()  # Blocking read.

get_data = gdata()

# Test of supplying a timediff
if is_micropython:
    def TimeDiff(start, end):
        return time.ticks_diff(start, end)/1000000
else:  # Cpython cheat: test data does not roll over
    def TimeDiff(start, end):
        return (start - end)/1000000

# Expect a timestamp. Use supplied differencing function.
fuse = Fusion(TimeDiff)

def getmag():  # Return (x, y, z) magnetometer vector.
    imudata = next(get_data)
    return imudata[2]

print(intro)
fuse.calibrate(getmag, lambda : not calibrate, lambda : time.sleep(0.01))
print('Cal done. Magnetometer bias vector:', fuse.magbias)
print('Heading    Pitch    Roll')

count = 0
while True:
    try:
        data = next(get_data)
    except StopIteration:  # A file is finite.
        break  # A real app would probably run forever.
    fuse.update(*data)
    if count % 25 == 0:
        print("{:8.3f} {:8.3f} {:8.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
    time.sleep(0.02)
    count += 1
