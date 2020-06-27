# fusion_r_asyn.py Test for sensor fusion remote device
# simulated by captured data mpudata
# Author Peter Hinch
# Released under the MIT License (MIT) See LICENSE
# Copyright (c) 2017-2020 Peter Hinch

# Requires:
# uasyncio V3 (Included in daily builds and release builds later than V1.12).
# Run under MicroPython on Unix or other target, or CPython 3.8 or later

try:
    import utime as time
except ImportError:
    import time
try:
    import ujson as json
except ImportError:
    import json
try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

from deltat import is_micropython
from fusion_async import Fusion

intro = '''
This demo reads data from a file created by recording IMU data from a Pyboard
equipped with an IMU device. The data (in JSON format) contains the 3 IMU
vectors followed by a timestamp in μs.

Initially the board was rotated around each orthogonal axis to calibrate the
magnetometer. A special line was inserted into the file to signal the end of
cal.

The device was then partially rotated in heading, then pitch, then roll: this
data is displayed when calibration is complete.

Calibration takes 15 seconds.
'''
# GetData produces synthetic remote data from a file.
# In an application this would asynchronously read data from a remote device.
# The change in calibration mode might be returned from the remote as here or
# handled at the fusion device by user input.

# Initially we're in cal mode
calibrate = True
class GetData:
    def __init__(self):
        self.f = open('mpudata', 'r')  # Initialise the comms link
        self.data = None
        # Because we're reading from a finite file we need a shutdown mechanism.
        # Would probably not apply to a comms link.
        self.running = True

    # Return [[ax, ay, az], [gx, gy, gz], [mx, my, mz], timestamp]
    # from whatever the device supplies (in this case JSON)
    async def read(self):
        global calibrate
        if not self.running:
            await asyncio.sleep(0.02)
            return self.data
        await asyncio.sleep(0.02)  # Simulate async read data from remote
        line = self.f.readline()
        if line:  # The file is finite
            if line.strip() == 'cal_end':  # Remote has signalled end of cal
                calibrate = False
                await asyncio.sleep(0.02)  # Read data from remote
                line = self.f.readline()

            self.data = json.loads(line)  # Convert from foreign format
        else:
            self.running = False
            self.f.close()
        return self.data  # Return old data if running has just been cleared

get_data = GetData()

# Test of supplying a timediff
if is_micropython:
    def TimeDiff(start, end):
        return time.ticks_diff(start, end)/1000000
else:  # Cpython cheat: test data does not roll over
    def TimeDiff(start, end):
        return (start - end)/1000000

fuse = Fusion(get_data.read, TimeDiff)

async def display():
    print('Heading    Pitch    Roll')
    while get_data.running:
        print("{:8.3f} {:8.3f} {:8.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
        await asyncio.sleep(0.5)

async def main_task():
    print(intro)
    await fuse.calibrate(lambda : not calibrate)
    print('Cal done. Magnetometer bias vector:', fuse.magbias)
    await fuse.start()  # Start the update task
    await display()

asyncio.run(main_task())
