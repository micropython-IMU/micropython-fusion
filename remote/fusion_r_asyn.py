# fusion_r_asyn.py Test for sensor fusion remote device
# simulated by captured data mpudata
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2018 Peter Hinch
# Run under MicroPython on Unix or other target, or CPython 3.5 or later

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
if is_micropython:
    import gc

from fusion_async import Fusion

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

running = True
data = None
async def read_coro():
    global data, running
    await asyncio.sleep(0.02)
    try:
        data = next(get_data)
    except StopIteration:
        running = False
    return data  # Return old data if this overruns

fuse = Fusion(read_coro, True, TimeDiff)

async def mem_manage():
    while True:
        await asyncio.sleep(0.1)
        gc.collect()
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())

async def display():
    while running:
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
        await asyncio.sleep(0.5)

async def main_task():
    print('Calibrating: takes 15 secs.')
    await asyncio.sleep(0.1)
    await fuse.calibrate(lambda : not calibrate)
    print('Cal done:', fuse.magbias)
    await fuse.start()  # Start the update task
    await display()

loop = asyncio.get_event_loop()
if is_micropython:  # Task may be necessary on some uP platforms
    loop.create_task(mem_manage())
loop.run_until_complete(main_task())
