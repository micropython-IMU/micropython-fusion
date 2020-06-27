# fusiontest_as.py Test for asynchronous sensor fusion on Pyboard.
# Author: Peter Hinch
# Released under the MIT License (MIT) See LICENSE
# Copyright (c) 2017-2020 Peter Hinch

# Requires:
# uasyncio V3 (Included in daily builds and release builds later than V1.12).
# From https://github.com/micropython-IMU/micropython-mpu9x50:
# imu.py, mpu9150.py, vector3d.py
# From this repo: deltat.py fusion_async.py

# MPU9150 on X position
# Normally open pushbutton connected between pin Y7 and ground

from machine import Pin
import uasyncio as asyncio
import gc
from mpu9150 import MPU9150
from fusion_async import Fusion # Using async version

switch = Pin('Y7', Pin.IN, pull=Pin.PULL_UP) # Switch to ground on Y7

imu = MPU9150('X')              # Attached to 'X' bus, 1 device, disable interrupts

# User coro returns data and determines update rate.
# For 9DOF sensors returns three 3-tuples (x, y, z) for accel, gyro and mag
async def read_coro():
    imu.mag_trigger()
    await asyncio.sleep_ms(20)  # Plenty of time for mag to be ready
    return imu.accel.xyz, imu.gyro.xyz, imu.mag_nonblocking.xyz

fuse = Fusion(read_coro)

async def mem_manage():         # Necessary for long term stability
    while True:
        await asyncio.sleep_ms(100)
        gc.collect()
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())

async def display():
    fs = 'Heading: {:4.0f} Pitch: {:4.0f} Roll: {:4.0f}'
    while True:
        print(fs.format(fuse.heading, fuse.pitch, fuse.roll))
        await asyncio.sleep_ms(500)

async def test_task():
    if switch.value() == 1:
        print('Calibrate. Push switch when done.')
        await fuse.calibrate(lambda : not switch.value())
        print('Mag bias vector: ', fuse.magbias)
    await fuse.start()  # Start the update task
    await display()


asyncio.create_task(mem_manage())
asyncio.run(test_task())
