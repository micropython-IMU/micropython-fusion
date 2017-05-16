# fusiontest_as.py Test for asynchronous sensor fusion on Pyboard.
# Author: Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2017 Peter Hinch

# Requires:
# uasyncio (official or modified version)
# MPU9150 on X position
# Normally open pushbutton connected between pin Y7 and ground

from machine import Pin
import uasyncio as asyncio
import gc
from mpu9150 import MPU9150
from fusion_async import Fusion # Using async version

switch = Pin('Y7', Pin.IN, pull=Pin.PULL_UP) # Switch to ground on Y7

imu = MPU9150('X')              # Attached to 'X' bus, 1 device, disable interruots

fuse = Fusion()

def mag():                      # Return (x, y, z) tuple
    return imu.mag_nonblocking.xyz

def accel():
    return imu.accel.xyz

def gyro():
    return imu.gyro.xyz

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

async def lcd_task(fusion, sw):
    if sw.value() == 1:
        print('Calibrate. Push switch when done.')
        await fusion.calibrate(mag, lambda : not sw.value(), 100)
        print('Mag bias vector: ', fuse.magbias)
    loop = asyncio.get_event_loop()
    loop.create_task(display())
    loop.create_task(fusion.update(accel, gyro, mag, 20))

# For 6DOF sensors
#    loop.create_task(fusion.update_nomag(accel, gyro, 20)


loop = asyncio.get_event_loop()
loop.create_task(mem_manage())
loop.create_task(lcd_task(fuse, switch))
loop.run_forever()
