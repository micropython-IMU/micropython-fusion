# fusionlcd.py Test for asynchronous sensor fusion on Pyboard. Uses LCD display and uasyncio.
# Author: Peter Hinch
# Copyright Peter Hinch 2017 Released under the MIT license
# V0.8 13th May 2017 Adapted for uasyncio
# V0.7 25th June 2015 Adapted for new MPU9x50 interface

# Requires:
# uasyncio (official or modified version)
# MPU9150 on X position
# Normally open pushbutton connected between pin Y7 and ground
# LCD driver alcd.py from https://github.com/peterhinch/micropython-async.git
# Hitachi HD44780 2 row LCD display wired using 4 bit data bus as follows:

# Name LCD connector Board
# Rs    4   1 red    Y1
# E     6   2        Y2
# D7   14   3        Y3
# D6   13   4        Y4
# D5   12   5        Y5
# D4   11   6        Y6

from machine import Pin
import uasyncio as asyncio
import gc
from mpu9150 import MPU9150
from fusion_async import Fusion # Using async version
from alcd import LCD, PINLIST   # Library supporting Hitachi LCD module

switch = Pin('Y7', Pin.IN, pull=Pin.PULL_UP) # Switch to ground on Y7

imu = MPU9150('X')              # Attached to 'X' bus, 1 device, disable interruots

fuse = Fusion()
lcd = LCD(PINLIST, cols = 24)   # Should work with 16 column LCD

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
    lcd[0] = "{:5s}{:5s} {:5s}".format("Yaw","Pitch","Roll")
    while True:
        lcd[1] = "{:4.0f} {:4.0f}  {:4.0f}".format(fuse.heading, fuse.pitch, fuse.roll)
        await asyncio.sleep_ms(500)

async def lcd_task(fusion, sw):
    if sw.value() == 1:
        lcd[0] = "Calibrate. Push switch"
        lcd[1] = "when done"
        await asyncio.sleep_ms(100)  # Let LCD coro run
        await fusion.calibrate(mag, lambda : not sw.value(), 100)
        print(fuse.magbias)
    loop = asyncio.get_event_loop()
    loop.create_task(display())
    loop.create_task(fusion.update(accel, gyro, mag, 20))

# For 6DOF sensors
#    loop.create_task(fusion.update_nomag(accel, gyro, 20)


loop = asyncio.get_event_loop()
loop.create_task(mem_manage())
loop.create_task(lcd_task(fuse, switch))
loop.run_forever()
