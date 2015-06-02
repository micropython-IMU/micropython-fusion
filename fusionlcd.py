# Fusionlcd.py Test for sensor fusion on Pyboard. Uses LCD display and threaded library.
# Author Peter Hinch
# V0.65 2nd June 2015

import pyb
from mpu9150 import MPU9150
from fusion import Fusion
from usched import Sched, wait, Poller
from lcdthread import LCD, PINLIST          # Library supporting Hitachi LCD module

"""
PINLIST based on a Hitachi HD44780 LCD wired with a 4-wire data bus as follows
Name LCD connector Board
Rs    4   1 red    Y1
E     6   2        Y2
D7   14   3        Y3
D6   13   4        Y4
D5   12   5        Y5
D4   11   6        Y6
"""
switch = pyb.Pin('Y7', pyb.Pin.IN, pull=pyb.Pin.PULL_UP) # Switch to ground on Y7

imu = MPU9150('X', 1, True)                # Attached to 'X' bus, 1 device, disable interruots
imu.gyro_range(0)
imu.accel_range(0)

fuse = Fusion()

def waitfunc():
    yield from wait(0.1)

def lcd_thread(mylcd, imu, sw):
    if sw.value() == 1:
        mylcd[0] = "Calibrate. Push switch"
        mylcd[1] = "when done"
        yield from wait(0.1)
        fuse.calibrate(imu.get_mag, lambda : not sw.value(), waitfunc)
        print(fuse.magbias)
    mylcd[0] = "{:5s}{:5s} {:5s}".format("Yaw","Pitch","Roll")
    count = 0
    while True:
        yield from wait(0.02) # IMU updates at 50Hz
        count += 1
        if count % 25 == 0:
            mylcd[1] = "{:4.0f} {:4.0f}  {:4.0f}".format(fuse.yaw, fuse.pitch, fuse.roll)
        fuse.update(imu.get_accel(), imu.get_gyro(), imu.get_mag())
# For 6DOF sensors
#        fuse.update_nomag(imu.get_accel(), imu.get_gyro())

objSched = Sched()
lcd0 = LCD(PINLIST, objSched, cols = 24) # Should work with 16 column LCD
objSched.add_thread(lcd_thread(lcd0, imu, switch))
objSched.run()
