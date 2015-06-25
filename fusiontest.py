# Simple test program for sensor fusion on Pyboard
# Author Peter Hinch
# V0.7 25th June 2015 Adapted for new MPU9x50 interface

import pyb
from mpu9150 import MPU9150
from fusion import Fusion

imu = MPU9150('X')

fuse = Fusion()

# Choose test to run
Calibrate = True
Timing = False

def getmag():                               # Return (x, y, z) tuple (blocking read)
    return imu.mag.xyz

if Calibrate:
    print("Calibrating. Press switch when done.")
    sw = pyb.Switch()
    fuse.calibrate(getmag, sw, lambda : pyb.delay(100))
    print(fuse.magbias)

if Timing:
    mag = imu.mag.xyz # Don't include blocking read in time
    accel = imu.accel.xyz # or i2c
    gyro = imu.gyro.xyz
    start = pyb.micros()
    fuse.update(accel, gyro, mag) # 1.65mS on Pyboard
    t = pyb.elapsed_micros(start)
    print("Update time (uS):", t)

count = 0
while True:
    fuse.update(imu.accel.xyz, imu.gyro.xyz, imu.mag.xyz) # Note blocking mag read
    if count % 50 == 0:
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
    pyb.delay(20)
    count += 1
