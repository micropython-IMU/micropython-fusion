# fusiontest6.py Simple test program for 6DOF sensor fusion on Pyboard
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2017 Peter Hinch
# V0.8 14th May 2017 Option for external switch for cal test. Make platform independent.
# V0.7 25th June 2015 Adapted for new MPU9x50 interface

from machine import Pin
import utime as time
from mpu9150 import MPU9150
from fusion import Fusion

imu = MPU9150('X')

fuse = Fusion()

# Choose test to run
Timing = True

if Timing:
    accel = imu.accel.xyz
    gyro = imu.gyro.xyz
    start = time.ticks_us()  # Measure computation time only
    fuse.update_nomag(accel, gyro) # 979μs on Pyboard
    t = time.ticks_diff(time.ticks_us(), start)
    print("Update time (uS):", t)

count = 0
while True:
    fuse.update_nomag(imu.accel.xyz, imu.gyro.xyz)
    if count % 50 == 0:
        print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
    time.sleep_ms(20)
    count += 1
