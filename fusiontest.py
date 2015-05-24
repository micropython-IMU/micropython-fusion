import pyb
from mpu9150 import MPU9150
from fusion import Fusion

imu = MPU9150('Y', 1, True)
imu.gyro_range(0)
imu.accel_range(0)

fuse = Fusion()

Calibrate = False
Timing = True

if Calibrate:
    print("Calibrating. Press switch when done.")
    sw = pyb.Switch()
    fuse.calibrate(imu.get_mag, sw)
    print(fuse.magbias)

if Timing:
    mag = imu.get_mag() # Don't include blocking read in time
    accel = imu.get_accel() # or i2c
    gyro = imu.get_gyro()
    start = pyb.micros()
    fuse.update(accel, gyro, mag) # 1.65mS on Pyboard
    t = pyb.elapsed_micros(start)
    print("Update time (uS):", t)

count = 0
while True:
    pyb.delay(20)
    count += 1
    if count % 50 == 0:
        print("Yaw, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.yaw, fuse.pitch, fuse.roll))
    fuse.update(imu.get_accel(), imu.get_gyro(), imu.get_mag()) # Note blocking mag read
