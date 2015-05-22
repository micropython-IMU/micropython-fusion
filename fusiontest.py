import pyb
from mpu9150 import MPU9150
from fusion import Fusion

imu = MPU9150('Y', 1, True)
imu.gyro_range(0)
imu.accel_range(0)

fuse = Fusion()

Calibrate = False

if Calibrate:
    print("Calibrating. Press switch when done.")
    sw = pyb.Switch()
    fuse.calibrate(imu.get_mag, sw)
    print(fuse.magbias)

count = 0
while True:
    pyb.delay(20)
    count += 1
    if count % 50 == 0:
        print("Yaw, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.yaw, fuse.pitch, fuse.roll))
    fuse.update(imu.get_accel(), imu.get_gyro(), imu.get_mag()) # Note blocking mag read
