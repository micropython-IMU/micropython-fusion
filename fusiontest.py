import pyb
from mpu9150 import MPU9150fusion
import micropython
import gc

imu = MPU9150fusion('Y', 1, True)
imu.gyro_range(0)
imu.accel_range(0)

count = 0
start = pyb.micros()
gc.disable()
# In systems with a continuously running loop, to avoid occasional delays
# of around 2mS do a manual gc on each pass
while True:
    pyb.delay(20)
    count += 1
    if count % 50 == 0:
        yaw, pitch, roll = imu.angles()
        print("Yaw, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(yaw, pitch, roll))
    end = pyb.elapsed_micros(start)
    start = pyb.micros()
    imu.update((end-start) / 1000000)
    gc.collect()

gc.enable()
