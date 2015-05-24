import pyb
from mpu9150 import MPU9150
from fusion import Fusion
from usched import Sched, wait, Poller
from lcdthread import LCD, PINLIST          # Library supporting Hitachi LCD module

imu = MPU9150('Y', 1, True)
imu.gyro_range(0)
imu.accel_range(0)

fuse = Fusion()

Calibrate = False

def scale(func, template, *vecs):
    res = []
    for v in vecs:
        res.append(tuple(map(func, template, v)))
    return res

rot = (-1, 1, 1)

def lcd_thread(mylcd, imu):
    if Calibrate:
        sw = pyb.Switch()
        mylcd[0] = "Calibrate. Push button"
        mylcd[1] = "when done"
        yield from wait(0.1)
        fuse.calibrate(imu.get_mag, sw)
        print(fuse.magbias)
    mylcd[0] = "{:8s}{:8s}{:8s}".format("Yaw","Pitch","Roll")
    count = 0
    while True:
        yield from wait(0.02) # IMU updates at 50Hz
        count += 1
        if count % 25 == 0:
            mylcd[1] = "{:7.0f} {:7.0f} {:7.0f}".format(fuse.yaw, fuse.pitch, fuse.roll)
#        vectors = scale(lambda x, y: x*y, rot, imu.get_mag())
        fuse.update(imu.get_accel(), imu.get_gyro(), imu.get_mag())
#        fuse.update(imu.get_accel(), imu.get_gyro(), (mx, my, mz))
#        fuse.update_nomag(imu.get_accel(), imu.get_gyro())

objSched = Sched()
lcd0 = LCD(PINLIST, objSched, cols = 24)
objSched.add_thread(lcd_thread(lcd0, imu))
objSched.run()
