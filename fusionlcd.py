import gc
from mpu9150 import MPU9150
from fusion import Fusion
from usched import Sched, wait, Poller
from lcdthread import LCD, PINLIST          # Library supporting Hitachi LCD module

imu = MPU9150('Y', 1, True)
imu.gyro_range(0)
imu.accel_range(0)

fuse = Fusion()

def makefilt(delta, T = None):              # Build a single pole IIR filter with time conatant T
    if T is None:                           # For testing
        alpha, beta = 0, 1
    else:
        alpha = 1 - delta/T
        beta = 1 - alpha
    output = 0
    def runfilt(inp):
        nonlocal output
        output = alpha * output + beta * inp
        return output
    return runfilt

gc.disable()
# In systems with a continuously running loop, to avoid occasional delays
# of around 2mS do a manual gc on each pass

def lcd_thread(mylcd, imu):
    mylcd[0] = "{:8s}{:8s}{:8s}".format("Yaw","Pitch","Roll")
    count = 0
    while True:
        yield from wait(0.02) # IMU updates at 50Hz
        count += 1
        if count % 25 == 0:
            yaw, pitch, roll = fuse.angles()
            mylcd[1] = "{:7.0f} {:7.0f} {:7.0f}".format(yaw, pitch, roll)
        fuse.Update(imu.get_accel(), imu.get_gyro(), imu.get_mag()) # Note blocking mag read
#        fuse.update_nomag(imu.get_accel(), imu.get_gyro())
        gc.collect()

objSched = Sched()
lcd0 = LCD(PINLIST, objSched, cols = 24)
objSched.add_thread(lcd_thread(lcd0, imu))
objSched.run()
gc.enable()
