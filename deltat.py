# deltat.py time difference calculation for sensor fusion
# Released under the MIT License (MIT)
# Copyright (c) 2018 Peter Hinch

# Provides TimeDiff function and DeltaT class.
# The following notes cover special cases. Where the device performing fusion
# is linked to the IMU and is running MicroPython no special treatment is
# needed.
# The special cases are:
# 1. Device connected to the IMU is linked to a separate platform doing fusion.
# 2. Either or both are not running MicroPython.

# If the device providing the vectors is not running on MicroPython the user
# must supply timestamps and a function capable of differencing these. The
# function is passed to the Fusion constructor and the timestamp is provided
# along with the vector, being the time when the vector was acquired.

# If the device providing the vectors is running MicroPython but fusion is
# being performed on a device which is not, the user must provide their own
# implementation of ticks_diff which accounts for MicroPython rollover and
# must supply the returned ticks_us() values as a timestamp.

# Under MicroPython TimeDiff(start, end) uses time.ticks_diff.

# A DeltaT instance, called with function call syntax, returns a time
# difference from the previous call as a float value. Units seconds.

# If running under MicroPython and no time differencing function is supplied
# to the Fusion constructor it uses time.ticks_us as its time source and a
# default timediff function using time.ticks_diff() with a division by 1e6.
# If time differencing function is supplied a timestamp must be passsed as an
# arg to instance calls of Fusion.update() or Fusion.update_nomag(). In the
# async version the user supplied read_coro() must return a timestamp with the
# vector.

# On 1st pass dt evidently can't be computed. A notional value of 100μs is
# returned. The Madgwick algorithm takes seconds to stabilise.

try:
    import utime as time
except ImportError:
    import time

is_micropython = hasattr(time, 'ticks_diff')

class DeltaT():
    def __init__(self, timediff):
        if timediff is None:
            self.expect_ts = False
            if is_micropython:
                self.timediff = lambda start, end : time.ticks_diff(start, end)/1000000
            else:
                raise ValueError('You must define a timediff function')
        else:
            self.expect_ts = True
            self.timediff = timediff
        self.start_time = None

    def __call__(self, ts):
        if self.expect_ts:
            if ts is None:
                raise ValueError('Timestamp expected but not supplied.')
        else:
            if is_micropython:
                ts = time.ticks_us()
            else:
                raise RuntimeError('Not MicroPython: provide timestamps and a timediff function')
        # ts is now valid
        if self.start_time is None:  # 1st call: self.start_time is invalid
            self.start_time = ts
            return 0.0001  # 100μs notional delay. 1st reading is invalid in any case

        dt = self.timediff(ts, self.start_time)
        self.start_time = ts
        return dt
