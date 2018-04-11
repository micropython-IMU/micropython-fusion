# DeltaT instances, called with function call syntax, returns a time difference
# as a float value. Units seconds.
# If expect_ts is False it uses time.ticks_us as its time source. The default
# timediff function uses time.ticks_diff() with a division by 1e6.
# If expect_ts is True a non-native timestamp format is assumed and a nonstandard
# time differencing function is likely to be required.
# In this instance calls to Fusion.update() and Fusion.update_nomag() must be passed
# timestamps in the non-native format

import time

class DeltaT():
    def __init__(self, expect_ts, timediff):
        self.expect_ts = expect_ts
        self.timediff = timediff
        self.start_time = None

    def ensure_ready(self, ts):
        if self.start_time is None:
            self(ts)

    def __call__(self, ts):
        if self.expect_ts:
            if ts is None:
                raise ValueError('Timestamp expected but not supplied.')
        else:
            ts = time.ticks_us()
        dt = self.timediff(ts, self.start_time)
        self.start_time = ts
        return dt
