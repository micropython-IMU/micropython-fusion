# Sensor Fusion Remote Mode

This caters for the case where a device acquires IMU data and returns that data
to another platform performing the fusion. The remote device may run any OS (or
none) and run any programming language so long as it returns the correct data.

The platform performing the fusion may run MicroPython. Alternatively CPython
(standard Python) may be used. For synchronous fusion the version must be 3.4
or better. For asynchronous fusion the version must be 3.5 or later.

The remote device must return a timestamp along with the IMU data, representing
the time when the IMU data was acquired. This avoids inaccuracy in the fusion
calculation caused by latency in the communications link. The format of the
timestamp is arbitrary, but (unless sourced by Micropython's `utime.ticks_us`)
the user program must provide a function for calculating the difference between
two timestamps. This is because timestamps may have different scaling and may
be modulo N (i.e. they may roll over).

# Files

 1. `mpudata` A set of data captured from a Pyboard performing calibration
 followed by movement around each axis in turn.
 2. `capture` The program used to create the above dataset.
 3. `fusion_r_syn` Synchronous test program.
 4. `fusion_r_asyn` Asynchronous test program.

# Remote program design

This needs to supply data at a rate dependent on the application. For fast
moving platforms this may need to be as fast as 10ms.

Format is user-dependent. If you have a choice consider a JSON encoded list of
form:

```
[[ax, ay, az], [gx, gy, gz], [mx, my, mz], timestamp]
```

where an, gn, mn are accelerometer, gyro, and magnetometer vectors. This
matches the call signature of the fusion code.

# Fusion program design (synchronous)

The `Fusion` constructor should be instantiated as follows:

```python
fuse = Fusion(True, TimeDiff)
```

The first arg tells it to expect a timestamp. The second is the user supplied
time differencing function. This arg may be omitted if the remote platform runs
MicroPython and the timestamp is derived from `utime.ticks_us()`.

The approach I have used is to encapsulate the data acquisition from the target
in a generator (`gdata` in the test program). This reads a line of data from
the target, decodes the JSON string, and returns a list of form

```
[[ax, ay, az], [gx, gy, gz], [mx, my, mz], timestamp]
```

The handling of the `StopIteration` exception is required because the data
source is a file of finite length.

If magnetometer calibration is to be used the fusion program needs some form of
input to tell it that rotation of the target about each axis is completed. In
the test program this is stored in the file.

# Fusion program design (asynchronous)

The `Fusion` constructor should be instantiated as follows:

```python
fuse = Fusion(read_coro, True, TimeDiff)
```

The first argument is a coroutine. This returns three (x, y, z) 3-tuples for
accelerometer, gyro, and magnetometer data respectively followed by a
timestamp. In the case of 6DOF sensors it returns two 3-tuples for
accelerometer and gyro followed by a timestamp. The coroutine must include at
least one `await asyncio.sleep_ms` statement to conform to Python syntax rules.

The second arg tells it to expect a timestamp. The third is the user supplied
time differencing function. This arg may be omitted if the remote platform runs
MicroPython and the timestamp is derived from `utime.ticks_us()`.

TBD: incomplete.
[back](../README.md)
