# 1. Sensor Fusion on generic devices

This library was written for MicroPython but can be used on any device capable
of running Python V3.4 or above (V3.5 or above for the asynchronous version).
In its default mode as described in the [main README](../README.md) it uses
specific MicroPython calls to handle precision time values.

On other platforms the application must construct a `Fusion` instance in a way
to prevent it doing this. The application must provide a precision time value
representing the instant that the vectors were acquired. It must also provide a
function capable of differencing a pair of these; this is typically merely
subtraction and scaling.

[Main README](../README.md)

# 2. Files

 1. `mpudata` A set of data captured from a Pyboard performing calibration
 followed by movement around each axis in turn.
 2. `capture` The program used to create the above dataset.
 3. `fusion_r_syn` Synchronous test program using the dataset.
 4. `fusion_r_asyn` Asynchronous test program using the dataset.
 
The test programs perform a calibration phase during which the device was fully
rotated around each orthogonal axis. They then display the data as the device
was partially rotated around each axis in turn.

Test programs include comments indicating how synchronous and asynchronous
applications might be developed.

# 3. Remote Mode

This caters for the case where a device acquires IMU data and returns that data
to another platform performing the fusion. The remote device may run any OS (or
none), run any programming language and return data in any format. It must
return two or three vectors (depending on whether it is a 6DOF or 9DOF device)
and a timestamp indicating when the vectors were acquired.

The platform performing the fusion may run any Python version as described in
[section 1](./README.md#1-sensor-fusion-on-generic-devices).

The remote device must return a timestamp along with the IMU data, representing
the time when the IMU data was acquired. This avoids inaccuracy in the fusion
calculation caused by latency in the communications link. The format of the
timestamp is arbitrary, but the user program must provide a function for
calculating the difference between two timestamps.

User code on the device performing the fusion must convert the data received
from the remote to the format acceptable to the fusion module, namely

```python
[[ax, ay, az], [gx, gy, gz], [mx, my, mz], timestamp]
```

where an, gn, mn are accelerometer, gyro, and magnetometer vectors. The axes
of these readings must match.

## 3.1 Remote program design

This needs to supply data at a rate dependent on the application. To track fast
moving platforms the repetition rate may need to be as fast as 10ms.

Format is user-dependent. If you have a choice consider a JSON encoded list as
detailed above: since this matches the call signature of the fusion code it is
trivial to convert.

# 4. Fusion application design

## 4.1 Timestamps

The library requires a function which accepts as arguments two timestamps and
returns a time difference in seconds as a floating point value. In the case
where the acquisition and fusion devices both run MicroPython, the acquisition
device can transmit values from `utime.ticks_us()` and the differnencing
function passed to the Fusion constructor would be

```python
lambda start, end: utime.ticks_diff(start, end)/1000000
```

In other cases if timestamps roll over modulo N, the differencing function must
accommodate this. In more usual case where times are floating point values
without rollover it may simply be a matter of scaling:

```python
def TimeDiff(start, end):  # Timestamps here are in μs
    return (start - end)/1000000  # Scale to seconds
```

If standard Python's `time.time()` provides the timestamps no scaling is
required. The function can reduce to:

```python
lambda start, end: start-end
```

## 4.2 Calibration

If magnetometer calibration is to be used the fusion program needs some form of
input to tell it that manual rotation of the target about each axis is
complete.

In the case of remote applications this might be sourced by user input to the
remote or to the fusion device. In the test data a button on the remote was
pressed and this is stored in the file, simulating communication with the host.

## 4.3 Synchronous applications

The `Fusion` constructor should be instantiated as follows:

```python
from fusion import Fusion
fuse = Fusion(TimeDiff)
```

The arg is the user supplied time differencing function.

For details of the method of updating the fusion data and retrieving the angles
see the [main document section 2](../README.md#2-fusion-module).

### 4.3.1 Test program fusion_r_syn.py

This encapsulates data acquisition from the target in the generator function
`gdata`. The resultant generator `get_data` reads a line of data from the
target, decodes the JSON string, and yields a list of form

```
[[ax, ay, az], [gx, gy, gz], [mx, my, mz], timestamp]
```

Code is included to handle the calibration mode: in the test file completion is
flagged by a special record created when a button on the device was pressed.
The handling of the `StopIteration` exception is required because the data
source is a file of finite length.

## 4.4 Asynchronous applications

The `Fusion` constructor should be instantiated as follows:

```python
from fusion_async import Fusion
fuse = Fusion(read_coro, TimeDiff)
```

The first argument is a coroutine. For 9DOF sensors this must be designed to
return three (x, y, z) 3-tuples for accelerometer, gyro, and magnetometer data
respectively followed by a timestamp. In the case of 6DOF sensors it returns
two 3-tuples for accelerometer and gyro followed by a timestamp. The coroutine
must include at least one `await` statement to conform to Python syntax rules.

The second arg is the user supplied time differencing function.

For details of the method of updating the fusion data and retrieving the angles
see the [main document section 3](../README.md#3-asynchronous-version).

### 4.4.1 Test program fusion_r_asyn.py

This encapsulates data acquisition from the target in the class `GetData`. In
principle the constructor initiates communications with the (local or remote)
IMU. The asynchronous `read` method acquires a record from the IMU, converts it
to the correct format, and returns a list of form

```
[[ax, ay, az], [gx, gy, gz], [mx, my, mz], timestamp]
```

Code is included to handle the calibration mode: in the test file completion is
flagged by a special record created when a button on the device was pressed.
Further code handles the fact that the test fileis of finite length.

[Main README](../README.md)
