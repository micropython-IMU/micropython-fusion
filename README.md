# Introduction: micropython-fusion

Sensor fusion calculates heading, pitch and roll from the outputs of motion
tracking devices. This uses the Madgwick algorithm, widely used in multicopter
designs for its speed and quality. An update takes under 2mS on the Pyboard.
The original Madgwick study indicated that an update rate of 10-50Hz was
adequate for accurate results, suggesting that the performance of this
implementation is fast enough.

Two implementations are provided: one for synchronous code and one for
asynchronous applications based on `asyncio`. The latter provides for
continuous background updates of the angle data enabling access with minimal
latency.

## Platforms

This document describes the case where sensor data is acquired, and fusion
is performed, on a single platform running MicroPython.

Other modes are supported:

 * Fusion and data acquisition run on a common device under standard Python.
 * Fusion and data acquisition run on separate devices linked by some form of
 communications link. In this mode the data acquisition device may run any type
 of code and return data in any format, with the user application reading and
 converting the data to a form acceptable to the library.

These modes are discussed [here](./remote/README.md).

## MicroPython issues

The code is intended to be independent of the sensor device: testing was done
with the InvenSense MPU-9150.

The algorithm makes extensive use of floating point maths. Under MicroPython
this implies RAM allocation. While the code is designed to be platform agnostic
problems may be experienced on platforms with small amounts of RAM. Options
are to use frozen bytecode and to periodically run a garbage collection; the
latter is advisable even on the Pyboard. See the `fusionlcd.py` test program
for an example of this.

## MicroPython firmware dependency

Some modules in this library use asynchronous programming. This uses the
`asyncio` library under CPython, `uasyncio` under MicroPython. The MicroPython
version is much improved after a complete rewrite and is at version 3.0. It is
syntax compatible with CPython 3.8 `asyncio`. All code has been updated to use
this syntax, which is unsupported by older versions.

To run asynchronous modules MicroPython targets should use a daily build of
firmware, or a release build after V1.12: such firmware incorporates `uasyncio`
V3.

Where asynchronous code is run under CPython, this must be V3.8 or later.

## Terminology and units of measurement

I should point out that I'm unfamiliar with aircraft conventions and would
appreciate feedback if the following observations about coordinate conventions
are incorrect.

### Angles

Inertial measurement units (IMU's) exist with and without magnetometers. Those
with them are known as 9DOF, and those without as 6DOF sensors where DOF stands
for "degrees of freedom". 6DOF sensors cannot provide heading information as
this is derived with reference to the Earth's magnetic field.

The object of sensor fusion is to determine a vehicle's attitude with respect
to Earth. This is expressed in the following three angles:

 1. `heading` Angle relative to North. Note some sources use the term "yaw".
 As this is also used to mean the angle of an aircraft's fuselage relative to
 its direction of motion, I have avoided it.
 2. `pitch` Angle of aircraft nose relative to ground (conventionally +ve is
 towards ground). Also known as "elevation".
 3. `roll` Angle of aircraft wings to ground, also known as "bank".

In this implementation these are measured in degrees.

### Sensors

The units of measurement provided by the sensor driver are important only in
the case of the gyroscope. This must provide a measurement in degrees per
second. Values from the accelerometer (typically g) and the magnetometer
(typically Microtesla) are normalised in the algorithm. This uses the fact that
the magnitude of these vectors is locally constant; consequently the units of
measurement are irrelevant.

The issue of the orientation of the sensor is discussed in
[section 4](./README.md#4-notes-for-constructors).

# Contents

 1. [Modules](./README.md#1-modules)  
 2. [Fusion module](./README.md#2-fusion-module)  
  2.1 [Fusion class](./README.md#21-fusion-class)  
   2.1.1 [Methods](./README.md#211-methods)  
   2.1.2 [Bound variables](./README.md#212-bound-variables)  
 3. [Asynchronous version](./README.md#3-asynchronous-version)  
  3.1 [Fusion class](./README.md#31-fusion-class)  
   3.1.1 [Methods](./README.md#311-methods)  
   3.1.2 [Variables](./README.md#312-variables)  
 4. [Notes for constructors](./README.md#4-notes-for-constructors)  
 5. [Background notes](./README.md#5-background-notes)  
  5.1 [Heading Pitch and Roll](./README.md#51-heading-pitch-and-roll)  
  5.2 [Beta](./README.md#52-beta)  
 6. [References](./README.md#6-references)

# 1. Modules

 1. `fusion.py` The standard synchronous fusion library.
 2. `fusion_async.py` Version of the library using uasyncio for nonblocking
 access to pitch, heading and roll.
 3. `deltat.py` Controls timing for above.
 4. `orientate.py` A utility for adjusting orientation of an IMU for sensor
 fusion.

Test/demo programs:

 1. `fusiontest.py` A simple test program for synchronous library.
 2. `fusiontest6.py` Variant of above for 6DOF sensors.
 3. `fusiontest_as.py` Simple test for the asynchronous library.
 4. `fusiontest_as6.py` Variant of above for 6DOF sensors.
 5. `fusionlcd.py` Tests the async library with a Hitachi HD44780 2-row LCD
 text display to continuously display angle values.

If using InvenSense MPU9150, MPU6050 or MPU9250 IMU's, drivers may be found
[here](https://github.com/micropython-IMU/micropython-mpu9x50).

The directory `remote` contains files and information specific to
[remote mode](./remote/README.md) and to running fusion on standard Python.

###### [Jump to Contents](./README.md#contents)

# 2. Fusion module

## 2.1 Fusion class

The module supports this one class. A Fusion object needs to be periodically
updated with data from the sensor. It provides heading, pitch and roll values
(in degrees) as properties. Note that if you use a 6DOF sensor, heading will be
invalid.

### 2.1.1 Methods

`update(accel, gyro, mag)`

For 9DOF sensors. Positional arguments:
 1. `accel` A 3-tuple (x, y, z) of accelerometer data.
 2. `gyro` A 3-tuple (x, y, z) of gyro data.
 3. `mag` A 3-tuple (x, y, z) of magnetometer data.

This method should be called periodically at a frequency depending on the
required response speed.

`update_nomag(accel, gyro)`

For 6DOF sensors.  Positional arguments:
 1. `accel` A 3-tuple (x, y, z) of accelerometer data.
 2. `gyro` A 3-tuple (x, y, z) of gyro data.

This should be called periodically, depending on the required response speed.

`calibrate(getxyz, stopfunc, wait=0)`

Positional arguments:  
 1. `getxyz` A function returning a 3-tuple of magnetic x,y,z values.
 2. `stopfunc` A function returning `True` when calibration is deemed
 complete: this could be a timer or an input from the user.
 3. `wait` A delay in ms. Some hardware may require a delay between
 magnetometer readings. Alternatively a function which returns after a delay
 may be passed.

Calibration updates the `magbias` bound variable. It is performed by rotating
the unit slowly around each orthogonal axis while the routine runs, the aim
being to compensate for offsets caused by static local magnetic fields.

### 2.1.2 Bound variables

Three bound variables provide access to the Euler angles in degrees:

 1. `heading`
 2. `pitch`
 3. `roll`

Quaternion data may be accesed via the `q` bound variable:

 1. `q` Contains `[w, x, y, z]` representing the normalised (unit) quaternion
 `w + xi + yj + zk`. Quaternion data is dimensionless.

See [my notes on quaternions](https://github.com/peterhinch/micropython-samples/blob/master/README.md#412-quaternions)
for code enabling them to be used to perform 3D rotation with minimal
mathematics. They are easier to use for this purpose than Euler angles.

A bound variable `beta` controls algorithm performance. The default value may
be altered after instantiation. See [section 5.2](./README.md#52-beta).

A class variable `declination`, defaulting to 0, enables the heading to be
offset in order to provide readings relative to true North rather than magnetic
North. A positive value adds to heading.

###### [Jump to Contents](./README.md#contents)

# 3. Asynchronous version

This uses the `uasyncio` library and is intended for applications based on
asynchronous programming. Updates are performed by a continuously running
coroutine. The `heading`, `pitch`, `roll` and `q` values are bound variables
which may be accessed at any time with effectively zero latency. The test
program `fusionlcd.py` illustrates its use showing realtime data on a text
LCD display, `fusiontest_as.py` prints it at the REPL.

## 3.1 Fusion class

The module supports this one class. The constructor is passed a user-supplied
coro which returns the accelerometer, gyro, and (in the case of 9DOF sensors)
magnetometer data. A Fusion instance has a continuously running coroutine which
maintains the heading, pitch and roll bound variables.

Typical constructor call:

```python
imu = MPU9150('X')  # Instantiate IMU (default orientation)

async def read_coro():
    imu.mag_trigger()  # Hardware dependent: trigger a nonblocking read
    await asyncio.sleep_ms(20)  # Wait for mag to be ready
    return imu.accel.xyz, imu.gyro.xyz, imu.mag_nonblocking.xyz
    # Returned (ax, ay, az), (gx, gy, gz), (mx, my, mz)

fuse = Fusion(read_coro)
```

The update method is started as follows (usually, in the case of 9DOF sensors,
after a calibration phase):

```python
    await fuse.start()
```

This starts a continuously running update task. It calls the coro supplied to
the constructor to determine (from the returned data) whether the sensor is a
6DOF or 9DOF variety. It then launches the appropriate task. From this point
the application accesses the `heading`, `pitch` and `roll` bound
variables as required.

### 3.1.1 Methods

Constructor:

This takes a single argument which is a coroutine. This returns three (x, y, z)
3-tuples for accelerometer, gyro, and magnetometer data respectively. In the
case of 6DOF sensors it returns two 3-tuples for accelerometer and gyro only.
The coroutine must include at least one `await asyncio.sleep_ms` statement to
conform to Python syntax rules. A nonzero delay may be required by the IMU
hardware; it may also be employed to limit the update rate, thereby controlling
the CPU resources used by this task.

`async def start(slow_platform=False)`  
This launches the update task, returning immediately.

Optional argument:  
 1. `slow_platform` Boolean. Adds a yield to the scheduler in the middle of
 the  computation. This may improve application performance on slow platforms
 such as the ESP8266.

`async def calibrate(stopfunc)`  
For 9DOF sensors only.

Argument:
 1. `stopfunc` Function returning `True` when calibration is deemed
 complete: this could be a timer or an input from the user.

Calibration updates the `magbias` bound variable. It is performed by rotating
the unit slowly around each orthogonal axis while the routine runs, the aim
being to compensate for offsets caused by static local magnetic fields.

### 3.1.2 Variables

Three bound variables provide the angles with negligible latency. Units are
degrees.

 1. `heading`
 2. `pitch`
 3. `roll`

Quaternion data may be accesed via the `q` bound variable:

 1. `q` Contains `[w, x, y, z]` representing the normalised (unit) quaternion
 `w + xi + yj + zk`. Quaternion data is dimensionless.

 A bound variable `beta` controls algorithm performance. The default value may
be altered after instantiation. See [section 5.2](./README.md#52-beta).

A class variable `declination`, defaulting to 0, enables the heading to be
offset in order to provide readings relative to true North rather than magnetic
North. A positive value adds to heading.

###### [Jump to Contents](./README.md#contents)

# 4. Notes for constructors

If you're developing a machine using a motion sensing device consider the
effects of vibration. This can be surprisingly high (use the sensor to measure
it). No amount of digital filtering will remove it because it is likely to
contain frequencies above the sampling rate of the sensor: these will be
aliased down into the filter passband and affect the results. It's normally
necessary to isolate the sensor with a mechanical filter, typically a mass
supported on very soft rubber mounts.

If using a magnetometer consider the fact that the Earth's magnetic field is
small: the field detected may be influenced by ferrous metals in the machine
being controlled or by currents in nearby wires. If the latter are variable
there is little chance of compensating for them, but constant magnetic offsets
may be addressed by calibration. This involves rotating the machine around each
of three orthogonal axes while running the fusion object's `calibrate`
method.

The local coordinate system for the sensor is usually defined as follows,
assuming the vehicle is on the ground:  
z Vertical axis, vector points towards ground  
x Axis towards the front of the vehicle, vector points in normal direction of
travel  
y Vector points left from pilot's point of view (I think)  
orientate.py has some simple code to correct for sensors mounted in ways which
don't conform to this convention.

You may want to take control of garbage collection (GC). In systems with
continuously running control loops there is a case for doing an explicit GC on
each iteration: this tends to make the GC time shorter and ensures it occurs at
a predictable time. See the MicroPython `gc` module.

###### [Jump to Contents](./README.md#contents)

# 5. Background notes

These are blatantly plagiarised as this isn't my field. I have quoted sources.

## 5.1 Heading Pitch and Roll

Perhaps better titled heading, elevation and bank: there seems to be ambiguity
about the concept of yaw, whether this is measured relative to the aircraft's
local coordinate system or that of the Earth: the original Madgwick study uses
the term "heading", a convention I have retained as the angles emitted by the
Madgwick algorithm (Tait-Bryan angles) are earth-relative.  
See [Wikipedia article](http://en.wikipedia.org/wiki/Euler_angles#Tait.E2.80.93Bryan_angles)

The following adapted from https://github.com/kriswiner/MPU-9250.git  
These are Tait-Bryan angles, commonly used in aircraft orientation (DIN9300).
In this coordinate system the positive z-axis is down toward Earth. Yaw is the
angle between Sensor x-axis and Earth magnetic North (or true North if
corrected for local declination). Looking down on the sensor positive yaw is
counter-clockwise. Pitch is angle between sensor x-axis and Earth ground plane,
aircraft nose down toward the Earth is positive, up toward the sky is negative.
Roll is angle between sensor y-axis and Earth ground plane, y-axis up is
positive roll. These arise from the definition of the homogeneous rotation
matrix constructed from quaternions. Tait-Bryan angles as well as Euler angles
are non-commutative; that is, the get the correct orientation the rotations
must be applied in the correct order which for this configuration is yaw,
pitch, and then roll. For more see
[Wikipedia article](http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
which has additional links.

I have seen sources which contradict the above directions for yaw (heading) and
roll (bank).

## 5.2 Beta

The Madgwick algorithm has a "magic number" Beta which determines the tradeoff
between accuracy and response speed.

[Source](https://github.com/kriswiner/MPU-9250.git) of comments below.  
There is a tradeoff in the beta parameter between accuracy and response speed.
In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError
of 2.7 degrees/s) was found to give optimal accuracy. However, with this value,
the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
Subsequent changes also require a longish lag time to a stable output, not fast
enough for a quadcopter or robot car! By increasing beta (GyroMeasError) by
about a factor of fifteen, the response time constant is reduced to ~2 sec. I
haven't noticed any reduction in solution accuracy. This is essentially the I
coefficient in a PID control sense; the bigger the feedback coefficient, the
faster the solution converges, usually at the expense of accuracy. In any case,
this is the free parameter in the Madgwick filtering and fusion scheme.

###### [Jump to Contents](./README.md#contents)

# 6. References

[Original Madgwick study](http://sharenet-wii-motion-trac.googlecode.com/files/An_efficient_orientation_filter_for_inertial_and_inertialmagnetic_sensor_arrays.pdf)  
[Euler and Tait Bryan angles](http://en.wikipedia.org/wiki/Euler_angles#Tait.E2.80.93Bryan_angles)  
[Quaternions to Euler angles](http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)  
[Beta](https://github.com/kriswiner/MPU-9250.git)
