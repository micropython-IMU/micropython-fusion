# micropython-fusion

Sensor fusion calculates heading, pitch and roll from the outputs of motion
tracking devices. This uses the Madgwick algorithm, widely used in multicopter
designs for its speed and quality. An update takes about 1.6mS on the Pyboard.
The original Madgwick study indicated that an update rate of 10-50Hz was
adequate for accurate results, suggesting that the performance of this
implementation is fast enough.

Two implementations are provided: one for synchronous code and one for
applications using ``uasyncio``. The latter provides for continuous background
updates of the angle data enabling its access with minimal latency.

The code is intended to be independent of the sensor device: testing was done
with the InvenSense MPU-9150.

The algorithm makes extensive use of floating point maths. Under MicroPython
this implies RAM allocation. While the code is designed to be platform agnostic
problems may be experienced on platforms with small amounts of RAM. Options
are to use frozen bytecode and to periodically run a garbage collection; the
latter is advisable even on the Pyboard. See the ``fusionlcd.py`` test program
for an example of this.

# Introduction

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

 1. ``heading`` Angle relative to North. Note some sources use the term "yaw".
 As this is also used to mean the angle of an aircraft's fuselage relative to
 its direction of motion, I have avoided it.
 2. ``pitch`` Angle of aircraft nose relative to ground (conventionally +ve is
 towards ground). Also known as "elevation".
 3. ``roll`` Angle of aircraft wings to ground, also known as "bank".

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

   2.1.2 [Properties and bound variables](./README.md#212-properties-and-bound-variables)

 3. [Asynchronous version](./README.md#3-asynchronous-version)

  3.1 [Fusion class](./README.md#31-fusion-class)

   3.1.1 [Methods](./README.md#311-methods)

   3.1.2 [Bound variables](./README.md#312-bound-variables)

 4. [Notes for constructors](./README.md#4-notes-for-constructors)
 
 5. [Background notes](./README.md#5-background-notes)

  5.1 [Heading Pitch and Roll](./README.md#51-heading-pitch-and-roll)

  5.2 [Beta](./README.md#52-beta)

 6. [References](./README.md#6-references)

# 1. Modules

 1. ``fusion.py`` The standard synchronous fusion library.
 2. ``fusion_async.py`` Version of the library using uasyncio for nonblocking
 access to pitch, heading and roll.
 3. ``orientate.py`` A utility for adjusting orientation of an IMU for sensor
 fusion.
 4. ``fusiontest.py`` A simple test program for synchronous library.
 5. ``fusiontest_as.py`` Simple test for the asynchronous library.
 6. ``fusionlcd.py`` A test program for the async library which uses a Hitachi
 HD44780 2-row LCD text display to show angle values.

###### [Jump to Contents](./README.md#contents)

# 2. Fusion module

## 2.1 Fusion class

The module supports this one class. A Fusion object needs to be periodically
updated with data from the sensor. It provides heading, pitch and roll values
(in degrees) as properties. Note that if you use a 6DOF sensor, heading will be
invalid.

### 2.1.1 Methods

``update(accel, gyro, mag)``

For 9DOF sensors. Positional arguments:
 1. ``accel`` A 3-tuple (x, y, z) of accelerometer data.
 2. ``gyro`` A 3-tuple (x, y, z) of gyro data.
 3. ``mag`` A 3-tuple (x, y, z) of magnetometer data.

This method should be called periodically at a frequency depending on the
required response speed.

``update_nomag(accel, gyro)``

For 6DOF sensors.  Positional arguments:
 1. ``accel`` A 3-tuple (x, y, z) of accelerometer data.
 2. ``gyro`` A 3-tuple (x, y, z) of gyro data.

This should be called periodically, depending on the required response speed.

``calibrate(getxyz, stopfunc, wait=0)``

Positional arguments:  
 1. ``getxyz`` A function returning a 3-tuple of magnetic x,y,z values.
 2. ``stopfunc`` A function returning ``True`` when calibration is deemed
 complete: this could be a timer or an input from the user.
 3. ``wait`` A delay in ms. Some hardware may require a delay between
 magnetometer readings. Alternatively a function which returns after a delay
 may be passed.

The method updates the ``magbias`` bound variable.

Calibration is performed by rotating the unit around each orthogonal axis while
the routine runs, the aim being to compensate for offsets caused by static
local magnetic fields.

### 2.1.2 Properties and bound variables

Three read-only properties provide access to the angles in degrees:

 1. ``heading``
 2. ``pitch``
 3. ``roll``

A bound variable ``beta`` controls algorithm performance. The default value may
be altered after instantiation. See [section 5.2](./README.md#52-beta).

A class variable ``declination``, defaulting to 0, enables the heading to be
offset in order to provide readings relative to true North rather than magnetic
North. A positive value adds to heading.

###### [Jump to Contents](./README.md#contents)

# 3. Asynchronous version

This uses the ``uasyncio`` library and is intended for applications based on
asynchronous programming. Updates are performed by a continuously running
coroutine. The ``heading``, ``pitch`` and ``roll`` values are bound variables
which may be accessed at any time with effectively zero latency. The test
program ``fusionlcd.py`` illustrates its use showing realtime data on a text
LCD display.

## 3.1 Fusion class

The module supports this one class. A Fusion instance runs an ``update``
coroutine contiuously which maintains the heading, pitch and roll bound
variables. This is achieved by running user supplied callbacks providing data
from the sensor. Note that if you use a 6DOF sensor, heading will be invalid
(0).

The update method is run as follows (typically, in the case of 9DOF sensors,
after a calibration phase):

```python
    loop = asyncio.get_event_loop()
    loop.create_task(fusion.update(accel, gyro, mag, 20))
```

### 3.1.1 Methods

```async def update(faccel, fgyro, fmag, delay)```

For 9DOF sensors. Positional arguments:  
 1. ``faccel`` Function returning a 3-tuple (x, y, z) of accelerometer data.
 2. ``fgyro`` Function returning a 3-tuple (x, y, z) of gyro data.
 3. ``fmag`` Function returning a 3-tuple (x, y, z) of magnetometer data.
 4. ``delay`` Delay between updates (ms). This value may be IMU hardware or
 application dependent.

``async def update_nomag(accel, gyro, delay)``

For 6DOF sensors. Mandatory positional arguments:
 1. ``faccel`` Function returning a 3-tuple (x, y, z) of accelerometer data.
 2. ``fgyro`` Function returning a 3-tuple (x, y, z) of gyro data.
 3. ``delay`` Delay between updates (ms). This value may be IMU hardware or
 application dependent.

``async def calibrate(getxyz, stopfunc, wait = 0)``

Arguments:
 1. ``getxyz`` Function returning a 3-tuple (x, y, z) of magnetometer data.
 2. ``stopfunc`` Function returning ``True`` when calibration is deemed
 complete: this could be a timer or an input from the user.
 3. ``wait`` Delay in ms between magnetometer readings. May be required by some
 hardware.

The method updates the ``magbias`` bound variable.

Calibration is performed by rotating the unit around each orthogonal axis while
the routine runs, the aim being to compensate for offsets caused by static
local magnetic fields.

### 3.1.2 Bound variables

Three bound variables provide the angles with negligible latency. Units are
degrees.

 1. ``heading``
 2. ``pitch``
 3. ``roll``

A bound variable ``beta`` controls algorithm performance. The default value may
be altered after instantiation. See [section 5.2](./README.md#52-beta).

A class variable ``declination``, defaulting to 0, enables the heading to be
offset in order to provide readings relative to true North rather than magnetic
North. A positive value adds to heading.

###### [Jump to Contents](./README.md#contents)

# 4. Notes for constructors

If you're developing a machine using a motion sensing device consider the
effects of vibration. This can be surprisingly high (use the sensor to measure
it). No amount of digital filtering will remove it because it is likely to
contain frequencies above the sampling rate of the sensor: these will be
aliased down into the filter passband and affect the results. It's normally
neccessary to isolate the sensor with a mechanical filter, typically a mass
supported on very soft rubber mounts.

If using a magnetometer consider the fact that the Earth's magnetic field is
small: the field detected may be influenced by ferrous metals in the machine
being controlled or by currents in nearby wires. If the latter are variable
there is little chance of compensating for them, but constant magnetic offsets
may be addressed by calibration. This involves rotating the machine around each
of three orthogonal axes while running the fusion object's ``calibrate``
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
a predictable time. See the MicroPython ``gc`` module.

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
counterclockwise. Pitch is angle between sensor x-axis and Earth ground plane,
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