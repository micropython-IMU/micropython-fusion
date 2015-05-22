# micropython-fusion
Sensor fusion calculating yaw, pitch and roll from the outputs of motion tracking devices. This
uses the Madgwick algorithm, widely used in multicopter designs for its speed and quality. An
update takes about 1.6mS on the Pyboard

# fusion module

## Fusion class

The module supports this one class. A Fusion object needs to be periodically updated with data
from the sensor. It provides yaw, pitch and roll values (in degrees) as properties. Note that
if you use a 6 degrees of freedom (DOF) sensor, yaw will be invalid.

### Methods

```update(accel, gyro, mag)```

For 9DOF sensors. Accepts 3-tuples (x, y, z) of accelerometer, gyro and magnetometer data and
updates the filters. This should be called periodically, depending on the required response
speed. Units:
accel: Typically G. Values are normalised in the algorithm so units are irrelevant.
gyro: Degrees per second.
magnetometer: Microteslas.

```update_nomag(accel, gyro)```

For 6DOF sensors. Accepts 3-tuples (x, y, z) of accelerometer and gyro data and
updates the filters. This should be called periodically, depending on the required response
speed. Units:
accel: Typically G. Values are normalised in the algorithm so units are irrelevant.
gyro: Degrees per second.

```calibrate(getxyz, stopfunc)```

The first argument is a function returning a tuple of magnetic x,y,z values from the sensor.
The second is a function returning ```True``` when calibration is deemed complete: a timer or an
input from the user. It updates the ```magbias``` property.

Calibration is performed by rotating the unit around each orthogonal axis while the routine
runs, the aim being to compensate for offsets caused by static local magnetic fields.

### Properties

Three read-only properties provide access to the angles. These are in degrees.

```yaw```

Angle relative to North (anticlockwise)

```pitch```

Angle of aircraft nose relative to ground (+ve is towards ground)

```roll```

Angle of aircraft wings to ground anticlockwise (left wing down)

### Notes for beginners

If you're developing a machine using a motion sensing device consider the effects of vibration.
This can be surprisingly high (use the sensor to measure it). No amount of digital filtering
will remove it because it is likely to contain frequencies above the sampling rate of the sensor:
these will be aliased down into the filter passband and affect the results. It's normally
neccessary to isolate the sensor with a mechanical filter, typically a mass supported on very
soft rubber mounts.

You may want to take control of garbage collection (GC). In systems with continuously running
control loops there is a case for doing an explicit GC on each iteration: this tends to make the
GC time shorter and ensures it occurs at a predictable time. See the MicroPython ```gc``` module.

If using a magnetoemeter consider the fact that the Earth's magnetic field is small: the field
detected may be influenced by ferrous metals in the machine being controlled or by currents in
nearby wires. If the latter are variable there is little chance of compensating for them, but
constant magnetic offsets may be addressed by calibration. This involves rotating the machine
around each of three orthogonal axes while running the fusion object's ```calibrate``` method.

Update rate - TODO

# Background notes

These are blatantly plagiarised as this isn't my field, but in my defence I have quoted sources.

### Yaw Pitch and Roll

Perhaps better titled heading, elevation and bank: there seems to be ambiguity about the concept
of yaw, whether this is measured relative to the aircraft's local coordinate system or that of
the Earth. I gather Tait-Bryan angles are earth-relative.  
See http://en.wikipedia.org/wiki/Euler_angles#Tait.E2.80.93Bryan_angles

The following adapted from https://github.com/kriswiner/MPU-9250.git  
These are Tait-Bryan angles, commonly used in aircraft orientation (DIN9300). In this coordinate
system the positive z-axis is down toward Earth. 
Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for
local declination). Looking down on the sensor positive yaw is counterclockwise.
Pitch is angle between sensor x-axis and Earth ground plane, aircraft nose down toward the Earth
is positive, up toward the sky is negative.
Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation
the rotations must be applied in the correct order which for this configuration is yaw, pitch, and then roll.
For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

### Beta

Source: https://github.com/kriswiner/MPU-9250.git  
There is a tradeoff in the beta parameter between accuracy and response speed.
In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s)
was found to give optimal accuracy. However, with this value, the LSM9SD0 response time is about
10 seconds to a stable initial quaternion. Subsequent changes also require a longish lag time to
a stable output, not fast enough for a quadcopter or robot car! By increasing beta (GyroMeasError)
by about a factor of fifteen, the response time constant is reduced to ~2 sec.
I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
