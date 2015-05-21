# micropython-fusion
Sensor fusion calculating yaw, pitch and roll from the outputs of motion tracking devices. This
uses the Madgwick algorithm, widely used in multicopter designs for its speed and quality. An
update takes about 1.6mS on the Pyboard

### Fusion Class

The module supports this one class. A Fusion object needs to be periodically updated with data
from the sensor. When queried by the ``angle`` method, it returns yaw, pitch and roll values in
degrees. Note that if you use a 6 degrees of freedom (DOF) sensor, yaw will be invalid.

Methods
-------

```angle()```
Return yaw, pitch and roll in degrees

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

### Notes for beginners

If you're designing a machine using a motion sensing device consider the effects of vibration.
This can be surprisingly high (use the sensor to measure it). No amount of digital filtering
will remove it because it is likely to contain frequencies above the sampling rate of the sensor:
these will be aliased down into the filter passband and affect the results. It's normally
neccessary to isolate the sensor with a mechanical filter, typically a mass supported on very
soft rubber mounts.

Update rate - TODO

