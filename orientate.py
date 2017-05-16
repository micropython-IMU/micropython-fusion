# orientate.py Utility for adjusting orientation of an IMU for sensor fusion
# Author Peter Hinch
# Released under the MIT License (MIT)
# Copyright (c) 2015 Peter Hinch

# Orientation.
# The convention for vehicle axes is
# z Vertical axis, vector points towards ground
# x Axis towards the front of the vehicle, vector points in normal direction of travel
# y Vector points left from pilot's point of view (I think)
# Assuming the sensor is aligned such that it has one axis Vertical and another forward, the
# transpose function enables axes to be swapped to achieve the conventional orientation.
# The invert() function enables the sign of each axis to be corrected.

# Note that it is the responsibilty of the driver to esnure that accelerometer, gyro and
# magnetometer share the same coordinate system. In particular the MPU-9150's magnetometer is
# rotated with respect to the other two sensors. The MicroPython driver transposes the magnetometer
# axes to conform with those of the other two sensors.


def invert(axes, vectors):  # Invert one or more axes on a list of vectors
    res = []
    for vector in vectors:
        res.append(tuple(map(lambda x, y: -y if x else y, axes, vector)))
    return res

def transpose(idx, vectors):# transpose axes in a list of vectors
    res = []                # e.g. swap x and y transpose((1, 0, 2), (accel, gyro, mag))
    for vector in vectors:
        res.append([vector[axis] for axis in idx])
    return res

# t: tuple of vectors to transpose (0, 1, 2) indicates do nothing
# i: tuple of booleans indicating any vectors to be inverted.
# folowed by accel, gyro and magnetometer vectors (x, y, z).
# Note inversion operates on conventional vectors i.e. after transposition.
# e.g. (False, False, True) invert vertical axis.
# Returns a list of vectors. Typical invocation:
# fuse.update(*orientate(T, I, imu.get_accel(), imu.get_gyro(), imu.get_mag()))

def orientate(t, i, *vecs):
    return invert(i, transpose(t, vecs))
