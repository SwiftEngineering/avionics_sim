# avionics_sim

A library for simulating avionics commonly used in small UAS, such as pressure sensors, IMUs, GPS, batteries and other supporting utility, model, and simulation code.

Currently intended for use with PX4's sitl_gazebo.

## Features

 * A configurable battery model with transient response
 * A differential pressure sensor model
 * A first order discrete time low pass filter
 * A Gaussian-Markov processes noise generator
   * Seedable, resettable
   * MT19937-64 based
 * An implementation of the US 1976 atmosphere model
 * A 2D (Bilinear) lookup table implementation

## Planned Features

* More sensor models

## Platform Requirements

 * C++11
 * Boost
 * CMake

## Related repositories

 * avionics_sim_unit_tests
 * avionics_sim_unit_tests_runner

## Copyright

Copyright (c) Swift Engineering Inc. 2018

## License

Licensed by Swift Engineering Inc. under the MIT license. See LICENSE file for details.