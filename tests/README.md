# avionics_sim tests

A collection of Google tests and a test runner for testing avionics_sim classes.

## Getting Started

Run ```cmake``` in the ```test_runner``` folder, then ```make```. This will build the test runner binary, ```avionics_sim_test_runner```. 

## Running the tests

To run unit tests only, run ```avionics_sim_test_runner``` with the  flag ```-UnitTests``` . To run integration tests, run it with ```-IntegrationTests```. Running ```avionics_sim_test_runner``` without options will display usage information.

## Features

 * Unit tests for avionics_sim classes
 * Integration tests for avionics_sim classes

## Planned Features

* System level tests
* Test Case Reports for Unit Tests
* Individual Test Execution and Reporting

## Platform Requirements

 * C++11
 * Boost
 * CMake
 * GTest

## Related repositories

 * avionics_sim

## Copyright

Copyright (c) Swift Engineering Inc. 2019

## License

Licensed by Swift Engineering Inc. under the MIT license. See LICENSE file for details.
