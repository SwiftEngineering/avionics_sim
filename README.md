# avionics_sim

A library for simulating avionics commonly used in small UAS, such as pressure sensors, IMUs, GPS, batteries and other supporting utility, model, and simulation code.

Currently intended for use with PX4's sitl_gazebo; however, development strives to be independent from any project.

## Features

 * Aerodynamic Model with configurable airfoils
 * Wind/Turbulence Modeling
 * Transmission Modeling for cases like elevon linkages
 * Configurable battery model with transient response
 * Differential pressure sensor model
 * First order discrete time low pass filter
 * Gaussian-Markov processes noise generator
   * Seedable, resettable
   * MT19937-64 based
 * US 1976 atmosphere model
 * 2D (Bilinear) lookup table implementation

## Planned Features

* More sensor models

## Platform Requirements

 * C++11
 * Boost
 * CMake
 * Ignition Math

Alternatively:

* Docker

## Getting Started

As part of an effort to make this library as open to extensibility as possible the choice was made to utilize containerization and an IDE configuration that readily supports it for quickly bringing up a build environment for developers. While still currently listed as the [third most popular IDE](https://pypl.github.io/IDE.html) VS Code does have integrated supported for docker containirization and is rapidly growing in popularity,  a feature not fully supported by the leading two IDEs. As such this project is distributed with project configuration files for a VS Code Project. Containerization helps in providing developers a consistent build environment the develop in without being inundated with the task of installing specific dependency versions. The containerization used by this project is Docker, while it is the [second most popular containerization](https://www.datanyze.com/market-share/containerization--321) it is designed for containing applications rather than entire operating systems like the most popular container system LXC.

To start development all that should be required is downloading the following:
* VS Code
* Docker
And opening the project folder within the IDE, and then opening the project in a container when prompted by the IDE.

If developers would like to further investigate what is happening in this regard, it is recommended to review the official documentation provided by the IDE team [here](https://code.visualstudio.com/docs/remote/containers) as that is not the scope of this project. It is the belief of the original developers of this project that providing a containerized environment and at least one IDE project configuration should become a standard adopted by most public repos as it handles the most frustating hurdle for new developers when working on a new project: setting up your environment/dependencies.

## Building

Avionics Sim uses cmake for configuration intended for a shadow build process.

From the project directory, that looks like the following steps

1. Initialize the shadow build directory, and enter it
```sh
mkdir -p build && cd build
```
2. Configure project using the Cmake file in the project directory
```sh
cmake ..
```
3. Build the project using the generated makefile
```
make
```

## Testing

Avionics Sim uses GTest as the test framework, as such within the directory where the project is configured and built(see building) any of the following can be ran to run the tests:

```sh
make test
```

```sh
ctest
```

for a more detailed version of the test

```sh
./test/unit/unit_tests_exe
```

```sh
ctest --verbose
```

running a specifc test will require filtering on said test, ie: Battery_model_UnitTest.Test_constant_current_discharge_no_interpolation_low
We can filter on any layer:
```sh
./test/unit/unit_tests_exe --gtest_filter=*Test_constant_current_discharge_no_interpolation_low
```

```sh
ctest -R Test_constant_current_discharge_no_interpolation_low
```

## Copyright

Copyright (c) Swift Engineering Inc. 2021

## License

Licensed by Swift Engineering Inc. under the MIT license. See LICENSE file for details.

## How to Read This Documentation

Features have their own documentation seperated out within the `documentation` folder. In addition to markdown files for feature description and usage, when necessary supplementary Jupyter Notebooks are provided for predicitive analysis of the model.

If reading this document on a local machine, please ensure that the complete contents of the `documentation` folder have been pulled and to use a markdown reader such as Typora.

## Bilinear Interpolation

The bilinear interpolation library is a convenience library containing a single class called `Bilinear_interp` which can be used to automate linear (1D) and bilinear (2D) interpolation from lookup tables. The class also contains helper functions which can load comma separated values into one or two dimensional `std::vector<float>`. This library is used extensively in both the `avionics_sim` LiftDragPlugin, and the `gazebo_motor_model`.

### CSV Helper Functions

The Bilinear Interpolation library contains two helper functions which can be used to load one dimensional and two dimensional lookup tables from CSV into C++ standard vectors. These functons are listed below:

* `bool get1DLUTelementsFromString(const ::stdstring& inputVals, std::vector<float>* const outputVect)`
* `bool get2DLUTelementsFromString(const std::string& inputVals, std::vector<std::vector<float>>* const outputVect)`

Both of these functions serve the purpose of loading lookup table values from a text string, and returning them as vectors. The first function loads a one dimensional lookup table from an input string `inputVals` and stores the parsed values in an `std::vector<float>` called `outputVect`. The function returns a `TRUE` if the parsing was successful and a `FALSE` if the parsing failed. Currently the only means by which parsing will fail is if the input string is malformed or contains illegal characters. The second function behaves in an identical manner to the first, except that it parses two dimensional values from a text string. Columns are delimited by a comma "," and rows are delimited by a semicolon ";". The resulting value which is stored in `outputVect` is a two dimensional vector instead of a one dimensional vector. The behavior of the returned boolean value is also the same as with its one dimensional counterpart.

These resulting vectors can then be used within the `interpolate` and `interpolate2D` functions as input data to be interpolated on.

### Internal Class Storage

The Bilinear Interpolation class contains internal storage for three vectors: X, Y, and Z. These three vectors contain LUT information for a two-dimensional interpolation. The X and Y values represent the two dimensional input to the LUT, and the Z values represent the output at each (X,Y) location. These values can bet set in one of two ways. The first method is through an overloaded constructor (shown below):

`Bilinear_interp(const std::vector<std::vector<float>>& xv, const std::vector<float>& yv, const std::vector<std::vector<float>>& zv);`

The second method is through the explicit calling of the setters for the X, Y, and Z values. The following functions can be used to set these values:

* `void setXVals(const std::vector<float>& xv)`
* `void setYVals(const std::vector<float>& yv)`
* `void setZVals(const std::vector<float>& zv)`

These functions take the standard vector for each of the LUT parameters and set the internal variables. These values can be retrieved by getter functions:

* `getX(std::vector<std::vector<float>>* const xVect)`
* `getY(std::vector<std::vector<float>>* const yVect)`
* `getZ(std::vector<std::vector<float>>* const zVect)`

These functions return the internally set X, Y and Z values of the `Bilinear_interp` class.

### Data Interpolation Functions

The `Bilinear_interp` class contains two functions for interpolation. A function for one-dimensional interpolation, and a function for two-dimensional interpolation. The single-dimensional interpolation function is used for linear interpolation. It appears as follows:

`InterpResult interpolate(const std::vector<float> &xv, const std::vector<float> &yv, float x, float* const y)`

This function takes an input for the X values and Y values of the LUTs as denoted by `xv` and `yv`. These inputs are in the form of standard vectors. The function also takes an x value to interpolate at, and sets the `y` float pointer with the result of the calculation. The function returns an enumeration called `InterpResult` which can be one of three values `INTERP_SUCCESS` (0), `INTERP_WARN_OUT_OF_BOUNDS` (-1), and `INTERP_ERROR_NO_LUT` (-2). If the interpolation is within bounds of the LUT data `INTERP_SUCCESS` will be returned. If the value given for `x` is out of bounds for the (X,Y) pairs of the lookup table, it will be clamped to the closes available Y value and `INTERP_WARN_OUT_OF_BOUNDS` will be returned.

The two-dimensional version of the interpolation function `interpolate2D` is shown below. It has two overloaded forms:

`InterpResult interpolate2D(const std::vector<std::vector<float>>& xv, const std::vector<float>& yv, const std::vector<std::vector<float>>& zv, float x, float y, float* const z)`

and

`interpolate2D(float x, float y, float* const z)`

The first form of this function is similar in operation to its one-dimensional counterpart. The values for the (X,Z) pairs are passed into the function in the form of two dimensional standard vectors (`xv` and `zv`). A one-dimensional set of Y values is also passed in. Finally, the x and y value for the interpolation to occur at are also passed in as `x` and `y`. The result of the interpolation is passed back through the float pointer `z`. Similarly to the one-dimensional version, `INTERP_SUCCESS` will be returned if the (x,y) pair provided are within the (X,Y) space of the LUT data, otherwise `INTERP_WARN_OUT_OF_BOUNDS` will be returned.  The second form of the function is the same as the first, except that it relies on internally stored LUT values for `X`, `Y`, and `Z`. These must be set beforehand either through the overloaded constructor, or the setter functions shown above in the "Internal Class Storage" section. An (x,y) pair is passed to this function for interpolation. The resulting value is returned as `z`.  If this function is called without setting the internal values `INTERP_ERROR_NO_LUT` will be returned to indicate an error. Error handling should be performed by the calling function to ensure that either a `INTERP_SUCCESS` or `INTERP_WARN_OUT_OF_BOUNDS` are returned by the function depending on the requirements.



## Coordinate Utilities Library

The coordinate utilities library is used to change vectors from one coordinate frame to another. Currently the library only has one function `project_vector_global` which is used to project a vector from one coordinate frame (i.e. global coordinate frame) into another (for example body-frame).

 `static int project_vector_global(ignition::math::Pose3d targetFrame, ignition::math::Vector3d vec, ignition::math::Vector3d * const res)`

The function takes three arguments two of which are inputs and one of which is an output. The target coordinate frame is passed to the function as a ignition math `Pose3d` which represents a position and an orientation. Since this function simply projects vectors, the position of the target coordinate frame does not matter and is ignored. It was chosen to use the input as a `Pose3d` however because that is the standard format readily available within `sitl_gazebo` and therefore would not require conversion. The second input is a `Vector3d` of the three dimensional vector to be projected. The function projects this vector into the target coordinate frame and stores the resulting vector in another three dimensional vector called `res`.  Currently the return value is always 0.

## Lift Drag Aerodynamic Model

This aerodynamic model replaces a single $\frac{C_L}{\alpha}$ slope coefficient and associated ${C_L}_{max}$ with an aerodynamic lookup table where, for each $\alpha$ there exists an associated $C_L$ and $C_D$.  The main benefit of this method is that it allows reasonable estimates of aerodynamic parameters up to and past stall, i.e. ${C_L}_{max}$. This is particularly important for tailsitting transition vehicles that operate in the range $0 < \alpha < 90$ and beyond.

The documentation for this aerodynamic model is in three parts.

[Documentation for the mathematical basis of this aerodynamic model](documentation/liftdrag_model/mathematical_model/README.md).

[Documentation for Gazebo plugin](documentation/liftdrag_model/devguide/plugin/index.html).

[Documentation for Lift Drag Model C++ class](documentation/liftdrag_model/devguide/model_class/index.html).

## CLang Support

CLang support is currently configured for use within the development container, as such all commands following are defined in the context of running in the container.

### Styling

Run the following in the development container to style all C/C++ related files.

```sh
astyle *.cpp *.hpp *.h --suffix=none --recursive
```

### Linting

This project is currently using the cpplint linting tools for its close ties with the Google C++ Style Guide for which 
this project uses. Additional configuration of the tool is done through the `CPPLINT.cfg` configuration file. Which 
should note that makes the following modifactions to the style guide:

* Line Length : 120
* private/protected/public markers are not required to follow the 3 space indentation
* Const References required is not followed, this is becuase while the Style Guide allows for non-const references the cpplint tool oddley does not

To lint all the relevant files, run the following in the development container

```sh
cpplint --recursive src include tests
```

The CI system currently uses linting as a metric, and does so by outputing the results as a junit xml using the following command:
```sh
cpplint --output=junit --recursive src include test 2> build/lint_report.xml
```