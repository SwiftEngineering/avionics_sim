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

## How to Read This Documentation

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

The documentation for this aerodynamic model is located [here](documentation/liftdrag_model/README.md).
