/**
 * @brief       Bilinear_interp
 * @file        Bilinear_interp.cpp
 * @author      Evan Johnson <erjohnson227@gmail.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <vector>
#include <string>

namespace avionics_sim {
class Bilinear_interp {
 public:

  // INTERP_SUCCESS             Indicates that interpolation was a success.
  // INTERP_WARN_OUT_OF_BOUNDS  Indicates that the interpolation was clamped because input data was out of bounds
  //                            and was therefore clamped to the nearest available data. This output still returns
  //                            the interpolation result for the clamped input data.
  // INTERP_ERROR_NO_LUT        Indicates that the interpolation failed due to no LUT data being loaded into the class
  //                            LUT variables. This error is only returned when the overloaded function
  //                            interpolate2D(...) is called which does not take the LUT data as input variables
  //                            if this error has occured, the user must either call set[X/Y/Z]Vals(...) or must
  //                            construct the class using the overloaded constructor which takes the LUT values as
  //                            input parameters.
  enum InterpResult {
    INTERP_SUCCESS =              0,  ///< Interpolation was a success.
    INTERP_WARN_OUT_OF_BOUNDS =  -1,  ///< Interpolation was clamped because input data was out of bounds.
    INTERP_ERROR_NO_LUT =         -2  ///< Interpolation failed due to no LUT being loaded in the class.
  };



  Bilinear_interp(); ///< Default constructor

  ///
  /// \brief      Constructs an interpolation object with x, y, and z vectors being passed at construciton.
  ///
  /// \param[in]  xv  Reference to x values of the 2D LUT
  /// \param[in]  yv  Reference to Y values of the 2D LUT
  /// \param[in]  zv  Reference to Z values of the 2D LUT
  /// X values and Y values must be in ascending order.
  Bilinear_interp(const std::vector<std::vector<double>> &xv,
                  const std::vector<double> &yv, const std::vector<std::vector<double>> &zv);

  // Function to perform 1D (Linear) interpolation.
  ///
  /// \brief      Performs a 1D interpolation based on X,Y LUT data.
  ///
  /// \details    Performs a 1D interpolation to find a Y value given an input of an x location withing a 1D
  ///             sample space. Clamps to nearest available data if input range is out of bounds for x w.r.t LUT
  ///             data.
  /// \param[in]  xv    reference to x values of the LUT
  /// \param[in]  yv    reference to y values of the LUT
  /// \param[in]  x     x location to be used in calculation of y
  /// \param      y     pointer to y variable to contain interpolation output.
  ///
  /// \return      Returns InterpResult enum containing potential interpolation errors.
  /// X values must be ascending
  static InterpResult interpolate(const std::vector<double> &xv,
                                  const std::vector<double> &yv, double x, double *const y);



  // Function to perfrom 2D (Bilinear) interpolation.
  ///
  /// \brief      Performs 2D interpolations on LUT
  ///
  /// \details    Performs a 2D linear interpolation to find a Z value given an input of x and y locations within a 2D
  ///             sample space. Clamps to nearest available data if input range is out of bounds for x and y w.r.t LUT
  ///             data.
  /// \param[in]      xv    reference to x values of the 2D LUT
  /// \param[in]      yv    reference to Y values of the 2D LUT
  /// \param[in]      zv    reference to Z values of the 2D LUT
  /// \param[in]      x     x location to be used in calculation of z
  /// \param[in]      y     y location to be used in calculation of z
  /// \param[out]     z     pointer to z value to contain interpolation output.
  ///
  /// \return      Returns InterpResult enum containing potential interpolation errors.
  /// X and Y values must be ascending
  InterpResult interpolate2D(const std::vector<std::vector<double>> &xv,
                             const std::vector<double> &yv, const std::vector<std::vector<double>> &zv,
                             double x, double y, double *const z);

  ///
  /// \brief      Performs 2D interpolations on LUT based on x, y, and z values stored within the class.
  ///
  /// \details    Performs a 2D linear interpolation to find a Z value given an input of x and y locations within a 2D
  ///             sample space. Clamps to nearest available data if input range is out of bounds for x and y w.r.t LUT
  ///             data. x, y, and z LUT values must be loaded into the class prior to calling this function, either via
  ///             the constructor method, or via the set[X/Y/Z]Vals functions.
  /// \param[in]      x     x location to be used in calculation of z
  /// \param[in]      y     y location to be used in calculation of z
  /// \param[out]     z     pointer to z value to contain interpolation output.
  ///
  /// \return     Returns InterpResult enum containing potential interpolation errors.
  /// X and Y values must be ascending
  InterpResult interpolate2D(double x, double y, double *const z);



  // Static helper functions to load data elements into vectors.
  ///
  /// \brief      Gets a one dimensional vector of LUT elements from a string.
  /// \details    Takes a string containing comma separated values representing values
  ///            withing a LUT.
  ///
  /// \param[in]  inputVals The input string to be processed.
  /// \param[out] outputVect A pointer to std::vector<double> containing the LUT elements.
  ///
  /// \return     A bool indicating whether the parsing was a success.
  ///
  static bool get1DLUTelementsFromString(const std::string &inputVals,
                                         std::vector<double> *const outputVect);

  ///
  /// \brief      Gets a two dimensional vector of LUT elements from a string.
  /// \details    Takes a string containing semicolon separated rows of comma separated
  ///            values representing points on a 2D LUT.
  ///
  /// \param[in]  inputVals  The input string to be processed.
  /// \param[out] outputVect A pointer to std::vector<std::vector<<double>> containing the 2D LUT elements.
  ///
  /// \return     A bool indicating whether the parsing was a success.
  ///
  static bool get2DLUTelementsFromString(const std::string &inputVals,
                                         std::vector<std::vector<double>> *const outputVect);



  ///
  /// \brief      Sets the x vals vector.
  ///
  /// \param[in]  inputVals  String to be parsed for input values.
  ///
  /// \return A bool indicating whether the operation was a success.
  ///         A failure is only returned if a parse call is made to
  ///         get[1D/2D]LUTelementsFromString() which has failed to parse.
  ///
  bool setXVals(const std::string &inputVals);

  ///
  /// \brief      Sets the x vals vector.
  ///
  /// \param[in]  xv     Reference to vector containing the x values.
  ///
  void setXVals(const std::vector<std::vector<double>> &xv);

  ///
  /// \brief      Sets the y vals vector.
  ///
  /// \param[in]  inputVals  String to be parsed for input values.
  ///
  /// \return A bool indicating whether the operation was a success.
  ///         A failure is only returned if a parse call is made to
  ///         get[1D/2D]LUTelementsFromString() which has failed to parse.
  ///
  bool setYVals(const std::string &inputVals);

  ///
  /// \brief      Sets the y vals vector.
  ///
  /// \param[in]  yv     Reference to vector containing the y values.
  ///
  void setYVals(const std::vector<double> &yv);


  ///
  /// \brief      Sets the z vals vector.
  ///
  /// \param[in]  inputVals  String to be parsed for input values.
  ///
  /// \return A bool indicating whether the operation was a success.
  ///         A failure is only returned if a parse call is made to
  ///         get[1D/2D]LUTelementsFromString() which has failed to parse.
  ///
  bool setZVals(const std::string &inputVals);

  ///
  /// \brief      Sets the z vals vector.
  ///
  /// \param[in]  zv     Reference to vector containing the z values.
  ///
  void setZVals(const std::vector<std::vector<double>> &zv);

  ///
  /// \brief      Gets the x vals vector.
  ///
  /// \param[out]  xVect    Pointer to array which will hold returned values.
  ///
  /// \return A bool indicating whether the operation was a success.
  ///
  bool getX(std::vector<std::vector<double>> *const xVect);

  ///
  /// \brief      Gets the y vals vector.
  ///
  /// \param[out]  yVect    Pointer to array which will hold returned values.
  ///
  /// \return A bool indicating whether the operation was a success.
  ///
  bool getY(std::vector<double> *const yVect);

  ///
  /// \brief      Gets the z vals vector.
  ///
  /// \param[out]  zVect    Pointer to array which will hold returned values.
  ///
  /// \return A bool indicating whether the operation was a success.
  ///
  bool getZ(std::vector<std::vector<double>> *const zVect);

 private:
  /// \brief Vectors to hold LUT data.
  /// Holds 2D LUT values for X and Z points
  std::vector< std::vector<double> > xVals, zVals;
  std::vector<double> yVals; ///< Holds Y values for LUT data.

  /// Bools to keep track of which LUT data values have been added.
  bool haveXVals, haveYVals, haveZVals;
};
}