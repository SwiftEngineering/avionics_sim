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

namespace avionics_sim
{
    class Bilinear_interp
    {
        public:
          Bilinear_interp(); ///< Default constructor

          ///
          /// \brief      Constructs an interpolation object with x, y, and z vectors being passed at construciton. 
          ///
          /// \param[in]  xv  x values of the 2D LUT
          /// \param[in]  yv  Y values of the 2D LUT
          /// \param[in]  zv  Z values of the 2D LUT
          ///
          Bilinear_interp(std::vector<std::vector<float>> xv, std::vector<float> yv, std::vector<std::vector<float>> zv);

          // Function to perform 1D (Linear) interpolation.     
          ///
          /// \brief      Performs a 1D interpolation based on X,Y LUT data. 
          ///
          /// \details    Performs a 1D interpolation to find a Y value given an input of an x location withing a 1D
          ///             sample space. Clamps to nearest available data if input range is out of bounds for x w.r.t LUT 
          ///             data. 
          /// \param[in]  xv    x values of the LUT
          /// \param[in]  yv    y values of the LUT
          /// \param[in]  x     x location to be used in calculation of y
          /// \param      y     y variable to contain interpolation output. 
          ///
          /// \return     -1 if clamped due to input out of bounds 
          /// X values must be ascending
          int interpolate(std::vector<float> xv, std::vector<float> yv, float x, float &y);
          


          // Function to perfrom 2D (Bilinear) interpolation.           
          ///
          /// \brief      Performs 2D interpolations on LUT 
          ///
          /// \details    Performs a 2D linear interpolation to find a Z value given an input of x and y locations within a 2D 
          ///             sample space. Clamps to nearest available data if input range is out of bounds for x and y w.r.t LUT 
          ///             data. 
          /// \param[in]      xv    x values of the 2D LUT
          /// \param[in]      yv    Y values of the 2D LUT
          /// \param[in]      zv    Z values of the 2D LUT
          /// \param[in]      x     x location to be used in calculation of z
          /// \param[in]      y     y location to be used in calculation of z
          /// \param[in/out]  z     z value to contain interpolation output.    
          ///
          /// \return     Returns 0 if interpolation is successful, -1 if clamped due to input out of bounds 
          ///             and clamped to nearest available data. 
          /// X and Y values must be ascending
          int interpolate2D(std::vector<std::vector<float>> xv, std::vector<float> yv, std::vector<std::vector<float>> zv, float x, float y, float &z);

          ///
          /// \brief      Performs 2D interpolations on LUT based on x, y, and z values stored within the class. 
          ///
          /// \details    Performs a 2D linear interpolation to find a Z value given an input of x and y locations within a 2D 
          ///             sample space. Clamps to nearest available data if input range is out of bounds for x and y w.r.t LUT 
          ///             data. x, y, and z LUT values must be loaded into the class prior to calling this function, either via
          ///             the constructor method, or via the set[X/Y/Z]Vals functions. 
          /// \param[in]      x     x location to be used in calculation of z
          /// \param[in]      y     y location to be used in calculation of z
          /// \param[in/out]  z     z value to contain interpolation output.   
          ///
          /// \return     Returns 0 if interpolation is successful, -1 if clamped due to input out of bounds , -2 if LUT data is not present
          ///             and clamped to nearest available data. 
          /// X and Y values must be ascending
          int interpolate2D(float, float, float&);


          
          // Static helper functions to load data elements into vectors. 
          ///
          /// \brief      Gets a one dimensional vector of LUT elements from a string.
          /// \details    Takes a string containing comma separated values representing values 
          ///            withing a LUT. 
          ///
          /// \param[in]  inputVals The input string to be processed. 
          ///
          /// \return     A std::vector<float> containing the LUT elements. 
          ///
          static std::vector<float> get1DLUTelementsFromString(std::string inputVals);

          ///
          /// \brief      Gets a two dimensional vector of LUT elements from a string. 
          /// \details    Takes a string containing semicolon separated rows of comma separated
          ///            values representing points on a 2D LUT. 
          ///
          /// \param[in]  inputVals  The input string to be processed. 
          ///
          /// \return     A std::vector<std::vector<float>> containing the 2D LUT elements. 
          ///
          static std::vector<std::vector<float>> get2DLUTelementsFromString(std::string inputVals);
          


          ///
          /// \brief      Sets the x vals vector.
          ///
          /// \param[in]  inputVals  String to be parsed for input values. 
          ///
          void setXVals(std::string inputVals); 

          ///
          /// \brief      Sets the x vals vector.
          ///
          /// \param[in]  xv         Vector containing the x values. 
          ///
          void setXVals(std::vector<std::vector<float>> xv);

          ///
          /// \brief      Sets the y vals vector.
          ///
          /// \param[in]  inputVals  String to be parsed for input values. 
          ///
          void setYVals(std::string); 

          ///
          /// \brief      Sets the y vals vector.
          ///
          /// \param[in]  yv         Vector containing the y values. 
          ///
          void setYVals(std::vector<float> yv);


          ///
          /// \brief      Sets the z vals vector.
          ///
          /// \param[in]  inputVals  String to be parsed for input values. 
          ///
          void setZVals(std::string inputVals); 

          ///
          /// \brief      Sets the z vals vector.
          ///
          /// \param[in]  zv         Vector containing the z values. 
          ///
          void setZVals(std::vector<std::vector<float>> zv);

        private: 
          /// \brief Vectors to hold LUT data.
          /// Holds 2D LUT values for X and Z points
          std::vector< std::vector<float> > xVals, zVals;
          std::vector<float> yVals; ///< Holds Y values for LUT data.

          /// Bools to keep track of which LUT data values have been added. 
          bool haveXVals, haveYVals, haveZVals; 
    };
}