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
          Bilinear_interp(); 

        //   Bilinear_interp(const Bilinear_interp &other) 
        //   {
        //       xVals = other.xVals; 
        //       yVals = other.yVals; 
        //       zVals = other.zVals; 

        //       haveXVals = other.haveXVals; 
        //       haveYVals = other.haveYVals;
        //       haveZVals = other.haveZVals; 
        //   }

          Bilinear_interp(std::vector<std::vector<float>>, std::vector<float>, std::vector<std::vector<float>>);

          // Function to perform 1D (Linear) interpolation. 
          int interpolate(std::vector<float>, std::vector<float>, float, float&);
          
          // Function to perfrom 2D (Bilinear) interpolation. 
          int interpolate2D(std::vector<std::vector<float>>, std::vector<float>, std::vector<std::vector<float>>, float, float, float&);
          int interpolate2D(float, float, float&);
          
          // Static helper functions to load data elements into vectors. 
          static std::vector<float> get1DLUTelementsFromString(std::string inputVals);
          static std::vector<std::vector<float>> get2DLUTelementsFromString(std::string inputVals);
          
          void setXVals(std::string); 
          void setXVals(std::vector<std::vector<float>>);

          void setYVals(std::string); 
          void setYVals(std::vector<float>);

          void setZVals(std::string); 
          void setZVals(std::vector<std::vector<float>>);

        private: 
          // Vectors to hold LUT data. 
          std::vector< std::vector<float> > xVals, zVals; 
          std::vector<float> yVals; 

          bool haveXVals, haveYVals, haveZVals; 
    };
}