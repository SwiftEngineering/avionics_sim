/**
 * @brief       Bilinear_interp
 * @file        Bilinear_interp.cpp
 * @author      Evan Johnson <erjohnson227@gmail.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "avionics_sim/Bilinear_interp.hpp"

#include <sstream> 
#include <cstdlib> 
#include <iostream>
#include <fstream> 


namespace avionics_sim
{
    Bilinear_interp::Bilinear_interp()
    {
        haveXVals = false; 
        haveYVals = false; 
        haveZVals = false; 
    }

    Bilinear_interp::Bilinear_interp(const std::vector<std::vector<float>>* xv, const std::vector<float>* yv, const std::vector<std::vector<float>>* zv)
    {
        xVals = *xv; 
        yVals = *yv; 
        zVals = *zv; 

        haveXVals = true; 
        haveYVals = true; 
        haveZVals = true; 
    }

    Bilinear_interp::InterpResult Bilinear_interp::interpolate(const std::vector<float>& xv, const std::vector<float>& yv, float x, float* const y)
    {
        float x_l, y_l, x_u, y_u; 

        // Check to make sure value is within bounds
        if(x >= xv.front())
        {
            if(x <= xv.back())
            {
                // Find out which indices to interpolate from. 
                std::vector<float>::const_iterator lowBound = lower_bound(xv.begin(), xv.end(), x);
                int i_u = std::distance(xv.begin(), lowBound); 

                // Otherwise we would interpolate between x[-1] and x[0]
                if(x == xv.front())
                    i_u++; 

                // Get the X and Y values. 
                x_l = xv[i_u-1];
                x_u = xv[i_u];
                y_l = yv[i_u-1]; 
                y_u = yv[i_u]; 

                // Perform linear interpolation
                *y = (x-x_l)/(x_u-x_l)*(y_u-y_l) + y_l;

                // Interpolation success. 
                return InterpResult::INTERP_SUCCESS; 
            } else{
                *y = yv.back(); 
                return InterpResult::INTERP_WARN_OUT_OF_BOUNDS; 
            }
        } else {
            *y = yv.front(); 
            return InterpResult::INTERP_WARN_OUT_OF_BOUNDS;
        }
    }

    Bilinear_interp::InterpResult Bilinear_interp::interpolate2D(const std::vector<std::vector<float>>& xv, const std::vector<float>& yv, const std::vector<std::vector<float>>& zv, float x, float y, float* const z)
    {
        float y_l, y_u;
        float z_l = 0.0f; 
        float z_u = 0.0f;
        InterpResult errorState = InterpResult::INTERP_SUCCESS; 

        float y_clamped; // Bound using first and last points of LUT. Clamp if out of range, but throw an error. 

        // Sanity check and bounding
        if(y < yv.front())
        {
            y_clamped = yv.front();
            errorState = InterpResult::INTERP_WARN_OUT_OF_BOUNDS; 
        } else if(y > yv.back())
        {
            y_clamped = yv.back(); 
            errorState = InterpResult::INTERP_WARN_OUT_OF_BOUNDS; 
        } else 
        {
            y_clamped = y; 
        }

        // Find out which y values are bounding our input point
        std::vector<float>::const_iterator lowBound = lower_bound(yv.begin(), yv.end(), y_clamped);
        int iy_u = std::distance(yv.begin(), lowBound);

        // Otherwise we would interpolate between x[-1] and x[0]
        if(y_clamped == yv.front())
            iy_u++; 

        if(y_clamped != yv.front() && y_clamped != yv.back())
        {
            // There are two curves to work with

            // Get the curve value pairs.
            if(interpolate(xv[iy_u-1], zv[iy_u-1], x, &z_l) != InterpResult::INTERP_SUCCESS)
            {
                // Error has occured in interpolation
                errorState = InterpResult::INTERP_WARN_OUT_OF_BOUNDS; 
            }
            if(interpolate(xv[iy_u], zv[iy_u], x, &z_u) != InterpResult::INTERP_SUCCESS)
            {
                // Error has occured in interpolation
                errorState = InterpResult::INTERP_WARN_OUT_OF_BOUNDS; 
            }

            // Y bounding values
            y_l = yv[iy_u-1]; 
            y_u = yv[iy_u];

            // Perform interpolation
            *z = (y-y_l)/(y_u-y_l)*(z_u-z_l) + z_l;
        }  else if (y_clamped == yv.front())
        {
            // Only have the first curve to work with. 
            if(interpolate(xv[iy_u-1], zv[iy_u-1], x, &z_l) != InterpResult::INTERP_SUCCESS)
            {
                errorState = InterpResult::INTERP_WARN_OUT_OF_BOUNDS; // Threw an error. 
            }
            *z = z_l; // No second interpolation
        }   else  {
            // Only have the last curve to work with.
            if(interpolate(xv[iy_u], zv[iy_u], x, &z_l) != InterpResult::INTERP_SUCCESS)
            {
                errorState = InterpResult::INTERP_WARN_OUT_OF_BOUNDS; // Threw an error. 
            }
            *z = z_l; // No second interpolation
        }

        return errorState; 
    }

    Bilinear_interp::InterpResult Bilinear_interp::interpolate2D(float x, float y, float* const z)
    {
        if(!haveXVals || !haveYVals || !haveZVals)
            return InterpResult::INTERP_ERROR_NO_LUT;

        // Have LUT data. Perform interpolation. 
        return interpolate2D(xVals, yVals, zVals, x, y, z);
    }

    bool Bilinear_interp::setXVals(const std::string& inputVals)
    {
        if(!Bilinear_interp::get2DLUTelementsFromString(inputVals, &xVals))
            return false;  // Parsing error 
        
        haveXVals = true; 
        
        return true; // Success
    }

    void Bilinear_interp::setXVals(const std::vector<std::vector<float>>& xv)
    {
        xVals = xv; 
        haveXVals = true; 
    }

    bool Bilinear_interp::setYVals(const std::string& inputVals)
    {
        if(!Bilinear_interp::get1DLUTelementsFromString(inputVals, &yVals))
            return false; // Parsing error 

        haveYVals = true; 

        return true; // Success
    }

    void Bilinear_interp::setYVals(const std::vector<float>& yv)
    {
        yVals = yv; 
        haveYVals = true; 
    }

    bool Bilinear_interp::setZVals(const std::string& inputVals)
    {
        if(!Bilinear_interp::get2DLUTelementsFromString(inputVals, &zVals))
            return false; // Parsing error  

        haveZVals = true; 

        return true; // Success
    }

    void Bilinear_interp::setZVals(const std::vector<std::vector<float>>& zv)
    {
        zVals = zv; 
        haveZVals = true; 
    }

    bool Bilinear_interp::getX(std::vector<std::vector<float>>* const xVect)
    {
        // Return false if values havent been set. 
        if(!haveXVals)
            return false; 
        
        *xVect = xVals; 
        return true; 
    }

    bool Bilinear_interp::getY(std::vector<float>* const yVect)
    {
        // Return false if values havent been set. 
        if(!haveYVals)
            return false; 
        
        *yVect = yVals; 
        return true; 
    }

    bool Bilinear_interp::getZ(std::vector<std::vector<float>>* const zVect)
    {
        // Return false if values havent been set. 
        if(!haveZVals)
            return false; 
        
        *zVect = zVals; 
        return true; 
    }

    bool Bilinear_interp::get1DLUTelementsFromString(const std::string& inputVals, std::vector<float>* const outputVect)
    {
        // Return failure if null string. 
        if(inputVals.empty())
            return false; 

        // Explicit copy operation.
        std::string inputString = inputVals; 

        std::string delimiter = ",";
        size_t pos = 0; 
        std::string token; 

        std::vector<float> row; 

        float tmpFloat; 

        while((pos = inputString.find(delimiter)) != std::string::npos)
        {
            token = inputString.substr(0,pos);  

            if(sscanf(token.c_str(), "%f", &tmpFloat) != 1)
                return false; // Return fault if sscanf fails
                
            row.push_back(tmpFloat); 
            inputString.erase(0, pos + delimiter.length()); 
        }

        // Push in last value; 
        if(sscanf(inputString.c_str(), "%f", &tmpFloat) != 1)
            return false; // Return fault if sscanf fails

        row.push_back(tmpFloat); 

        // Copy to output vector
        *outputVect = row; 

        return true; 
    }

    bool Bilinear_interp::get2DLUTelementsFromString(const std::string& inputVals, std::vector<std::vector<float>>* const outputVect)
    {
        // Return failure if null string. 
        if(inputVals.empty())
            return false; 

        std::stringstream ss(inputVals); 
        std::string to; 

        std::string delimiter = ",";
        size_t pos = 0; 
        std::string token; 

        std::vector<std::vector<float>> vect_vals; 
        std::vector<float> row; 

        float tmpFloat; 

        while(std::getline(ss, to, ';')){
            while((pos = to.find(delimiter)) != std::string::npos)
            {
                token = to.substr(0,pos); 
                if(sscanf(token.c_str(), "%f", &tmpFloat) != 1)
                    return false; // Return fault if sscanf fails
                row.push_back(tmpFloat); 
                to.erase(0, pos + delimiter.length()); 
            }
            // Push in last value; 
            if(sscanf(to.c_str(), "%f", &tmpFloat) != 1)
                return false; // Return fault if sscanf fails

            row.push_back(tmpFloat); 
            vect_vals.push_back(row); 
            row.clear(); 
        }

        // Copy to output vector
        *outputVect = vect_vals; 

        return true; 
    }
}