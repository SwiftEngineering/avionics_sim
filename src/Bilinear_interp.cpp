/**
 * @brief       Bilinear_interp
 * @file        Bilinear_interp.cpp
 * @author      Evan Johnson <erjohnson227@gmail.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "avionics_sim/Bilinear_interp.hpp"

#include <sstream> 
#include <stdlib.h> 


namespace avionics_sim
{
    Bilinear_interp::Bilinear_interp()
    {
        haveXVals = false; 
        haveYVals = false; 
        haveZVals = false; 
    }

    Bilinear_interp::Bilinear_interp(std::vector<std::vector<float>> xv, std::vector<float> yv, std::vector<std::vector<float>> zv)
    {
        xVals = xv; 
        yVals = yv; 
        zVals = zv; 

        haveXVals = true; 
        haveYVals = true; 
        haveZVals = true; 
    }

    int Bilinear_interp::interpolate(std::vector<float> xv, std::vector<float> yv, float x, float &y)
    {
        float x_l, y_l, x_u, y_u; 

        // Check to make sure value is within bounds
        if(x >= xv.front())
        {
            if(x <= xv.back())
            {
                // Find out which indices to interpolate from. 
                std::vector<float>::iterator lowBound = lower_bound(xv.begin(), xv.end(), x);
                int i_u = int(lowBound - xv.begin());

                // Otherwise we would interpolate between x[-1] and x[0]
                if(x == xv.front())
                    i_u++; 

                // Get the X and Y values. 
                x_l = xv[i_u-1];
                x_u = xv[i_u];
                y_l = yv[i_u-1]; 
                y_u = yv[i_u]; 

                // Perform linear interpolation
                y = (x-x_l)/(x_u-x_l)*(y_u-y_l) + y_l;

                // Interpolation success. 
                return 0; 
            } else{
                y = yv.back(); 
                return -1; 
            }
        } else {
            y = yv.front(); 
            return -1;
        }
    }

    int Bilinear_interp::interpolate2D(std::vector<std::vector<float>> xv, std::vector<float> yv, std::vector<std::vector<float>> zv, float x, float y, float &z)
    {
        float y_l, y_u;
        float z_l, z_u = 0.0f;
        bool error = false; 

        float y_clamped; // Bound using first and last points of LUT. Clamp if out of range, but throw an error. 

        // Sanity check and bounding
        if(y < yv.front())
        {
            y_clamped = yv.front();
            error = true; 
        } else if(y > yv.back())
        {
            y_clamped = yv.back(); 
            error = true; 
        } else 
        {
            y_clamped = y; 
        }

        // Find out which y values are bounding our input point
        std::vector<float>::iterator lowBound = lower_bound(yv.begin(), yv.end(), y_clamped);
        int iy_u = int(lowBound - yv.begin());

        // Otherwise we would interpolate between x[-1] and x[0]
        if(y == yv.front())
        iy_u++; 

        if(y_clamped != yv.front() && y_clamped != yv.back())
        {
            // There are two curves to work with

            // Get the curve value pairs.
            if(interpolate(xv[iy_u-1], zv[iy_u-1], x, z_l) < 0)
            {
                // Error has occured in interpolation
                error = true; 
            }
            if(interpolate(xv[iy_u], zv[iy_u], x, z_u) < 0)
            {
                // Error has occured in interpolation
                error = true; 
            }

            // Y bounding values
            y_l = yv[iy_u-1]; 
            y_u = yv[iy_u];

            // Perform interpolation
            z = (y-y_l)/(y_u-y_l)*(z_u-z_l) + z_l;

        }  else if (y_clamped == yv.front())
        {
            // Only have the first curve to work with. 
            if(interpolate(xv[iy_u-1], zv[iy_u-1], x, z_l) < 0)
            {
                error = true; // Threw an error. 
            }
            z = z_l; // No second interpolation
        }   else  {
            // Only have the last curve to work with.
            if(interpolate(xv[iy_u], zv[iy_u], x, z_l) < 0)
            {
                error = true; // Threw an error. 
            }
            z = z_l; // No second interpolation
        }

        if(error)
        {
            return -1; // Errors occured
        }

        return 0; // Success        
    }

    int Bilinear_interp::interpolate2D(float x, float y, float &z)
    {
        if(haveXVals && haveYVals && haveZVals)
        {
            return interpolate2D(xVals, yVals, zVals, x, y, z);
        } else {
            return -2; 
        }
    }

    void Bilinear_interp::setXVals(std::string inputVals)
    {
        xVals = Bilinear_interp::get2DLUTelementsFromString(inputVals); 

        haveXVals = true; 
    }

    void Bilinear_interp::setXVals(std::vector<std::vector<float>> xv)
    {
        xVals = xv; 
    }

    void Bilinear_interp::setYVals(std::string inputVals)
    {
        yVals = Bilinear_interp::get1DLUTelementsFromString(inputVals); 

        haveYVals = true; 
    }

    void Bilinear_interp::setYVals(std::vector<float> yv)
    {
        yVals = yv; 
    }

    void Bilinear_interp::setZVals(std::string inputVals)
    {
        zVals = Bilinear_interp::get2DLUTelementsFromString(inputVals); 

        haveZVals = true; 
    }

    void Bilinear_interp::setZVals(std::vector<std::vector<float>> zv)
    {
        zVals = zv; 
    }

    std::vector<float> Bilinear_interp::get1DLUTelementsFromString(std::string inputVals)
    {
        std::stringstream ss(inputVals); 

        std::string delimiter = ",";
        size_t pos = 0; 
        std::string token; 

        std::vector<float> row; 

        if(!inputVals.empty())
        {
            while((pos = inputVals.find(delimiter)) != std::string::npos)
            {
                token = inputVals.substr(0,pos); 
                row.push_back(atof(token.c_str())); 
                inputVals.erase(0, pos + delimiter.length()); 
            }
        row.push_back(atof(inputVals.c_str())); 
        }

        return row; 
    }

    std::vector< std::vector<float> > Bilinear_interp::get2DLUTelementsFromString(std::string inputVals)
    {
        std::stringstream ss(inputVals); 
        std::string to; 

        std::string delimiter = ",";
        size_t pos = 0; 
        std::string token; 

        std::vector<std::vector<float>> vect_vals; 
        std::vector<float> row; 

        if(!inputVals.empty())
        {
        int lines = 0 ; 
            while(std::getline(ss, to, ';')){
                while((pos = to.find(delimiter)) != std::string::npos)
                {
                    token = to.substr(0,pos); 
                    row.push_back(atof(token.c_str())); 
                    to.erase(0, pos + delimiter.length()); 
                }
                row.push_back(atof(to.c_str())); 
                vect_vals.push_back(row); 
                row.clear(); 
            }
        }

        return vect_vals; 
    }
}