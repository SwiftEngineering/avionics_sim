/**
 * @brief       Lift_drag_model
 * @file        Lift_drag_model.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Coordinate_Utils.hpp"

#include <sstream> 
#include <cstdlib> 
#include <iostream>
#include <fstream> 
#include <iterator>
#include <cmath>

#define DEGREES_IN_RADIANS 57.2958

namespace avionics_sim
{
    Lift_drag_model::Lift_drag_model() : 
        alpha(0.0), 
        vInf(0.0), 
        area(1.0), 
        cl(0.0), 
        cd(0.0), 
        lift(0.0), 
        drag(0.0),
        rho(1.225),
        q(0.0),
        AeroInterp(avionics_sim::Bilinear_interp()){}

    void Lift_drag_model::setAlpha(double _alpha, bool isInRadians)
    {
        if (isInRadians)
        {
            alpha=convertRadiansToDegrees(_alpha);
        }
        else
        {
            alpha=_alpha;
        }
    }

    double Lift_drag_model::getAlpha()
    {
        return alpha;
    }

    void Lift_drag_model::setSpeed(double _speed)
    {
        vInf=_speed;
    }

    double Lift_drag_model::getSpeed()
    {
        return vInf;
    }

    void Lift_drag_model::setArea(double _area)
    {
        area=_area;
    }

    double Lift_drag_model::getArea()
    {
        return area;
    }

    void Lift_drag_model::setLUTs(const std::vector<float>& alphas, const std::vector<float>& cls, const std::vector<float>& cds)
    {
	//Clear out the vectors if they are not empty.
        if (!Aero_LUT_alpha.empty())
        {
            Aero_LUT_alpha.clear();
        }
        if (!Aero_LUT_CL.empty())
        {
            Aero_LUT_CL.clear();
        }
        if (!Aero_LUT_CD.empty())
        {
            Aero_LUT_CD.clear();
        }

        if (!alphas.empty())
	{
	    Aero_LUT_alpha.reserve(alphas.size());
  	    copy(alphas.begin(),alphas.end(),std::back_inserter(Aero_LUT_alpha));
	}

        if (!cls.empty())
	{
	    Aero_LUT_CL.reserve(cls.size());
  	    copy(cls.begin(),cls.end(),std::back_inserter(Aero_LUT_CL));
	}

        if (!alphas.empty())
	{
	    Aero_LUT_CD.reserve(cds.size());
  	    copy(cds.begin(),cds.end(),std::back_inserter(Aero_LUT_CD));
	}
    }

    void Lift_drag_model::setAirDensity(double _rho)
    {
        rho=_rho;
    }

    double Lift_drag_model::getAirDensity()
    {
        return rho;
    }

    float Lift_drag_model::getCL()
    {
        return cl;
    }

    float Lift_drag_model::getCD()
    {
        return cd;
    }

    void Lift_drag_model::lookupCL()
    {
        AeroInterp.interpolate(Aero_LUT_alpha,Aero_LUT_CL, alpha, &cl);
    }

    void Lift_drag_model::lookupCD()
    {
         AeroInterp.interpolate(Aero_LUT_alpha,Aero_LUT_CD, alpha, &cd);
    }

    void Lift_drag_model::calculateLift()
    {
        lift=cl*q*getArea();
    }

    void Lift_drag_model::calculateDrag()
    {
        drag=cd*q*getArea();
    }

    double Lift_drag_model::getLift()
    {
        return lift;
    }

    double Lift_drag_model::getDrag()
    {
        return drag;
    }

    double Lift_drag_model::convertRadiansToDegrees(double _angleRadians)
    {
        return _angleRadians * DEGREES_IN_RADIANS;
    }

    void Lift_drag_model::calculateDynamicPressure()
    {
        q=0.5*rho*vInf*vInf;
    }

    double Lift_drag_model::getDynamicPressure()
    {
        return q;
    }

    void Lift_drag_model::calculateLiftDragModelValues()
    {
        calculateDynamicPressure();
        lookupCL();
        lookupCD();
        calculateLift();
        calculateDrag();
    }

    void Lift_drag_model::calculateAlpha(ignition::math::Pose3d wingPose, ignition::math::Vector3d worldVel, double * const alpha_p, ignition::math::Vector3d * const vInf_p)
    {
        ignition::math::Vector3d vInf; // vInf = -world velocity
        ignition::math::Vector3d wingFrameVelocity; // Vector 3 of world linear velocity in wing frame. 
        ignition::math::Vector3d vInLDPlane_v; // Vector 3 of velocity in LD plane. 
        double alpha; 

        double vInLDPlane_s; // Scalar magnitude of speed in LD plane. 

        // Vinf is in the opposite direction as world velocity. 
        vInf = ignition::math::Vector3d(worldVel.X(), worldVel.Y(), worldVel.Z());

        // Calculate world linear velocity in wing coordinate frame. 
        avionics_sim::Coordinate_Utils::project_vector_global(wingPose, vInf, &wingFrameVelocity); 

        // Remove spanwise component
        vInLDPlane_v = ignition::math::Vector3d(-wingFrameVelocity.X(), 0, wingFrameVelocity.Z()); 

        vInLDPlane_s = vInLDPlane_v.Length(); // Calculate scalar

        this->setSpeed(vInLDPlane_s); // Set velocity. 

        // Calculate alpha
        alpha = atan2(wingFrameVelocity.X(), wingFrameVelocity.Z()); 

        this->setAlpha(alpha); // Set alpha based on calculation. 

        // Set value for alpha_p if provided. 
        if(alpha_p != NULL) 
        {
            // Calculate alpha
            *alpha_p = alpha; 
        }

        // Set value for vInf_p if provided
        if(vInf_p != NULL)
        {
            *vInf_p = vInf; 
        }
    }
}
