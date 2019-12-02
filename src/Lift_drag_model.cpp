/**
 * @brief       Lift_drag_model
 * @file        Lift_drag_model.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>, Evan Johnson <erjohnson227@gmail.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Coordinate_Utils.hpp"
#include <functional>

/*
Added so can now throw exceptions. Plugin should do a try/catch. Depending on exception message,
Lift Drag Plugin should be able to report a warning or critical error.
*/
#include "avionics_sim/Lift_drag_model_exception.hpp"

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
        rho(1.225),
        area(1.0), 
        cl(0.0), 
        cd(0.0), 
        lift(0.0), 
        drag(0.0),
        q(0.0),
        AeroInterp(avionics_sim::Bilinear_interp()){}
    
    void Lift_drag_model::emptyLUTAndCoefficientVectors()
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
    }

    Lift_drag_model::~Lift_drag_model()
    {
        emptyLUTAndCoefficientVectors();
    }

    void Lift_drag_model::setAlpha(double _alpha, bool isInRadians)
    {

        //Check if incoming alpha is NaN. Throw an exception if it is.
        if (isnan(_alpha))
        {
            std::string errMsg="Lift_drag_model::setAlpha: Angle of attack is NaN";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
            //throw new Lift_drag_model_exception(errMsg,true);
        }

        //Else, set new value, but check if alpha is within bounds when vInf is greater than a certain minimum. Throw an exception if it is not.
        else
        {
            if (isInRadians)
            {
                alpha=convertRadiansToDegrees(_alpha);
            }
            else
            {
                alpha=_alpha;
            }
            if (mu.rough_gte(vInf, MIN_VINF_ALPHA, tolerance))
            {
                if ( (mu.rough_lt(alpha, MIN_AOA, tolerance)) || (mu.rough_gt(alpha, MAX_AOA, tolerance)) )
                {
                    std::string errMsg="Lift_drag_model::setAlpha: Angle of attack is not within bounds of ["+mu.to_string_with_precision(MIN_AOA,16)+", "+mu.to_string_with_precision(MAX_AOA, 16)+"]. Angle of attack= "+mu.to_string_with_precision(alpha,16);
                    //throw new Lift_drag_model_exception(errMsg);
                    Lift_drag_model_exception e(errMsg);
                    throw e;
                }
            }
        }
    }

    double Lift_drag_model::getAlpha()
    {
        return alpha;
    }

    void Lift_drag_model::setSpeed(double _speed)
    {
        //Check if incoming speed is NaN. Throw an exception if it is.
        if (isnan(_speed))
        {
            std::string errMsg="Lift_drag_model::setSpeed: vInf is NaN";
            //throw new Lift_drag_model_exception(errMsg,true);
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }
        
        else
        {
            vInf=_speed;

            //Check if vInf is within bounds. Throw an exception if it is not.
            if ( mu.rough_lt(vInf, MIN_VINF, tolerance) || mu.rough_gt(vInf, MAX_VINF, tolerance) )
            {
                std::string errMsg="vInf (speed) is not within bounds of ["+mu.to_string_with_precision(MIN_VINF,16)+", "+mu.to_string_with_precision(MAX_VINF,16)+"]. vInf= "+mu.to_string_with_precision(vInf,16);
                //throw new Lift_drag_model_exception(errMsg);
                Lift_drag_model_exception e(errMsg);
                throw e;
            }
        }
    }

    double Lift_drag_model::getSpeed()
    {
        return vInf;
    }

    void Lift_drag_model::setArea(double _area)
    {
        //Check if incoming area is NaN. Throw an exception if it is.
        if (isnan(_area))
        {
            std::string errMsg="Lift_drag_model::setArea: area is NaN";
            //throw new Lift_drag_model_exception(errMsg,true);
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }
        
        else
        {
            area=_area;

            //Check if area is within bounds. Throw an exception if it is not.
            if ( mu.rough_lt(area, MIN_AREA, tolerance) || mu.rough_gt(area, MAX_AREA, tolerance) )
            {
                std::string errMsg="Area is not within bounds of ["+mu.to_string_with_precision(MIN_AREA,16)+", "+mu.to_string_with_precision(MAX_AREA,16)+"]. Area= "+mu.to_string_with_precision(area,16);
                //throw new Lift_drag_model_exception(errMsg);
                Lift_drag_model_exception e(errMsg);
                throw e;
            }
        }
    }

    double Lift_drag_model::getArea()
    {
        return area;
    }

    void Lift_drag_model::setLUTs(const std::vector<float>& alphas, const std::vector<float>& cls, const std::vector<float>& cds)
    {
	    //Clear out the vectors if they are not empty.
        emptyLUTAndCoefficientVectors();

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
        //Check if incoming _rho is NaN. Throw an exception if it is.
        if (isnan(_rho))
        {
            std::string errMsg="Lift_drag_model::setAirDensity: rho (air density) is NaN";
            //throw new Lift_drag_model_exception(errMsg,true);
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }
        else
        {
            rho=_rho;
            //Check if air density is within bounds. Throw an exception if it is not.
            if ( mu.rough_lt(rho, MIN_AIR_DENSITY, tolerance) || mu.rough_gt(rho, MAX_AIR_DENSITY, tolerance) )
            {
                std::string errMsg="rho (air density) is not within bounds of ["+mu.to_string_with_precision(MIN_AIR_DENSITY,16)+", "+mu.to_string_with_precision(MAX_AIR_DENSITY,16)+"]. rho= "+mu.to_string_with_precision(rho,16);
                //throw new Lift_drag_model_exception(errMsg);
                Lift_drag_model_exception e(errMsg);
                throw e;
            }
        } 
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

        /*
        Check if the lift coefficient is within bounds. Throw an exception if it is not. 
        NB: There is no need to test for NaN in the lookup tables, as those values must be of type float for the vector that holds those values.
        */
        //if ( (cl<MIN_CL) || (cl>MAX_CL) )
        if ( mu.rough_lt(cl, MIN_CL, floatTolerance) || mu.rough_gt(cl, MAX_CL, floatTolerance) )
        {
            std::string errMsg="CL (Coefficient of Lift) is not within bounds of ["+mu.to_string_with_precision(MIN_CL,16)+", "+mu.to_string_with_precision(MAX_CL,16)+"]. CL= "+mu.to_string_with_precision(cl,16);
            //throw new Lift_drag_model_exception(errMsg);
            Lift_drag_model_exception e(errMsg);
            throw e;
        }
    }

    void Lift_drag_model::lookupCD()
    {
         AeroInterp.interpolate(Aero_LUT_alpha,Aero_LUT_CD, alpha, &cd);

        //Check if the lift coefficient is within bounds. Throw an exception if it is not.
        //if ( (cd<MIN_CD) || (cd>MAX_CD) )
        if ( mu.rough_lt(cd, MIN_CD, floatTolerance) || mu.rough_gt(cd, MAX_CD, floatTolerance) )
        {
            std::string errMsg="CD (Coefficient of Drag) is not within bounds of ["+mu.to_string_with_precision(MIN_CD,16)+", "+mu.to_string_with_precision(MAX_CD,16)+"]. CD= "+mu.to_string_with_precision(cd,16);
            //throw new Lift_drag_model_exception(errMsg);
            Lift_drag_model_exception e(errMsg);
            throw e;
        }
    }

    void Lift_drag_model::calculateLift()
    {
        lift=cl*q*getArea();
        
        if ( (mu.rough_lt(lift, MIN_LIFT, tolerance)) || (mu.rough_gt(lift, MAX_LIFT, tolerance)) )
        {
            std::string errMsg="Lift is not within bounds of ["+mu.to_string_with_precision(MIN_LIFT,16)+", "+mu.to_string_with_precision(MAX_LIFT,16)+"]. Lift= "+mu.to_string_with_precision(lift,16)+", cl="+mu.to_string_with_precision(cl,16)+", q="+mu.to_string_with_precision(q,16)+", area="+mu.to_string_with_precision(getArea(),16);
            //throw new Lift_drag_model_exception(errMsg);
            Lift_drag_model_exception e(errMsg);
            throw e;
        }
        
    }

    void Lift_drag_model::calculateDrag()
    {
        drag=cd*q*getArea();

        //Check if the lift coefficient is within bounds. Throw an exception if it is not.
        //if ( (drag<MIN_DRAG) || (drag>MAX_DRAG) )
        if ( mu.rough_lt(drag, MIN_DRAG, tolerance) || mu.rough_gt(drag, MAX_DRAG, tolerance) )
        {
            std::string errMsg="Drag is not within bounds of ["+mu.to_string_with_precision(MIN_DRAG,16)+", "+mu.to_string_with_precision(MAX_DRAG,16)+"]. Drag= "+mu.to_string_with_precision(drag,16);
            //throw new Lift_drag_model_exception(errMsg);
            Lift_drag_model_exception e(errMsg);
            throw e;
        }
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

    double Lift_drag_model::convertDegreesToRadians(double _angleDegrees)
    {
        return _angleDegrees / DEGREES_IN_RADIANS;
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

        std::string exceptions="Lift_drag_model::calculateLiftDragModelValues:\n";

        /*bool exceptionOccurred=false;
        bool criticalExceptionOccurred=false;*/

        AvionicsSimTryCatchBlock cldmvBlock;

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::lookupCL);

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::lookupCD);

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::calculateLift);

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::calculateDrag);

        //If any exceptions occurred along the way, throw a new exception that has the combined exception history of all the prior exceptions.
        if (cldmvBlock.exceptionHasOccurred())
        {
            Lift_drag_model_exception e(cldmvBlock.getExceptionMessage(), cldmvBlock.exceptionIsCritical());
            throw e;
        }
    }

    void Lift_drag_model::calculateAlpha(ignition::math::Pose3d wingPose, ignition::math::Vector3d worldVel, double * const alpha_p, ignition::math::Vector3d * const vInf_p)
    {
        ignition::math::Vector3d vInfbar; // vInf = -world velocity
        ignition::math::Vector3d wingFrameVelocity; // Vector 3 of world linear velocity in wing frame. 
        ignition::math::Vector3d vInLDPlane_v; // Vector 3 of velocity in LD plane. 
        double alpha; 

        double vInLDPlane_s; // Scalar magnitude of speed in LD plane. 

        std::string exceptions="Lift_drag_model::calculateAlpha:\n";

        bool exceptionOccurred=false;
        bool criticalExceptionOccurred=false;

        // Vinf is in the opposite direction as world velocity. 
        vInfbar = ignition::math::Vector3d(worldVel.X(), worldVel.Y(), worldVel.Z());

        // Calculate world linear velocity in wing coordinate frame. 
        avionics_sim::Coordinate_Utils::project_vector_global(wingPose, vInfbar, &wingFrameVelocity); 

        // Remove spanwise and vertical component
        vInLDPlane_v = ignition::math::Vector3d(wingFrameVelocity.X(), 0, wingFrameVelocity.Z());

        //Original remedy for "egregious reverse velocity" issue. Does not pass integration test. Originally, US 1046 had completely passed until this solution below was put into master without an integration test run. Since PX/Firmware US1046 does not work with the modified solution commented out below this line, this will be retained, despite test failure (avionics_sim does not run integration tests always) to get PX4/Firmware to pass. This will be corrected such that integration and flight tests pass in US1061.
        //vInLDPlane_v = ignition::math::Vector3d(-wingFrameVelocity.Z(), 0, 0);

        //Updated version of solution above that passes integration tests.
        //vInLDPlane_v = ignition::math::Vector3d(-wingFrameVelocity.Z(), 0, wingFrameVelocity.X());

        vInLDPlane_s = vInLDPlane_v.Length(); // Calculate scalar

        try
        {
            setSpeed(vInLDPlane_s); // Set velocity.
        }
        catch(Lift_drag_model_exception& e)
        {
            exceptionOccurred=true;

            if (e.isCritical())
            {
                criticalExceptionOccurred=true;
            }
            
            exceptions=exceptions+e.what()+"\n";
        }

        // Calculate alpha
        double y=wingFrameVelocity.X();
        double x=wingFrameVelocity.Z();
        alpha = atan2(y,x);  

        // Set value for alpha_p if provided. 
        if(alpha_p != NULL) 
        {
            // Calculate alpha
            *alpha_p = alpha; 
        }

        // Set value for vInf_p if provided
        if(vInf_p != NULL)
        {
            //Was set originally to vInfBar, which is the same as worldVel. Shouldn't wingFrameVelocity be returned?
            *vInf_p = wingFrameVelocity;//vInfbar; 
        }

        //Set alpha
        /**/
        try
        {
            setAlpha(alpha,true);
        }
        catch(Lift_drag_model_exception& e)
        {
            exceptionOccurred=true;

            if (e.isCritical())
            {
                criticalExceptionOccurred=true;
            }
            
            exceptions=exceptions+e.what()+"\n";
        }

        //If any exceptions occurred along the way, throw a new exception that has the combined exception history of all the prior exceptions.
        if (exceptionOccurred)
        {
            Lift_drag_model_exception e(exceptions, criticalExceptionOccurred);
            throw e;
        }

    }
}
