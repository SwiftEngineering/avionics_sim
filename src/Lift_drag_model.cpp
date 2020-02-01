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
        lateral_force(0.0),
        q(0.0),
        AeroInterp(avionics_sim::Bilinear_interp()),
        isControlSurface(false),
        beta(0.0),
        lateralArea(0.0),
        lateral_velocity(0.0){}

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
        const double LOWER_ALPHA_BOUND=-180.0;
        const double UPPER_ALPHA_BOUND=180.0;

        //Check if incoming alpha is NaN. Throw an exception if it is.
        if (isnan(_alpha))
        {
            std::string errMsg="Lift_drag_model::setAlpha: Angle of attack is NaN";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
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

            /*
            Check if alpha is in [-180, 180]. If not, condition it to be so, then throw an exception. While this should not happen (atan2 guarantees that resulting value is in the range of [-pi, pi]), this check is put into place in case a function that does not use atan2 sets the value of the angle).
            */

            //Employ new method valueIsWithinBounds for better checking (compares with floating point precision)
            std::string errMsg;
            if (!valueIsWithinBounds(alpha, LOWER_ALPHA_BOUND, UPPER_ALPHA_BOUND, "Alpha", errMsg))
            {
                double conditionedAngle=conditionAngle(alpha);
                errMsg="Lift_drag_model::setAlpha: "+errMsg;

                alpha=conditionedAngle;

                errMsg=errMsg+". Angle will be conditioned to a value of "+mu.to_string_with_precision(conditionedAngle, 2)+".";

                Lift_drag_model_exception e(errMsg);
                throw e;
            }
        }
    }

    double Lift_drag_model::getAlpha()
    {
        return alpha;
    }

    void Lift_drag_model::calculateBeta(ignition::math::Pose3d wingPose, ignition::math::Vector3d worldVel, double * const beta_p)
    {
        //TODO: Refactor (combine into calculateAlpha)
        ignition::math::Vector3d vInfbar; // vInf = -world velocity
        ignition::math::Vector3d wingFrameVelocity; // Vector 3 of world linear velocity in wing frame.
        ignition::math::Vector3d vInLDPlane_v; // Vector 3 of velocity in LD plane.

        double beta;

        std::string exceptions="Lift_drag_model::calculateBeta:\n";

        bool exceptionOccurred=false;
        bool criticalExceptionOccurred=false;

        // Vinf is in the opposite direction as world velocity.
        vInfbar = ignition::math::Vector3d(worldVel.X(), worldVel.Y(), worldVel.Z());

        // Calculate world linear velocity in wing coordinate frame.
        avionics_sim::Coordinate_Utils::project_vector_global(wingPose, vInfbar, &wingFrameVelocity);

        // Calculate beta (formula for angle beta derived from excerpt of Flight Dynamics Principles (Third Edition), 2013)
        try
        {
            double y=wingFrameVelocity.Y();
            double x=vInfbar.Length();
            beta = atan2(y,x);
            //std::cout<<"wingFrameVelocity.Y()="<<wingFrameVelocity.Y()<<", vinfbar="<<vInfbar.Length()<<", beta in degrees="<<convertRadiansToDegrees(beta)<<std::endl;
        }
        catch(const std::exception& e)
        {
            //Set angle to zero.;
            beta=0;

            //Package up again as a lift drag exception and throw again.
            std::string errMsg="Lift_drag_model::calculateBeta: domain error for atan";
            Lift_drag_model_exception lde(errMsg, true);
            throw lde;
        }

        // Set value for beta_p if provided.
        if(beta_p != NULL)
        {
            *beta_p = beta;
        }

        //Set beta angle value
        try
        {
            setBeta(beta,true);
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

    void Lift_drag_model::setBeta(double _beta, bool isInRadians)
    {
        const double LOWER_BETA_BOUND=-180.0;
        const double UPPER_BETA_BOUND=180.0;

        std::string errMsg;

        //Check if incoming alpha is NaN. Throw an exception if it is.
        if (isnan(_beta))
        {
            errMsg="Lift_drag_model::setBeta: Side slip angle is NaN";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }

        //Else, set new value, but check if alpha is within bounds when vInf is greater than a certain minimum. Throw an exception if it is not.
        else
        {
            if (isInRadians)
            {
                beta=convertRadiansToDegrees(_beta);
            }
            else
            {
                beta=_beta;
            }

            /*Check if beta is in [-180, 180]. If not, condition it to be so, then throw an exception. While this should not happen (atan2 guarantees that resulting value is in the range of [-pi, pi]), this check is put into place in case a function that does not use atan2 sets the value of the angle).*/

            //Use floating point comparison in DE260
            /*if ((beta<-180)||(beta>180))
            {
                double conditionedAngle=conditionAngle(beta);

                std::string errMsg="Lift_drag_model::setBeta: Beta of "+mu.to_string_with_precision(beta,2)+" is not within bounds of ["+mu.to_string_with_precision(-180,2)+", "+mu.to_string_with_precision(180, 2)+"]. Angle will be conditioned to a value of "+mu.to_string_with_precision(conditionedAngle, 2)+".";

                beta=conditionedAngle;

                Lift_drag_model_exception e(errMsg);
                throw e;
            }*/

            if (!valueIsWithinBounds(beta, LOWER_BETA_BOUND, UPPER_BETA_BOUND, "Beta", errMsg))
            {
                double conditionedAngle=conditionAngle(beta);
                errMsg="Lift_drag_model::setBeta: "+errMsg;

                beta=conditionedAngle;

                errMsg=errMsg+". Angle will be conditioned to a value of "+mu.to_string_with_precision(conditionedAngle, 2)+".";

                Lift_drag_model_exception e(errMsg);
                throw e;
            }
        }

    }

    double Lift_drag_model::getBeta()
    {
        return beta;
    }

    void Lift_drag_model::setSpeed(double _speed)
    {
        std::string errMsg;

        //Check if incoming speed is NaN. Throw a critical exception if it is.
        if (isnan(_speed))
        {
            errMsg="Lift_drag_model::setSpeed: vInf is NaN";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }

        else
        {
            vInf=_speed;

            //Check if vInf is within bounds. If not, store vInf as either MIN_VINF or MAX_VINF if bounds exceeded, then throw an exception.
            if (!valueIsWithinBounds(vInf, MIN_VINF, MAX_VINF, "VInf (speed)", errMsg, true))
            {
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
        std::string errMsg;
        //Check if incoming area is NaN. Throw a critical exception if it is.
        if (isnan(_area))
        {
            errMsg="Lift_drag_model::setArea: area is NaN";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }

        //Check if the area is less than zero. If it is, throw a critical exception.
        else if (_area<0)
        {
            errMsg="Lift_drag_model::setArea: area is less than zero";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }

        else
        {
            area=_area;

            //Check if area is within bounds. If not, store vInf as either MIN_VINF or MAX_VINF if bounds exceeded, then throw an exception.
            if (!valueIsWithinBounds(area, MIN_AREA, MAX_AREA, "Area", errMsg, true))
            {
                Lift_drag_model_exception e(errMsg);
                throw e;
            }

        }
    }

    double Lift_drag_model::getArea()
    {
        return area;
    }

    void Lift_drag_model::setLateralArea(double _area)
    {

        std::string errMsg;

        //Check if incoming area is NaN. Throw a critical exception if it is.
        if (isnan(_area))
        {
            errMsg="Lift_drag_model::setLateralArea: area is NaN";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }

        //Check if the area is less than zero. If it is, throw a critical exception.
        else if (_area<0)
        {
            errMsg="Lift_drag_model::setLateralArea: area is less than zero";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }

        else
        {
            //No minimum or maximum has been specified for lateral area, so just set the value as received.
            lateralArea=_area;
        }
    }

    double Lift_drag_model::getLateralArea()
    {
        return lateralArea;
    }

    void Lift_drag_model::setLUTs(const std::vector<double>& alphas, const std::vector<double>& cls, const std::vector<double>& cds)
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
        std::string errMsg;

        //Check if incoming _rho is NaN. Throw a critical exception if it is.
        if (isnan(_rho))
        {
            errMsg="Lift_drag_model::setAirDensity: rho (air density) is NaN";
            Lift_drag_model_exception e(errMsg, true);
            throw e;
        }
        else
        {
            rho=_rho;

            //Check if air density is within bounds. Throw an exception if it is not.
            if (!valueIsWithinBounds(rho, MIN_AIR_DENSITY, MAX_AIR_DENSITY, "Air density", errMsg, true))
            {
                Lift_drag_model_exception e(errMsg);
                throw e;
            }

        }
    }

    double Lift_drag_model::getAirDensity()
    {
        return rho;
    }

    double Lift_drag_model::getCL()
    {
        return cl;
    }

    double Lift_drag_model::getCD()
    {
        return cd;
    }

    void Lift_drag_model::lookupCL()
    {
        AeroInterp.interpolate(Aero_LUT_alpha,Aero_LUT_CL, alpha, &cl);

        /*
        Check if the lift coefficient is within bounds. Throw an exception if it is not.
        NB: There is no need to test for NaN in the lookup tables, as those values must be of type double for the vector that holds those values.
        */
        //if ( mu.rough_lt(cl, MIN_CL, floatTolerance) || mu.rough_gt(cl, MAX_CL, floatTolerance) )
        std::string errMsg;
        if (!valueIsWithinBounds(rho, MIN_CL, MAX_CL, "CL (Coefficient of Lift)", errMsg))
        {
            Lift_drag_model_exception e(errMsg);
            throw e;
        }
    }

    void Lift_drag_model::lookupCD()
    {
        if (isControlSurface)
        {
            double cdAtAlphaZero;

            //Now, we subtract cd @ alpha=0 from the one obtained at alpha iff alpha not zero.
            AeroInterp.interpolate(Aero_LUT_alpha,Aero_LUT_CD, alpha, &cd);
            AeroInterp.interpolate(Aero_LUT_alpha,Aero_LUT_CD, 0, &cdAtAlphaZero);
            cd=cd-cdAtAlphaZero;
        }
        else
        {
            AeroInterp.interpolate(Aero_LUT_alpha,Aero_LUT_CD, alpha, &cd);
        }
    }

    void Lift_drag_model::calculateLift()
    {
        lift=cl*q*getArea();

        //if ( (mu.rough_lt(lift, MIN_LIFT, tolerance)) || (mu.rough_gt(lift, MAX_LIFT, tolerance)) )
        std::string errMsg;
        if (!valueIsWithinBounds(lift, MIN_LIFT, MAX_LIFT, "Lift", errMsg))
        {
            errMsg=errMsg+"Cl="+mu.to_string_with_precision(cl,16)+", q="+mu.to_string_with_precision(q,16)+", area="+mu.to_string_with_precision(getArea(),16);
            Lift_drag_model_exception e(errMsg);
            throw e;
        }

    }

    void Lift_drag_model::calculateDrag()
    {
        drag=cd*q*getArea();

        //Check if the drag force is within bounds. Throw an exception if it is not.
        std::string errMsg;
        //if ( mu.rough_lt(drag, MIN_DRAG, tolerance) || mu.rough_gt(drag, MAX_DRAG, tolerance) )
        if (!valueIsWithinBounds(drag, MIN_DRAG, MAX_DRAG, "Drag", errMsg))
        {
            errMsg=errMsg+"Cd="+mu.to_string_with_precision(cd,16)+", q="+mu.to_string_with_precision(q,16)+", area="+mu.to_string_with_precision(getArea(),16);;
            Lift_drag_model_exception e(errMsg);
            throw e;
        }
    }

    void Lift_drag_model::calculateLateralForce()
    {
        /*
        Need to convert beta to radians because domain of atan2 is [-pi,pi]and radians are inherently dimensionless (they are the ratio of two lengths).
        */
        double q_lat=q=0.5*rho*lateral_velocity*lateral_velocity;
        lateral_force=q_lat*coefficientLateralForce*convertDegreesToRadians(getBeta())*getLateralArea();

        std::string errMsg;
        if (!valueIsWithinBounds(lateral_force, MIN_LATERAL_FORCE, MAX_LATERAL_FORCE, "Lateral force", errMsg))
        {
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

    double Lift_drag_model::getLateralForce()
    {
        return lateral_force;
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

        AvionicsSimTryCatchBlock cldmvBlock;

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::lookupCL);

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::lookupCD);

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::calculateLift);

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::calculateDrag);

        cldmvBlock.tryCatch(this,&avionics_sim::Lift_drag_model::calculateLateralForce);

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

        //Problem: world velocity as coming in from Gazebo is way too high.
        //std::cout<<"World velocity X="<<worldVel.X()<<", world velocity Y="<<worldVel.Y()<<", world velocity Z="<<worldVel.Z()<<std::endl;

        // Calculate world linear velocity in wing coordinate frame.
        avionics_sim::Coordinate_Utils::project_vector_global(wingPose, vInfbar, &wingFrameVelocity);

        // Remove spanwise and vertical component
        vInLDPlane_v = ignition::math::Vector3d(-wingFrameVelocity.X(), 0, wingFrameVelocity.Z());

        //std::cout<<"Wing frame velocity X="<<-wingFrameVelocity.X()<<", wing frame velocity z="<<wingFrameVelocity.Z()<<std::endl;

        //Examine components. The corresponding length is way too big at times.

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

        try
        {
            alpha = atan2(y,x);
        }
        catch(const std::exception& e)
        {
            //Set angle to zero.
            beta=0;

            //Package up again as a lift drag exception and throw again.
            std::string errMsg="Lift_drag_model::setAlpha: domain error for atan";
            Lift_drag_model_exception lde(errMsg, true);
            throw lde;
        }

        // Set value for alpha_p if provided.
        if(alpha_p != NULL)
        {
            *alpha_p = alpha;
        }

        // Set value for vInf_p if provided
        if(vInf_p != NULL)
        {
            //Was set originally to vInfBar, which is the same as worldVel. Shouldn't wingFrameVelocity be returned?
            *vInf_p = vInfbar;
        }

        //Set alpha
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

    void Lift_drag_model::setControlSurfaceFlag(bool isCSSurface)
    {
        isControlSurface=isCSSurface;
    }

    bool Lift_drag_model::getControlSurfaceFlag()
    {
        return isControlSurface;
    }

    double Lift_drag_model::conditionAngle(double angle_in, double lowerLimit, double upperLimit, int increment)
    {
        double angle_out=angle_in;

        //Sanity check: Make adjustment only if upperLimit<lowerLimit.
        if ( (lowerLimit<upperLimit) )
        {
            while (angle_out > upperLimit)
            {
                angle_out = angle_out - increment ;
            }

            while (angle_out < lowerLimit)
            {
                angle_out = angle_out + increment ;
            }
        }
        return angle_out;
    }

    bool Lift_drag_model::valueIsWithinBounds(double &value, double lowerBound, double upperBound, std::string itemName, std::string &errMsg, bool capToBound)
    {
        bool withinBounds=true;

        bool below=mu.rough_lt(value, lowerBound, tolerance);
        bool above=mu.rough_gt(value, upperBound, tolerance);

        if ( below || above )
        {
            withinBounds=false;

            //Set error message.
            errMsg=itemName+"= "+mu.to_string_with_precision(value,16)+" and is not within bounds of ["+mu.to_string_with_precision(lowerBound,16)+", "+mu.to_string_with_precision(upperBound,16)+"]. ";

            //Set to nearest bound if capToBound.
            if (capToBound)
            {
                if (below)
                {
                    value=lowerBound;
                }
                else if (above)
                {
                    value=upperBound;
                }

                errMsg=errMsg+itemName+" will be constrained to "+mu.to_string_with_precision(value,16)+". ";
            }
        }

        return withinBounds;
    }

        void Lift_drag_model::setLateralVelocity(ignition::math::Pose3d wingPose, ignition::math::Vector3d worldVel)
    {
        ignition::math::Vector3d vInfbar; // vInf = -world velocity
        ignition::math::Vector3d wingFrameVelocity; // Vector 3 of world linear velocity in wing frame.
        ignition::math::Vector3d vInLDPlane_v; // Vector 3 of velocity in LD plane.

        double vInLDPlane_s; // Scalar magnitude of speed in LD plane.

        std::string exceptions="Lift_drag_model::calculateAlpha:\n";

        bool exceptionOccurred=false;
        bool criticalExceptionOccurred=false;

        // Vinf is in the opposite direction as world velocity.
        vInfbar = ignition::math::Vector3d(worldVel.X(), worldVel.Y(), worldVel.Z());

        // Calculate world linear velocity in wing coordinate frame.
        avionics_sim::Coordinate_Utils::project_vector_global(wingPose, vInfbar, &wingFrameVelocity);

        //Set lateral velocity for lateral force calculation (will move code for beta angle in here, hence placed here and not in calculateBeta)
        lateral_velocity = wingFrameVelocity.Y();
    }
}
