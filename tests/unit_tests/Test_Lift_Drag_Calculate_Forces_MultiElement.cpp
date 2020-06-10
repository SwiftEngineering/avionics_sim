/**
 * @brief       Lift_drag_model::calculateLiftDragModelValues Test Class
 * @file        Test_Lift_Drag_Calculate_Forces.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "LiftDragCalculateForcesMultiElementParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragCalculateMultiElementForcesParameterized, LiftDragCalculateForcesMultiElement_UnitTest) {

    LiftDragCalculateMultiElementForcesParams param = GetParam();

    avionics_sim::Lift_drag_model ldm;
    const double tolerance=1.0;
	double rho;
	ignition::math::Vector3d inputWorldVel;
	ignition::math::Pose3d inputPose;
	ignition::math::Vector3d vFwd;
	ignition::math::Vector3d vUpwd;
    double area, motorExitVelocity, controlAlpha;
	std::vector<double> LUT_alpha;
	std::vector<double> LUT_CL;
	std::vector<double> LUT_CD;
    ignition::math::Vector3d trueLift;
	ignition::math::Vector3d trueDrag;
	ignition::math::Vector3d trueLateralForce;
    bool hasMotorExitVelocity;
    bool isControlSurface;
    ignition::math::Vector3d force = ignition::math::Vector3d(0.0,0.0,0.0);
    bool debug=false;
    double vInf;

    for (LiftDragCalculateForcesParams &v : param.elementData)
    {
        rho=v.rho;
        inputWorldVel=v.inputWorldVel;
        inputPose=v.inputPose;
        vUpwd=v.vUpwd;
        vFwd=v.vFwd;
        motorExitVelocity=v.motorExitVelocity;
        area=v.area;
        LUT_alpha=v.LUT_alpha;
        LUT_CL=v.LUT_CL;
        LUT_CD=v.LUT_CD;
        trueLift=v.trueLift;
        trueDrag=v.trueDrag;
        trueLateralForce=v.trueLateralForce;
        hasMotorExitVelocity=v.hasMotorExitVelocity;
        isControlSurface=v.isControlSurface;
        controlAlpha=v.controlAlpha;
        vInf=v.vInf;
        ignition::math::Vector3d elementForce = ignition::math::Vector3d(0.0,0.0,0.0);

        try
        {
            ldm.setWorldPose(inputPose);
            ldm.setWorldVelocity(inputWorldVel);
            ldm.setForwardVector(vFwd);
            ldm.setUpwardVector(vUpwd);
            ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
            ldm.calculatePortVector();
            ldm.calculateWindAnglesAndLocalVelocities();
            ldm.setAirDensity(rho);
            ldm.setArea(area);
            ldm.setPlanarVelocity(vInf);

            if (debug)
            {
                std::cout<<"Link name: "<<v.linkName<<std::endl;
                std::cout<<"World pose: "<<ldm.getWorldPose()<<std::endl;
                std::cout<<"World velocity: "<<ldm.getWorldVelocity()<<std::endl;
                std::cout<<"Forward vector: "<<ldm.getForwardVector()<<std::endl;
                std::cout<<"Upward vector: "<<ldm.getUpwardVector()<<std::endl;
                std::cout<<"Port vector: "<<ldm.getPortVector()<<std::endl;
                std::cout<<"Area: "<<ldm.getArea()<<std::endl;
                std::cout<<"Air density: "<<ldm.getAirDensity()<<std::endl;
                std::cout<<"LUT CD: "<<ldm.getAirDensity()<<std::endl;
                std::cout<<"LUT alpha values: "<<std::endl;
                for (double &LUTAlpha : ldm.getLUTAlpha())
                {
                    std::cout<<LUTAlpha<<" ";
                }
                std::cout<<std::endl;
                std::cout<<"LUT CD: "<<std::endl;
                for (double &LUTCD : ldm.getLUTCD())
                {
                    std::cout<<LUTCD<<" ";
                }
                std::cout<<std::endl;
                std::cout<<"LUT CL: "<<std::endl;
                for (double &LUTCL : ldm.getLUTCL())
                {
                    std::cout<<LUTCL<<" ";
                }
                std::cout<<std::endl;
            }

            if(hasMotorExitVelocity)
            {
                // If motor exit velocity is greater than the calculated one, use motor exit velocity instead of freestream velocity.

                if (debug)
                {
                    std::cout<<"Element has motor exit velocity. Motor exit velocity: "<<motorExitVelocity<<", freestream velocity: "<<ldm.getFreeStreamVelocity()<<std::endl;
                }
                
                if (motorExitVelocity > ldm.getFreeStreamVelocity())
                {
                    ldm.setFreeStreamVelocity(motorExitVelocity);
                    std::cout<<"Overriding freestream velocity! "<<std::endl;
                }

                //Set the alpha under propwash to zero.
                ldm.setAlpha(0,true);

                if (debug)
                {
                    std::cout<<"Overriding alpha with value of zero because there is a motor exit velocity."<<std::endl;
                }
                
            }

            // If this is a control surface, set alpha from control surface deflection angle
            if (isControlSurface)
            {
                ldm.setAlpha(controlAlpha);

                if (debug)
                {
                    std::cout<<"Element is a control surface. Overriding calculated alpha with provided value of "<<controlAlpha<<std::endl;
                }
                
            }

            bool calculateRotatedForces=!isControlSurface;

            ldm.calculateLiftDragModelValues(calculateRotatedForces);
            
            elementForce=ldm.getForceVector();

            if (debug)
            {
                std::cout<<"vPlanar: "<<ldm.getPlanarVelocity()<<std::endl;

                std::cout<<"vFreestream: "<<ldm.getFreeStreamVelocity()<<std::endl;

                std::cout<<"Alpha (in radians): "<<ldm.convertDegreesToRadians(ldm.getAlpha())<<std::endl;

                std::cout<<"Alpha: "<<ldm.getAlpha()<<std::endl;

                std::cout<<"Dynamic pressure: "<<ldm.getDynamicPressure()<<std::endl;

                std::cout<<"CL: "<<ldm.getCL()<<std::endl;

                std::cout<<"CD: "<<ldm.getCD()<<std::endl;

                std::cout<<"Element force calculated: "<<elementForce<<std::endl;
            }
            

            //Add the element force to the running tally.
            force = force + elementForce;
        }
        catch(const Lift_drag_model_exception& e)
        {
            std::cerr<<"Exception successfully caught for calculateLiftDragModelValues in test. A function within has caused an exception."<<std::endl;
            std::cerr << e.what() <<std::endl;
        }
        
    }

    std::cout<<"Final calculated force: "<<force<<std::endl;
    std::cout<<"Summed true force: "<<param.summedTrueForce<<std::endl;
    
    ASSERT_NEAR(force.X(),param.summedTrueForce.X(),tolerance);
    ASSERT_NEAR(force.Y(),param.summedTrueForce.Y(),tolerance);
    ASSERT_NEAR(force.Z(),param.summedTrueForce.Z(),tolerance);
}
