/**
 * @brief       Lift_drag_model::calculateLiftDragModelValues Test Class
 * @file        Test_Lift_Drag_Calculate_Forces.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragCalculateForcesParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragCalculateForcesParameterized, LiftDragCalculateForces_UnitTest) {

	avionics_sim::Lift_drag_model ldm;
	const double tolerance=1.0;//5e-01;
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
    bool isNonSpecific;
	
	LiftDragCalculateForcesParams param=GetParam();
    rho=param.rho;
    inputWorldVel=param.inputWorldVel;
    inputPose=param.inputPose;
    vUpwd=param.vUpwd;
	vFwd=param.vFwd;
    motorExitVelocity=param.motorExitVelocity;
    area=param.area;
	LUT_alpha=param.LUT_alpha;
	LUT_CL=param.LUT_CL;
	LUT_CD=param.LUT_CD;
    trueLift=param.trueLift;
	trueDrag=param.trueDrag;
	trueLateralForce=param.trueLateralForce;
    hasMotorExitVelocity=param.hasMotorExitVelocity;
    isControlSurface=param.isControlSurface;
    controlAlpha=param.controlAlpha;
    isNonSpecific=param.isNonSpecific;
    ignition::math::Vector3d rotated_lift = ignition::math::Vector3d(0.0,0.0,0.0);
    ignition::math::Vector3d rotated_drag = ignition::math::Vector3d(0.0,0.0,0.0);
    ignition::math::Vector3d lateral_force = ignition::math::Vector3d(0.0,0.0,0.0);
    ignition::math::Vector3d force = ignition::math::Vector3d(0.0,0.0,0.0);
    ignition::math::Vector3d trueForce = trueLift+trueLateralForce+trueDrag;
    
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

        //Presumed that controlAlpha will be in degrees for lookup. If this changes, update.
        if (isNonSpecific)
        {
            ldm.calculateLiftDragModelValues(true);
        }
        else
        {
            if (isControlSurface)
            {
                ldm.setAlpha(controlAlpha);
            }

            ldm.calculateLiftDragModelValues(isControlSurface);
        }
        
        force=ldm.getForceVector();

        /*std::cout<<"Force calculated from unit test: "<<force<<std::endl<<"True force:"<<trueForce<<std::endl;
        std::cout<<"True lift: "<<trueLift<<std::endl<<"True drag: "<<trueDrag<<std::endl<<"True lateral force:"<<trueLateralForce<<std::endl;*/
        ASSERT_NEAR(force.X(),trueForce.X(),tolerance);
        ASSERT_NEAR(force.Y(),trueForce.Y(),tolerance);
        ASSERT_NEAR(force.Z(),trueForce.Z(),tolerance);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for calculateLiftDragModelValues in test. A function within has caused an exception."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}
