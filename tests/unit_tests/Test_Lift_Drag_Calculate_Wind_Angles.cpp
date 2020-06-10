/**
 * @brief       Lift_drag_model::calculateFreestreamVelocity Test Class
 * @file        Test_Lift_Drag_Set_Get_Freestream_Velocity.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragCalculateWindAnglesParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragCalculateWindAnglesParameterized, LiftDragCalculateWindAngles_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	
	ignition::math::Vector3d worldVel;
	ignition::math::Pose3d pose;
	ignition::math::Vector3d forward;
	ignition::math::Vector3d upward;
	double correctAlpha;
	double correctBeta;

	LiftDragCalculateWindAnglesParams param=GetParam();
	worldVel=param.inputWorldVel;
	pose=param.inputPose;
	correctAlpha=param.correctAlpha;
	correctBeta=param.correctBeta;
	upward=param.vUpwd;
	forward=param.vFwd;
	try
	{
		ldm.setWorldPose(pose);
		ldm.setWorldVelocity(worldVel);
		ldm.setForwardVector(forward);
		ldm.setUpwardVector(upward);
		ldm.calculatePortVector();
		ldm.calculateWindAnglesAndLocalVelocities();

		//std::cout<<"Alpha="<<ldm.convertDegreesToRadians(ldm.getAlpha())<<", beta="<<ldm.convertDegreesToRadians(ldm.getBeta())<<std::endl;
		
		ASSERT_NEAR(ldm.convertDegreesToRadians(ldm.getAlpha()),correctAlpha,1e-01);
		ASSERT_NEAR(ldm.convertDegreesToRadians(ldm.getBeta()),correctBeta,1e-01);
		//SUCCEED();
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for calculateLocalVelocities."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}

