/**
 * @brief       Lift_drag_model::calculateFreestreamVelocity Test Class
 * @file        Test_Lift_Drag_Set_Get_Freestream_Velocity.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragCalculateLocalVelocitiesParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragCalculateLocalVelocitiesParameterized, LiftDragCalculateLocalVelocities_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	
	ignition::math::Vector3d worldVel;
	ignition::math::Pose3d pose;
	double correctVinf, correctVPlanar, correctVLateral;

	LiftDragCalculateLocalVelocitiesParams param=GetParam();
	worldVel=param.inputWorldVel;
	pose=param.inputPose;
	correctVinf=param.correctVinf;
	correctVPlanar=param.correctVPlanar;
	correctVLateral=param.correctVLateral;
	try
	{
		/*std::cout<<"Pose received: ("<<pose.Pos().X()<<", "<<pose.Pos().Y()<<", "<<pose.Pos().Z()<<", "<<pose.Rot().Euler().X()<<", "<<pose.Rot().Euler().Y()<<", "<<pose.Rot().Euler().Z()<<")"<<std::endl;
		std::cout<<"Vel received: ("<<worldVel.X()<<", "<<worldVel.Y()<<", "<<worldVel.Z()<<")"<<std::endl;*/
		ldm.setWorldPose(pose);
		ldm.setWorldVelocity(worldVel);
		ldm.calculateLocalVelocities();

		//std::cout<<"Freestream velocity="<<ldm.getFreeStreamVelocity()<<", planar velocity="<<ldm.getPlanarVelocity()<<", lateral velocity="<<ldm.getLateralVelocity()<<std::endl;

		ASSERT_NEAR(ldm.getFreeStreamVelocity(),correctVinf,1e-01);
		ASSERT_NEAR(ldm.getPlanarVelocity(),correctVPlanar,1e-01);
		ASSERT_NEAR(ldm.getLateralVelocity(),correctVLateral,1e-01);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for calculateLocalVelocities."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}

