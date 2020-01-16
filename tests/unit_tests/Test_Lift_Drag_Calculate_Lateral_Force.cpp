/**
 * @brief       Lift_drag_model::calculateLateralForce Test Class
 * @file        Test_Lift_Drag_Calculate_Lateral_Force.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragCalculateLateralForceParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragCalculateLateralForceParameterized, LiftDragCalculateLateralForce_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double vInf;
	double rho;
	double angle;
	std::vector<double> LUT_alpha; 
	std::vector<double> LUT_CL; 
	std::vector<double> LUT_CD; 

	LiftDragCalculateLateralForceParams param=GetParam();
	vInf=param.vInf;
	rho=param.rho;
	angle=param.angle;
	LUT_alpha=param.LUT_alpha;
	LUT_CL=param.LUT_CL;
	LUT_CD=param.LUT_CD;

	try
	{
		ldm.setSpeed(vInf);
		ldm.setAirDensity(rho);
		ldm.setBeta(angle);
		ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
		ldm.calculateLateralForce();

		//In addition to making sure that the values fall between the minimum and maximum forces, the sign of the force must match the sign of beta if beta is not zero.
		if (angle!=0)
		{
			if (angle<=0)
			{
				ASSERT_TRUE(ldm.getLateralForce() <= 0)<<"Angle: "<<angle<<", force: "<<ldm.getLateralForce()<<std::endl;
			}
			else 
			{
				ASSERT_TRUE(ldm.getLateralForce() >= 0)<<"Angle: "<<angle<<", force: "<<ldm.getLateralForce()<<std::endl;
			}
		}
		ASSERT_TRUE((ldm.getLateralForce() >= avionics_sim::Lift_drag_model::MIN_LATERAL_FORCE) && (ldm.getLateralForce() <= avionics_sim::Lift_drag_model::MAX_LATERAL_FORCE));
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for calculateLateralForce. A function within has caused an exception."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}
