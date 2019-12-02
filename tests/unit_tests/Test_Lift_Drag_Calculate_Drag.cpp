/**
 * @brief       Lift_drag_model::setAirDensity Test Class
 * @file        Test_Lift_Drag_Set_Get_Air_Density.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragCalculateDragParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragCalculateDragParameterized, LiftDragCalculateDrag_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double vInf;
	double rho;
	double angle;
	std::vector<float> LUT_alpha; 
	std::vector<float> LUT_CL; 
	std::vector<float> LUT_CD; 

	LiftDragCalculateDragParams param=GetParam();
	vInf=param.vInf;
	rho=param.rho;
	angle=param.angle;
	LUT_alpha=param.LUT_alpha;
	LUT_CL=param.LUT_CL;
	LUT_CD=param.LUT_CD;

	/*
	Use a try-catch in testing composite functions (functions that call other methods to get values required to compute others). Here, the test will pass if if there is an exception or not, but we want also to indicate that the exception was successfully caught when one of the functions within the composite throws an exception.
	*/
	try
	{
		ldm.setSpeed(vInf);
		ldm.setAirDensity(rho);
		ldm.setAlpha(angle);
		ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
		ldm.lookupCD();
		ldm.calculateDrag();
		ASSERT_TRUE((ldm.getDrag() >= avionics_sim::Lift_drag_model::MIN_DRAG) && (ldm.getDrag() <= avionics_sim::Lift_drag_model::MAX_DRAG));
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for calculateDrag. A function within has caused an exception."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}