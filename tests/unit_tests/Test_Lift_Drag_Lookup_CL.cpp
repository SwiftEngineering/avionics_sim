/**
 * @brief       Lift_drag_model::lookupCL Test Class
 * @file        Test_Lift_Drag_Lookup_CL.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragLookupCLParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragLookupCLParameterized, LiftDragUnitTestLookupCL_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double inputAngle;
	std::vector<double> LUT_alpha; 
	std::vector<double> LUT_CL; 
	std::vector<double> LUT_CD; 
	LiftDragLookupCLParams param=GetParam();
	inputAngle=param.value;
	LUT_alpha=param.LUT_alpha;
	LUT_CL=param.LUT_CL;
	LUT_CD=param.LUT_CD;
	try
	{
		ldm.setAlpha(inputAngle);
		ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
		ldm.lookupCL();
		std::string failStr="CL is "+std::to_string(ldm.getCL());
		ASSERT_TRUE((ldm.getCL() >= avionics_sim::Lift_drag_model::MIN_CL) && (ldm.getCL() <= avionics_sim::Lift_drag_model::MAX_CL))<< failStr;
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for lookupCL. A function within has caused an exception."<<std::endl;
	}
}
