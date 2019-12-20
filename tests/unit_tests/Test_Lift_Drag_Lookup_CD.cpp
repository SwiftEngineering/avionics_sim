/**
 * @brief       Lift_drag_model::setAirDensity Test Class
 * @file        Test_Lift_Drag_Set_Get_Air_Density.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragLookupCDParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragLookupCDParameterized, LiftDragUnitTestLookupCD_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double inputAngle;
	LiftDragLookupCDParams param=GetParam();
	std::vector<double> LUT_alpha; 
	std::vector<double> LUT_CL; 
	std::vector<double> LUT_CD; 
	inputAngle=param.value;
	LUT_alpha=param.LUT_alpha;
	LUT_CL=param.LUT_CL;
	LUT_CD=param.LUT_CD;
	try
	{
		ldm.setAlpha(inputAngle);
		ldm.setLUTs(LUT_alpha, LUT_CL, LUT_CD);
		ldm.lookupCD();
		ASSERT_TRUE((ldm.getCD() >= avionics_sim::Lift_drag_model::MIN_CD) && (ldm.getCD() <= avionics_sim::Lift_drag_model::MAX_CD));
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for lookupCD."<<std::endl;
		std::cerr<<e.what();
	}
}
