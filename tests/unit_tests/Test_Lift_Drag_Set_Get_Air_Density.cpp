/**
 * @brief       Lift_drag_model::setAirDensity Test Class
 * @file        Test_Lift_Drag_Set_Get_Air_Density.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragSetAirDensityParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragSetAirDensityParameterized, LiftDragUnitTestSetGetAirDensity_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double valueToTestAgainst;
	LiftDragSetAirDensityParams param=GetParam();
	valueToTestAgainst=param.value;
	double inputAirDensity=param.value;
	try
	{
		ldm.setAirDensity(inputAirDensity);
		ASSERT_FLOAT_EQ(ldm.getAirDensity(), valueToTestAgainst);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for setAirDensity."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}