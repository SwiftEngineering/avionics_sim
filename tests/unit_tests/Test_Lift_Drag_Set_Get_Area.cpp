/**
 * @brief       Lift_drag_model::setArea Test Class
 * @file        Test_Lift_Drag_Set_Get_Area.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragSetAreaParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"


TEST_P(LiftDragSetAreaParameterized, LiftDragUnitTestSetGetArea_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double valueToTestAgainst;
	LiftDragSetAreaParams param=GetParam();
	valueToTestAgainst=param.value;
	double inputAirDensity=param.value;
	try
	{
		ldm.setArea(inputAirDensity);
		ASSERT_FLOAT_EQ(ldm.getArea(), valueToTestAgainst);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for setArea."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}