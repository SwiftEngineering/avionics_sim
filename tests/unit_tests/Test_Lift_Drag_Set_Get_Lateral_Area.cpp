/**
 * @brief       Lift_drag_model::setArea Test Class
 * @file        Test_Lift_Drag_Set_Get_Area.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragSetLateralAreaParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"


TEST_P(LiftDragSetLateralAreaParameterized, LiftDragUnitTestSetGetLateralArea_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double valueToTestAgainst;
	LiftDragSetLateralAreaParams param=GetParam();
	valueToTestAgainst=param.value;
	double inputArea=param.value;
	try
	{
		ldm.setLateralArea(inputArea);
		ASSERT_FLOAT_EQ(ldm.getLateralArea(), valueToTestAgainst);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for setLateralArea."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}