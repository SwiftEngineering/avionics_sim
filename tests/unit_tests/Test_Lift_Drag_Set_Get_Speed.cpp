/**
 * @brief       Lift_drag_model::setSpeed Test Class
 * @file        Test_Lift_Drag_Set_Get_Speed.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragSetSpeedParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragSetSpeedParameterized, LiftDragUnitTestSetGetSpeed_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double valueToTestAgainst;
	LiftDragSetSpeedParams param=GetParam();
	valueToTestAgainst=param.value;
	double inputSpeed=param.value;
	try
	{
		ldm.setSpeed(inputSpeed);
		ASSERT_FLOAT_EQ(ldm.getSpeed(), valueToTestAgainst);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for setSpeed."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}