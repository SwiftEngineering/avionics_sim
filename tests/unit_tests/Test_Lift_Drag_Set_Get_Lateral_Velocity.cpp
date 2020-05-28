/**
 * @brief       Lift_drag_model::setLateralVelocity Test Class
 * @file        Test_Lift_Drag_Set_Get_Lateral_Velocity.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragSetVelocityParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragSetVelocityParameterized, LiftDragUnitTestSetGetLateralVelocity_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double valueToTestAgainst;
	LiftDragSetVelocityParams param=GetParam();
	valueToTestAgainst=param.value;
	double inputSpeed=param.value;
	try
	{
		ldm.setLateralVelocity(inputSpeed);
		ASSERT_FLOAT_EQ(ldm.getLateralVelocity(), valueToTestAgainst);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for setLateralVelocity."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}

