/**
 * @brief       Lift_drag_model::setFreestreamVelocity Test Class
 * @file        Test_Lift_Drag_Set_Get_Freestream_Velocity.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragSetVelocityParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragSetVelocityParameterized, LiftDragUnitTestSetGetFreestreamVelocity_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double valueToTestAgainst;
	LiftDragSetVelocityParams param=GetParam();
	valueToTestAgainst=param.value;
	double inputSpeed=param.value;
	try
	{
		ldm.setFreeStreamVelocity(inputSpeed);
		ASSERT_FLOAT_EQ(ldm.getFreeStreamVelocity(), valueToTestAgainst);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for setFreeStreamVelocity."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}

