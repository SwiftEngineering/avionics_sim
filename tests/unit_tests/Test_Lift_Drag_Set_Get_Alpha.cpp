/**
 * @brief       Lift_drag_model::setAlpha Test Class
 * @file        Test_Lift_Drag_Set_Get_Alpha.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "LiftDragSetAlphaParameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(LiftDragSetAlphaParameterized, LiftDragUnitTestSetGetAlpha_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double valueToTestAgainst;
	LiftDragSetAlphaParams param=GetParam();
	double inputAngle=param.value;

	if (param.isRadians)
	{
		valueToTestAgainst=inputAngle*57.2958;
	}
	else
	{
		valueToTestAgainst=inputAngle;
	}
	try
	{
		ldm.setAlpha(inputAngle,param.isRadians);
		ASSERT_FLOAT_EQ(ldm.getAlpha(), valueToTestAgainst);
	}
	catch(const Lift_drag_model_exception& e)
	{
		std::cerr<<"Exception successfully caught for setAlpha."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}