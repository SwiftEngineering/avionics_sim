/**
 * @brief       Atan2 Parameterized Unit Test Class
 * @file        Test_Atan2.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 * @detail     This test class was written to ensure that atan2 domain error is catchable on platform.
 * @   Copyright (c) 2019, Swift Engineering Inc.
 */
#include "Atan2Parameterized.h"
#include "avionics_sim/Lift_drag_model.hpp"
#include "avionics_sim/Lift_drag_model_exception.hpp"

TEST_P(Atan2Parameterized, Atan2_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double x;
	double y;

	Atan2Params param=GetParam();
	x=param.x;
	y=param.y;

	try
	{
		double arctan=atan2(y,x);
		ASSERT_FLOAT_EQ(arctan, arctan);
	}
	catch(const std::exception& e)
	{
		std::cerr<<"Exception successfully caught for atan2."<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}
