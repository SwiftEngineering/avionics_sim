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
#include <errno.h>

TEST_P(Atan2Parameterized, Atan2_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	double x;
	double y;

	Atan2Params param=GetParam();
	x=param.x;
	y=param.y;

	double arctan=atan2(y,x);

	if (math_errhandling & MATH_ERRNO) 
	{
		if (errno==EDOM)
		{
			std::cerr<<"Exception successfully caught for atan2."<<std::endl;

			ASSERT_EQ(1,1);
		}
		
	}

	ASSERT_FLOAT_EQ(arctan, arctan);
}
