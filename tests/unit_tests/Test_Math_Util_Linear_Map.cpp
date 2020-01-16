/**
 * @brief       Math_util::linear_map Test Class
 * @file        Test_Math_Util_Linear_Map.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "MathUtilLinearMapParameterized.h"
#include "avionics_sim/Math_util.hpp"

TEST_P(MathUtilLinearMapParameterized, MathUtilLinearMap_UnitTest) {
	double x;
	double in_min;
	double in_max;
	double out_min;
	double out_max;
	double testValue;

	MathUtilParams param=GetParam();
	x=param.x;
	in_min=param.in_min;
	in_max=param.in_max;
	out_min=param.out_min;
	out_max=param.out_max;
	testValue=param.testValue;

	try
	{
		avionics_sim::Math_util mu;
		double result=mu.linear_map(x, in_min, in_max, out_min, out_max);
		ASSERT_EQ(result,testValue);
	}
	catch(const MathUtilException& e)
	{
		std::cerr<<"Exception successfully caught for linearMap. Error: "<<std::endl;
		std::cerr << e.what() <<std::endl;
	}
}