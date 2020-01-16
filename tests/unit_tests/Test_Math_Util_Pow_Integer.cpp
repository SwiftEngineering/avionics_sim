/**
 * @brief       Math_util::pow_integer Test Class
 * @file        Test_Math_Util_Pow_Integer.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "MathUtilPowIntegerParameterized.h"
#include "avionics_sim/Math_util.hpp"

TEST_P(MathUtilPowIntegerParameterized, MathUtilPowInteger_UnitTest) {
	double base;
	size_t power;
	double testValue;

	MathUtilPowIntegerParams param=GetParam();
	base=param.base;
	power=param.power;
	testValue=param.testValue;
	try
	{
		avionics_sim::Math_util mu;
		double result;
		result=mu.pow_integer(base, power);
		ASSERT_EQ(result,testValue);
	}
	catch(const MathUtilException& e)
	{
		std::cerr<<"Exception successfully caught for pow_integer. Error: "<<std::endl;
		std::cerr << e.what() <<std::endl;
	}

}