/**
 * @brief       Math_util::calculate_mean Test Class
 * @file        Test_Math_Util_Calculate_RMS.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "MathUtilFPComparisonParameterized.h"
#include "avionics_sim/Math_util.hpp"

TEST_P(MathUtilFPComparisonParameterized, MathUtilFPComparison_UnitTest) {
	double lhs;
	double rhs;
	unsigned int operation;
	bool testValue;

	MathUtilFPComparisonParams param=GetParam();
	lhs=param.lhs;
	rhs=param.rhs;
	operation=param.operation;
	testValue=param.testValue;
	try
	{
		avionics_sim::Math_util mu;
		double result;
		switch (operation)
		{
			case EQUALS:
			result=mu.rough_eq(lhs, rhs);
			break;

			case LESS_THAN:
			result=mu.rough_lt(lhs, rhs);
			break;

			case LESS_THAN_EQ:
			result=mu.rough_lte(lhs, rhs);
			break;

			case GREATER_THAN:
			result=mu.rough_gt(lhs, rhs);
			break;

			case GREATER_THAN_EQ:
			result=mu.rough_gte(lhs, rhs);
			break;
		}
		ASSERT_EQ(result,testValue);
	}
	catch(const MathUtilException& e)
	{
		std::cerr<<"Exception successfully caught for calculate_mean. Error: "<<std::endl;
		std::cerr << e.what() <<std::endl;
	}

}