/**
 * @brief       Math_util::calculate_rms Test Class
 * @file        Test_Math_Util_Calculate_RMS.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "MathUtilCalculateRMSParameterized.h"
#include "avionics_sim/Math_util.hpp"

TEST_P(MathUtilCalculateRMSParameterized, MathUtilCalculateRMS_UnitTest) {
	std::vector<double> samples;
	double count;
	double testValue;

	MathUtilCalculateRMSParams param=GetParam();
	samples=param.samples;
	testValue=param.testValue;
	count=param.count;
	try
	{
		avionics_sim::Math_util mu;
		double result;
		result=mu.calculate_rms(samples.begin(), samples.end(), count);
		ASSERT_EQ(result,testValue);
	}
	catch(const MathUtilException& e)
	{
		std::cerr<<"Exception successfully caught for pow_integer. Error: "<<std::endl;
		std::cerr << e.what() <<std::endl;
	}

}