/**
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include <gtest/gtest.h>

#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "Math_util.hpp"

const unsigned int EQUALS = 1;
const unsigned int LESS_THAN = 2;
const unsigned int GREATER_THAN = 3;
const unsigned int LESS_THAN_EQ = 4;
const unsigned int GREATER_THAN_EQ = 5;

class MathUtilTest : public ::testing::Test {
  protected:
    avionics_sim::Math_util mathUtil;
    double tolerance = 0.00001;
};

TEST_F(MathUtilTest, TestCalculateMean) {
    // Given: Range of Values
    std::vector<double> samples = {2.0, -10, 99, -100000};
    double expectedMean = -24977.25;

    // When: Dynamic Pressure is Calculated
    double resultantMean = mathUtil.calculate_mean(samples.begin(), samples.end(),
                           samples.size());

    // Then: Dynamic Pressure should match expected
    ASSERT_NEAR(resultantMean, expectedMean, tolerance);
}

TEST_F(MathUtilTest, TestCalculateRMS) {
    // Given: Range of Values
    std::vector<double> samples = {2.0, -10, 99, -100000};
    double expectedRMS = 50000.02476;

    // When: Dynamic Pressure is Calculated
    double resultantRMS = mathUtil.calculate_rms(samples.begin(), samples.end(),
                          samples.size());

    // Then: Dynamic Pressure should match expected
    ASSERT_NEAR(resultantRMS, expectedRMS, tolerance);
}

// TEST_F(MathUtilTest, TestCalculateFPComparison) {
//   // Given: Planar Velocity and Air Density
//   double lhs;
//   double rhs;
//   unsigned int operation;
//   bool testValue;

//   MathUtilFPComparisonParams param = GetParam();
//   lhs = param.lhs;
//   rhs = param.rhs;
//   operation = param.operation;
//   testValue = param.testValue;

//   avionics_sim::Math_util mu;
//   double result;

//   switch (operation) {
//   case EQUALS:
//     result = mu.rough_eq(lhs, rhs);
//     break;

//   case LESS_THAN:
//     result = mu.rough_lt(lhs, rhs);
//     break;

//   case LESS_THAN_EQ:
//     result = mu.rough_lte(lhs, rhs);
//     break;

//   case GREATER_THAN:
//     result = mu.rough_gt(lhs, rhs);
//     break;

//   case GREATER_THAN_EQ:
//     result = mu.rough_gte(lhs, rhs);
//     break;
//   }

//   ASSERT_EQ(result, testValue);
// }

TEST_F(MathUtilTest, TestLinearMap) {
    // Given:
    double x = 4.651;
    double in_min = 4.0;
    double in_max = 5.0;
    double out_min = 0.44;
    double out_max = 0.55;
    double expectedValue = 0.511614;


    // When:
    double result = mathUtil.linear_map(x, in_min, in_max, out_min, out_max);

    // Then:
    ASSERT_NEAR(result, expectedValue, tolerance);
}

// TODO(Mike Lyons): Figure out why this has a hold up in the test
// TEST_F(MathUtilTest, TestPowInteger) {
//   // Given:
//   double base = 5;
//   size_t power = -2;
//   double expectedValue = 0.04;

//   // When:
//   double result = mathUtil.pow_integer(base, power);

//   // Then
//   ASSERT_NEAR(result, expectedValue, tolerance);
// }
