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
#include <ignition/math/Quaternion.hh>

#include "Coordinate_Utils.hpp"

// #define ASSERT_PERCENT_NEAR(value, tolerance)

TEST(CoordUtilsTest, RotationFromBasis) {
    // Given
    ignition::math::Vector3d forward(0, 0, 1);
    ignition::math::Vector3d upward(-1, 0, 0);

    ignition::math::Vector3d aVec(0.81165, -0.90368, 9.9767);

    ignition::math::Quaterniond rotAtoB = avionics_sim::Coordinate_Utils::QuatFromBasis(forward, upward);

    ignition::math::Vector3d bVec = rotAtoB.RotateVector(aVec);

    ignition::math::Vector3d expectedBVec(-0.81165, 0.90368, 9.9767);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(bVec[i], expectedBVec[i], 0.01);
    }
}

struct CoordUtilsParams {
    std::string case_name;
    ignition::math::Pose3d poseInWorld;
    ignition::math::Vector3d vecToRotate;
    ignition::math::Vector3d vecExpected;
};

class CoordUtilsParamTest : public ::testing::TestWithParam<CoordUtilsParams> {
  protected:
    // Some expensive resource shared by all tests.
    // static T* shared_resource_;

    /**
     * Sets up the test suite, called before all test cases are run
     */
    static void SetUpTestSuite() {
    }

    /**
     * Tears down the test suite, called after all test cases are run
     */
    static void TearDownTestSuite() {
    }

    // You can define per-test set-up logic as usual.
    virtual void SetUp() {
    }

    // You can define per-test tear-down logic as usual.
    virtual void TearDown() {
    }
};

TEST_P(CoordUtilsParamTest, ProjectVectorGlobalTest) {
    CoordUtilsParams params = GetParam();

    ignition::math::Vector3d vecResult;
    avionics_sim::Coordinate_Utils::project_vector_global(params.poseInWorld, params.vecToRotate, &vecResult);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(vecResult[i], params.vecExpected[i], 0.01);
    }
}

const std::vector<CoordUtilsParams> params{
    {
        "Test 1",
        ignition::math::Pose3d(0, 0, 0, -0.09, 1.48, 0.1),
        ignition::math::Vector3d(10, 1, 0.1),
        ignition::math::Vector3d(0.81165, -0.90368, 9.97673)
    }
};

INSTANTIATE_TEST_CASE_P(CoordUtilsTest,
                        CoordUtilsParamTest,
                        ::testing::ValuesIn(params));
