#include <gtest/gtest.h>
// #include <gmock/gmock.h>

#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include "Coordinate_Utils.hpp"

// #define ASSERT_PERCENT_NEAR(value, tolerance)

struct CoordUtilsParams {
    std::string case_name;
    ignition::math::Pose3d poseInWorld;
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

TEST_P(CoordUtilsParamTest, CalcTransform) {
  CoordUtilsParams params = GetParam();

  // ignition::math::Quaterniond rot = params.poseInWorld.Rot();
  // std::cerr << "Rotation: <" << rot.W() << ", " << rot.X() << ", "<< rot.Y() << ", "<< rot.Z() << "> " ;

  // ignition::math::Quaterniond rotInv = params.poseInWorld.Rot().Inverse();
  // std::cerr << "Rotation Inverse: <" << rotInv.W() << ", " << rotInv.X() << ", "<< rotInv.Y() << ", "<< rotInv.Z() << "> " ;


  EXPECT_EQ(2,2);
}

const std::vector<CoordUtilsParams> params{
    {
      "Test 1",
      ignition::math::Pose3d(0, 0, 0, -0.09, 1.48, 0.1)
    }
};

INSTANTIATE_TEST_CASE_P(CoordUtilsTest,
                         CoordUtilsParamTest,
                         ::testing::ValuesIn( params ));
