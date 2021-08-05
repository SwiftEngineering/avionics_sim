/**
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "TestUtils.hpp"

#include <iostream>

#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "IWindModel.hpp"
#include "GaussianWindModel.hpp"

#define PERCENT_DIFF_100(a, b) ((a == 0) && (b == 0)) ? 0 : (2.0 * abs(a - b) / (a + b) * 100.0)
#define EXPECT_PERCENT_DIFF_LT(a, b, tolerance_100) EXPECT_LT(PERCENT_DIFF_100(a, b), tolerance_100)


namespace avionics_sim {

struct WorldFrameCaseParams {
    std::string case_name;
    ignition::math::Pose3d pose_world_m;
    ignition::math::Vector3d velocity_world_m_per_s;
    ignition::math::Vector3d expectedRates_body_m_per_s;
};

class IWindModelTest : public ::testing::TestWithParam<WorldFrameCaseParams> {
  protected:
    avionics_sim::IWindModel *wind_model_;
    double tolerance_100 = 0.5; /**< Allowable percent tolerance, note this is 0.5%, not 50% */

    static void SetUpTestSuite() {
    }

    static void TearDownTestSuite() {
        // delete shared_resource_;
    }

    virtual void SetUp() {
        // Its not optimal to use a non-mocked RNG, but with a defined seed, there is a defined output
        std::default_random_engine str_generator(1);
        std::default_random_engine dir_generator(2);

        wind_model_ = new GaussianWindModel(str_generator, dir_generator);
    }

    virtual void TearDown() {
    }
};

TEST_P(IWindModelTest, AerodynamicForcesFromVelocityAndOrientation) {
    WorldFrameCaseParams params = GetParam();

    // Given:
    // Initial Position, Rotation and Velocity from params
    double altitude = 0;
    ignition::math::Vector3d velocity = params.velocity_world_m_per_s;
    ignition::math::Pose3d orientation = params.pose_world_m;

    // When:
    WindRate wind_rates = wind_model_->get_rates();

    // Then:
    // The Resultant Forces should equal the expected forces
    v3 expectedLinearRates_m_per_s = params.expectedRates_body_m_per_s;
    std::cout << "Rates: " << wind_rates.linear_rate << "\n";


    EXPECT_NEAR(wind_rates.linear_rate.X(), expectedLinearRates_m_per_s.X(), tolerance_100);
    EXPECT_NEAR(wind_rates.linear_rate.Y(), expectedLinearRates_m_per_s.Y(), tolerance_100);
    EXPECT_NEAR(wind_rates.linear_rate.Z(), expectedLinearRates_m_per_s.Z(), tolerance_100);
}

const std::vector<WorldFrameCaseParams> params{
    {
        "Home Work Problem alpha = 4 deg, beta = -5.1587 deg",
        ignition::math::Pose3d(
            ignition::math::Vector3d(-0.09, 1.48, 0.1),
            ignition::math::Quaterniond(1, 0, 0, 0)),
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Vector3d(0, 0, 0)
    }
};

INSTANTIATE_TEST_CASE_P(WindModelTests,
                        IWindModelTest,
                        ::testing::ValuesIn(params));

}  // namespace avionics_sim
