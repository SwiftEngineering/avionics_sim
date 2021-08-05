/**
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "TestUtils.hpp"

#include "GaussianWindModel.hpp"

namespace avionics_sim {

/*
  With A seed of 1, the order of outputs for default random engine and consecuatively from normal distribution
  random_engine | normal distribution
  16807         | -0.121966
  282475249     | -1.08682
  1622650073    | 0.68429
  984943658     | -1.07519
  1144108930    | 0.0332695
  470211272     | 0.744836
  101027544     | 0.0336061
  1457850878    | -0.526637
 */


TEST(DiscreteGaussianWindModelTest, Test_Blank_Initialization) {
    std::default_random_engine str_generator(1);
    std::default_random_engine dir_generator(1);

    GaussianWindModel wind_model(str_generator, dir_generator);

    WindRate wind_rate = wind_model.get_rates();

    ASSERT_TRUE(&wind_model != nullptr);
}

TEST(DiscreteGaussianWindModelTest, Test_Getting_WindRate) {
    std::default_random_engine str_generator(1);
    std::default_random_engine dir_generator(1);

    GaussianWindModel wind_model(str_generator, dir_generator);

    WindRate wind_rate = wind_model.get_rates();

    EXPECT_NEAR(wind_rate.linear_rate.Length(), 0.1219657839, epsilon);
}

TEST(DiscreteGaussianWindModelTest, Test_Setting_Direction) {
    std::default_random_engine str_generator(1);
    std::default_random_engine dir_generator(1);

    GaussianWindModel wind_model(str_generator, dir_generator);

    wind_model.set_direction({0, 0, 20}, 1E-10);

    WindRate wind_rate = wind_model.get_rates();

    EXPECT_NEAR(wind_rate.linear_rate.Length(), 0.1219657839, epsilon);
    EXPECT_V3_NEAR(wind_rate.linear_rate.Normalize(), v3(0, 0, -1), epsilon);
}

TEST(DiscreteGaussianWindModelTest, Test_Initializing_With_Strength_and_Direction) {
    std::default_random_engine str_generator(1);
    std::default_random_engine dir_generator(1);
    double strength_m_per_s = 8;
    double strength_variance = 1E-10;

    v3 direction(0, -12, 0);
    v3 dir_variance(1E-10, 1E-10, 1E-10);

    GaussianWindModel wind_model(
        str_generator, dir_generator,
        strength_m_per_s, strength_variance,
        direction, dir_variance);

    WindRate wind_rate = wind_model.get_rates();

    EXPECT_NEAR(wind_rate.linear_rate.Length(), 8, epsilon);
    EXPECT_V3_NEAR(wind_rate.linear_rate.Normalize(), v3(0, -1, 0), epsilon);
}

}  // namespace avionics_sim
