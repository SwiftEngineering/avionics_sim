/**
 * @brief       Test for Aggregate Wind Model
 * @file        AggregateWindModelTest.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "TestUtils.hpp"

#include "SustainedWindModel.hpp"
#include "AggregateWindModel.hpp"

namespace avionics_sim {

TEST(AggregateWindModelTest, Test_Blank_Initialization) {
    AggregateWindModel wind_model;

    WindRate wind_rate = wind_model.get_rates();

    EXPECT_NEAR(wind_rate.linear_rate.Length(), 0, epsilon);
}

TEST(AggregateWindModelTest, Test_With_Two_Models_Same_Dir) {
    SustainedWindModel wind_model_1(7, v3(0, 1, 0));
    SustainedWindModel wind_model_2(3, v3(0, 1, 0));

    AggregateWindModel wind_model;
    wind_model.add_model(wind_model_1);
    wind_model.add_model(wind_model_2);

    WindRate wind_rate = wind_model.get_rates();

    EXPECT_NEAR(wind_rate.linear_rate.Length(), 10, epsilon);
    EXPECT_V3_NEAR(wind_rate.linear_rate.Normalize(), v3(0, 1, 0), epsilon);
}

TEST(AggregateWindModelTest, Test_With_Two_Models_Diff_Dir) {
    SustainedWindModel wind_model_1(3, v3(0, 1, 0));
    SustainedWindModel wind_model_2(3, v3(1, 0, 0));

    AggregateWindModel wind_model;
    wind_model.add_model(wind_model_1);
    wind_model.add_model(wind_model_2);

    WindRate wind_rate = wind_model.get_rates();

    EXPECT_NEAR(wind_rate.linear_rate.Length(), 4.242640687, epsilon);
    EXPECT_V3_NEAR(wind_rate.linear_rate.Normalize(), v3(0.707107, 0.707107, 0), epsilon);
}

}  // namespace avionics_sim
