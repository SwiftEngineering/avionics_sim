/**
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include <gtest/gtest.h>

#include "TestUtils.hpp"
#include "FourBarMechanism.h"


namespace avionics_sim {

TEST(FourBarMechanismTest, Test_transmit_for_positive_link) {
    FourBarMechanism linkage(0.034738, 0.71124);

    float input_angle_deg_percent = 0.385;

    double output_angle_rad = linkage.transmit(input_angle_deg_percent);

    ASSERT_NEAR(RAD2DEG(output_angle_rad), 24.98, epsilon);
}

TEST(FourBarMechanismTest, Test_transmit_for_negative_link) {
    FourBarMechanism linkage(-0.0815, 0.53191);

    float input_angle_deg_percent = 0.715;

    double output_angle_rad = linkage.transmit(input_angle_deg_percent);

    ASSERT_NEAR(RAD2DEG(output_angle_rad), 24.98, epsilon);
}


TEST(FourBarMechanismTest, Test_Limiting_Behavior) {
    FourBarMechanism linkage(-0.0815, 0.53191);

    float input_angle_deg_percent = -1.8;

    double output_angle_rad = linkage.transmit(input_angle_deg_percent);

    ASSERT_NEAR(RAD2DEG(output_angle_rad), -24.98, epsilon);
}

}  // namespace avionics_sim
