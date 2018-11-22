/**
 * @brief       Implementation for PropWashModel class functions
 * @file        Prop_wash_model.cpp
 * @author      Nuno Marques <nuno.marques@dronesolutions.io>
 * @copyright   Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 */

#include <cmath>
#include "swift_common/Prop_wash_model.hpp"

double PropWashModel::exit_velocity_from_motor_thrust(const std::array<double, 3> &poly, const double &thrust) {
	return poly[0] * std::pow(thrust, 3) + poly[1] * std::pow(thrust, 2) + poly[2] * thrust;
}

double PropWashModel::motor_thrust_from_exit_velocity(const std::array<double, 3> &poly, const double &velocity) {
	return poly[0] * std::pow(velocity, 3) + poly[1] * std::pow(velocity, 2) + poly[2] * velocity;
}
