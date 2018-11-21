/**
 * @brief       Implementation for PropWashModel class functions
 * @file        Prop_wash_model.cpp
 * @author      Nuno Marques <nuno.marques@dronesolutions.io>
 * @copyright   Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 */

#include "swift_common/Prop_wash_model.hpp"
#include <cmath>

double PropWashModel::hover_exit_velocity_from_motor_thrust(const double &thrust) {
	return 0.0736560590372073 * std::pow(thrust, 3) - 0.702633077657167 * std::pow(thrust, 2) + 2.7030763802904 * thrust;
}

double PropWashModel::cruise_exit_velocity_from_motor_thrust(const double &thrust) {
	return 5.49293488577616 * std::pow(thrust, 3) - 12.8282803262074 * std::pow(thrust, 2) + 11.972544278189 * thrust;
}

double PropWashModel::motor_thrust_from_hover_exit_velocity(const double &velocity) {
	return -1.33511353301094E-17 * std::pow(velocity, 3) + 0.2098022317 * std::pow(velocity, 2);
}

double PropWashModel::motor_thrust_from_cruise_exit_velocity(const double &velocity) {
	return 3.14157801432187E-17 * std::pow(velocity, 3) + 0.0446915997 * std::pow(velocity, 2) + 3.83435246246275E-16 * velocity;
}
