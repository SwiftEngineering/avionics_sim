/**
 * @brief       PropWashModel class for testing
 * @file        Prop_wash_model.cpp
 * @author      Nuno Marques <nuno.marques@dronesolutions.io>
 * @copyright   Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 */

#pragma once

#include <array>

class PropWashModel {
public:
	PropWashModel() {};
	virtual ~PropWashModel() = default;

	double hover_exit_velocity_from_motor_thrust(const double &thrust);
	double cruise_exit_velocity_from_motor_thrust(const double &thrust);

	double motor_thrust_from_hover_exit_velocity(const double &velocity);
	double motor_thrust_from_cruise_exit_velocity(const double &velocity);
};
