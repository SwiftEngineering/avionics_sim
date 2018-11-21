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

	const std::array<double, 5> hover_thrust_newtons =  {{ 4.496178473106780,
							       3.372133854830080,
							       2.248089236553390,
							       1.124044618276700,
							       0.000000000000000 }};
	const std::array<double, 5> hover_exit_velocity =   {{ 4.629314850723420,
							       4.009104262843050,
							       3.273419923194120,
							       2.314657425361710,
							       0.000000000000000 }};
	const std::array<double, 6> cruise_thrust_newtons = {{ 1.124044618276700,
							       0.899235694621356,
							       0.674426770966017,
							       0.449617847310678,
							       0.224808923655339,
							       0.000000000000000 }};
	const std::array<double, 6> cruise_exit_velocity =  {{ 5.015091088283710,
							       4.485633834702310,
							       3.884672852927200,
							       3.171822102437820,
							       2.242816917351150,
							       0.000000000000000 }};
};
