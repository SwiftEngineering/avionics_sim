/**
 * @brief       PropWashModel class for testing
 * @file        Prop_wash_model.cpp
 * @author      Nuno Marques <nuno.marques@dronesolutions.io>
 * @copyright   Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 */

#ifndef PROP_WASH_MODEL_HPP
#define PROP_WASH_MODEL_HPP

#include <array>

/// \brief Default prop wash polynomial indeterminates for hover motors
static const auto kDefaultPropWashHoverPoly =  std::array<double, 3> {
	{ 0.0736560590372073,
	  -0.702633077657167,
	  2.7030763802904000 }
};

/// \brief Default prop wash polynomial indeterminates for cruise motors
static const auto kDefaultPropWashCruisePoly =  std::array<double, 3> {
	{ 5.492934885776160,
	  -12.8282803262074,
	  11.97254427818900 }
};

/// \brief Default prop wash inverse polynomial indeterminates for hover motors
static const auto kDefaultPropWashHoverInvPoly =  std::array<double, 3> {
	{ -1.33511353301094E-17,
	  0.2098022317000000000,
	  0.0000000000000000000 }
};

/// \brief Default prop wash inverse polynomial indeterminates for cruise motors
static const auto kDefaultPropWashCruiseInvPoly =  std::array<double, 3> {
	{ 3.14157801432187E-17,
	  0.044691599700000000,
	  3.83435246246275E-16 }
};

/// \brief PropWashModel class that implements the prop wash effect polinomial fit curves
/// for the relation between the motor exit velocity and the motor thrust
class PropWashModel {
public:
	PropWashModel() {};
	virtual ~PropWashModel() = default;

	/// \brief Gets the exit velocity from the motor thrust
	double exit_velocity_from_motor_thrust(const std::array<double, 3> &poly, const double &thrust);

	/// \brief Gets the motor thrust from the exit velocity
	double motor_thrust_from_exit_velocity(const std::array<double, 3> &poly, const double &velocity);
};	// class PropWashModel

#endif	// PROP_WASH_MODEL_HPP
