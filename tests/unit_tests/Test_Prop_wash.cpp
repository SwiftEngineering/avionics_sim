/**
 * @brief       Implementation for testing the prop wash polynomials
 * @file        Test_Prop_wash.cpp
 * @author      Nuno Marques <nuno.marques@dronesolutions.io>
 * @copyright   Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <avionics_sim/NPolynomial_Eval.hpp>

/// \brief Default prop wash polynomial coefficients for hover motors
static const auto kDefaultPropWashHoverPoly =  boost::array<double,4> {
	{ 0.0000000000000000,	// 0
	  2.7030763802904000,	// 1
	  -0.702633077657167,	// 2
	  0.0736560590372073 }	// 3
};

/// \brief Default prop wash polynomial coefficients for cruise motors
static const auto kDefaultPropWashCruisePoly =  boost::array<double,4> {
	{ 0.000000000000000,	// 0
	  11.97254427818900,	// 1
	  -12.8282803262074,	// 2
	  5.492934885776160 }	// 3
};

/// \brief Default prop wash inverse polynomial coefficients for hover motors
static const auto kDefaultPropWashHoverInvPoly =  boost::array<double,4> {
	{ 0.0000000000000000000, // 0
	  0.0000000000000000000, // 1
	  0.2098022317000000000, // 2
	  -1.33511353301094E-17 }// 3
};

/// \brief Default prop wash inverse polynomial coefficients for cruise motors
static const auto kDefaultPropWashCruiseInvPoly =  boost::array<double,4> {
	{ 0.000000000000000000,	// 0
	  3.83435246246275E-16,	// 1
	  0.044691599700000000,	// 2
	  3.14157801432187E-17 }// 3
};

// Test variables
static const auto hover_thrust_newtons = boost::array<double, 5> {
	{ 4.496178473106780,
	  3.372133854830080,
	  2.248089236553390,
	  1.124044618276700,
	  0.000000000000000 }
};
static const auto hover_exit_velocity = boost::array<double, 5> {
	{ 4.629314850723420,
	  4.009104262843050,
	  3.273419923194120,
	  2.314657425361710,
	  0.000000000000000 }
};
static const auto cruise_thrust_newtons = boost::array<double, 6> {
	{ 1.124044618276700,
	  0.899235694621356,
	  0.674426770966017,
	  0.449617847310678,
	  0.224808923655339,
	  0.000000000000000 }
};
static const auto cruise_exit_velocity = boost::array<double, 6> {
	{ 5.015091088283710,
	  4.485633834702310,
	  3.884672852927200,
	  3.171822102437820,
	  2.242816917351150,
	  0.000000000000000 }
};

/// \brief Intantiate a NPolynomialEval for a 3rd degree polynomial of type double
NPolynomialEval<double, 4> pf;

/// \brief Test getting hover exit velocity from motor thurst input
TEST(Prop_wash_UnitTest, Test_hover_exit_velocity_from_motor_thrust) {
	pf.load_poly_fit_curve(kDefaultPropWashHoverPoly);
	for (size_t i = 0; i < hover_thrust_newtons.size(); i++)
		ASSERT_NEAR(pf.get_output_from_poly(0, hover_thrust_newtons[i]), hover_exit_velocity[i], 2E-1);
}

/// \brief Test getting cruise exit velocity from motor thurst input
TEST(Prop_wash_UnitTest, Test_cruise_exit_velocity_from_motor_thrust) {
	pf.load_poly_fit_curve(kDefaultPropWashCruisePoly);
	for (size_t i = 0; i < cruise_thrust_newtons.size(); i++)
		ASSERT_NEAR(pf.get_output_from_poly(1, cruise_thrust_newtons[i]), cruise_exit_velocity[i], 2E-1);
}

/// \brief Test getting motor thrust from hover exit velocity output
TEST(Prop_wash_UnitTest, Test_motor_thrust_from_hover_exit_velocity) {
	pf.load_poly_fit_curve(kDefaultPropWashHoverInvPoly);
	for (size_t i = 0; i < hover_exit_velocity.size(); i++)
		ASSERT_NEAR(pf.get_output_from_poly(2, hover_exit_velocity[i]), hover_thrust_newtons[i], 2E-1);
}

/// \brief Test getting motor thrust from cruise exit velocity output
TEST(Prop_wash_UnitTest, Test_motor_thrust_from_cruise_exit_velocity) {
	pf.load_poly_fit_curve(kDefaultPropWashCruiseInvPoly);
	for (size_t i = 0; i < cruise_exit_velocity.size(); i++)
		ASSERT_NEAR(pf.get_output_from_poly(3, cruise_exit_velocity[i]), cruise_thrust_newtons[i], 2E-1);
}

/// \brief Test getting hover exit velocity from motor thurst input and inputting the poly as well
TEST(Prop_wash_UnitTest, Test_hover_exit_velocity_from_motor_thrust_with_poly_input) {
	for (size_t i = 0; i < hover_thrust_newtons.size(); i++)
		ASSERT_NEAR(pf.get_output_from_poly(kDefaultPropWashHoverPoly, hover_thrust_newtons[i]), hover_exit_velocity[i], 2E-1);
}

/// \brief Test getting hover exit velocity from motor thurst input and inputting the poly as well
TEST(Prop_wash_UnitTest, Test_cruise_exit_velocity_from_motor_thrust_with_poly_input) {
	for (size_t i = 0; i < cruise_thrust_newtons.size(); i++)
		ASSERT_NEAR(pf.get_output_from_poly(kDefaultPropWashCruisePoly, cruise_thrust_newtons[i]), cruise_exit_velocity[i], 2E-1);
}
