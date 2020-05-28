/**
 * @brief       Implementation for testing calculation of dynamic pressure.
 * @file        Test_Lift_Drag_Calculate_Dynamic_Pressure.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright(C) 2019, Swift Engineering Inc. All rights reserved.
 */

#include <gtest/gtest.h>
#include "avionics_sim/Lift_drag_model.hpp"
#include "LiftDragCalculateDynamicPressureParameterized.h"

//Accurate within a thousandth. Decreasing this value increases rates of test failure.
float acceptable_error=10E-3;
const float rho=1.225;

//TODO: Choose random acceptable values for rho, speed. Parameterize. Rename this file to be Test_Lift_Drag_Calculate_Dynamic_Pressure.
/// \brief Test setting and getting alpha aerodynamic force.
TEST_P(LiftDragCalculateDynamicPressureParameterized, LiftDragUnitTestCalculateDynamicPressure_UnitTest) {
	avionics_sim::Lift_drag_model ldm;
	
	//Disable test implementation until US1273 and US1275 are completed and signed off.
	/*LiftDragCalculateDynamicPressureParams param=GetParam();
	double testSpeed=param.vInf;
    double testRho=param.rho;
	double valueToTestAgainst=0.5*testRho*testSpeed*testSpeed;
	ldm.setPlanarVelocity(testSpeed);
	ldm.setAirDensity(testRho);
	ldm.calculateDynamicPressure();
	ASSERT_FLOAT_EQ(ldm.getDynamicPressure(), valueToTestAgainst);*/
	ASSERT_EQ(1,1);
}