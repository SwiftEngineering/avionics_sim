/**
 * @brief       LiftDrag::CalculateForcesParams Parameterized Unit Test Class
 * @file        LiftDragCalculateForcesParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#ifndef _LIFT_DRAG_CALCULATE_FORCES_PARAMS_H
#define _LIFT_DRAG_CALCULATE_FORCES_PARAMS_H
#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// LiftDragCalculateForcesParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts force calculation related parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragCalculateForcesParams {
	double rho;
	ignition::math::Vector3d inputWorldVel;
	ignition::math::Pose3d inputPose;
	ignition::math::Vector3d vFwd;
	ignition::math::Vector3d vUpwd;
    double area, motorExitVelocity, controlAlpha;
	std::vector<double> LUT_alpha;
	std::vector<double> LUT_CL;
	std::vector<double> LUT_CD;
    ignition::math::Vector3d trueLift;
	ignition::math::Vector3d trueDrag;
	ignition::math::Vector3d trueLateralForce;
    bool hasMotorExitVelocity;
    bool isControlSurface;
    bool isNonSpecific; //Added this flag for generic, non 021 specific test case.
	std::string linkName; //Added for debug purposes
	double vInf;
}LiftDragCalculateForcesParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragCalculateForcesParams> LiftDragCalculateForcesParameterCollections;

typedef struct LiftDragCalculateMultiElementForcesParams {
	LiftDragCalculateForcesParameterCollections elementData;
	ignition::math::Vector3d summedTrueForce;
}LiftDragCalculateMultiElementForcesParams;

//Forced to do this because parameter test cannot take struct directly (?). Use only one of these-pack the element data.
/*
TODO: 
1) Test that this works (see cpp)
2) construct multi-element test. When this works, fork into try-catch and documentation
branches while awaiting last test.
*/
typedef std::vector<LiftDragCalculateMultiElementForcesParams> LiftDragCalculateMultiElementForcesParameterCollections;

#endif