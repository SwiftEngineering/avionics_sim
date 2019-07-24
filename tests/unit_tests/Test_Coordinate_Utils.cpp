/**
 * @brief       Parameterized implementation for testing Coordinate Utils class. 
 * Canonical values obtained from Conrad McGreal's MATLAB hand calcs.
 * @file        Test_Coordinate_Utils.cpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright(C) 2019, Swift Engineering Inc. All rights reserved.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <ignition/math.hh>
#include <stdio.h>
#include "CoordUtilsTest.h"

TEST_P(CoordUtilsTest, CoordUtilsTest_Project_Vector_Global_UnitTest) {
	CoordUtilsParams param=GetParam();
    double wingPoseX=std::stof(param.worldOrientations.at(0));
    double wingPoseY=std::stof(param.worldOrientations.at(1));
    double wingPoseZ=std::stof(param.worldOrientations.at(2));
    double worldVelocityX=std::stof(param.worldLinearVelocities.at(0));
    double worldVelocityY=std::stof(param.worldLinearVelocities.at(1));
    double worldVelocityZ=std::stof(param.worldLinearVelocities.at(2));
    double canonicalX=std::stof(param.globalVelocities.at(0));
    double canonicalY=std::stof(param.globalVelocities.at(1));
    double canonicalZ=std::stof(param.globalVelocities.at(2));
    ignition::math::Vector3d canonicalResult =ignition::math::Vector3d(worldVelocityX, worldVelocityY, worldVelocityZ); 
    ignition::math::Pose3d wingPose =ignition::math::Pose3d(0.0,0.0,0.0,wingPoseX, wingPoseY, wingPoseZ); 
    ignition::math::Vector3d worldVelocity =ignition::math::Vector3d(worldVelocityX, worldVelocityY, worldVelocityZ);
    ignition::math::Vector3d destVector;
    ignition::math::Vector3d vBody; 
    ignition::math::Vector3d wingFrameVelocity;
    ignition::math::Vector3d vInLDPlane_v; 
    vBody = ignition::math::Vector3d(worldVelocity.X(), worldVelocity.Y(), worldVelocity.Z());
    avionics_sim::Coordinate_Utils::project_vector_global(wingPose, vBody, &wingFrameVelocity);
    ASSERT_NEAR(wingFrameVelocity.X(), canonicalX, 1e-03);
    ASSERT_NEAR(wingFrameVelocity.Y(), canonicalY, 1e-03);
    ASSERT_NEAR(wingFrameVelocity.Z(), canonicalZ, 1e-03);
}
