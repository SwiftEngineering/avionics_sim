#include <gtest/gtest.h>
// #include <gmock/gmock.h>

#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "AerodynamicModel.hpp"
//#include "Lift_drag_model_exception.hpp"

using namespace avionics_sim;

class LiftDragModelTest : public ::testing::Test {
 protected:
  avionics_sim::AerodynamicModel lift_drag_model_;
  double tolerance = 0.001;

  // You can define per-test set-up logic as usual.
  virtual void SetUp() {
    std::vector<double> LUT_NACA0012_alpha;
    std::vector<double> LUT_NACA0012_CL;
    std::vector<double> LUT_NACA0012_CD;
    std::string alphaLUTString =
      "-180.0000,-175.0000,-170.0000,-165.0000,-160.0000,-155.0000,-150.0000,-145.0000,-140.0000,-135.0000,-130.0000,-125.0000,-120.0000,-115.0000,-110.0000,-105.0000,-100.0000,-95.0000,-90.0000,-85.0000,-80.0000,-75.0000,-70.0000,-65.0000,-60.0000,-55.0000,-50.0000,-45.0000,-40.0000,-35.0000,-30.0000,-27.0000,-26.0000,-25.0000,-24.0000,-23.0000,-22.0000,-21.0000,-20.0000,-19.0000,-18.0000,-17.0000,-16.0000,-15.0000,-14.0000,-13.0000,-12.0000,-11.0000,-10.0000,-9.0000,-8.0000,-7.0000,-6.0000,-5.0000,-4.0000,-3.0000,-2.0000,-1.0000,-0.0000,0.0000,1.0000,2.0000,3.0000,4.0000,5.0000,6.0000,7.0000,8.0000,9.0000,10.0000,11.0000,12.0000,13.0000,14.0000,15.0000,16.0000,17.0000,18.0000,19.0000,20.0000,21.0000,22.0000,23.0000,24.0000,25.0000,26.0000,27.0000,30.0000,35.0000,40.0000,45.0000,50.0000,55.0000,60.0000,65.0000,70.0000,75.0000,80.0000,85.0000,90.0000,95.0000,100.0000,105.0000,110.0000,115.0000,120.0000,125.0000,130.0000,135.0000,140.0000,145.0000,150.0000,155.0000,160.0000,165.0000,170.0000,175.0000,180.0000";
    std::string alphaLUTCL =
      "0.0000,0.6900,0.8500,0.6750,0.6600,0.7400,0.8500,0.9100,0.9450,0.9450,0.9100,0.8400,0.7350,0.6250,0.5100,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5150,-0.6500,-0.7650,-0.8750,-0.9650,-1.0400,-1.0850,-1.0750,-1.0200,-0.9150,-0.9646,-0.9109,-0.8572,-0.8034,-0.7497,-0.6956,-0.6414,-0.5870,-0.5322,-0.4768,-0.4200,-0.3620,-0.3082,-0.2546,-0.2030,-0.1533,-0.1095,-0.1325,-0.8527,-0.8274,-0.7460,-0.6600,-0.5500,-0.4400,-0.3300,-0.2200,-0.1100,-0.0000,0.0000,0.1100,0.2200,0.3300,0.4400,0.5500,0.6600,0.7460,0.8274,0.8527,0.1325,0.1095,0.1533,0.2030,0.2546,0.3082,0.3620,0.4200,0.4768,0.5322,0.5870,0.6414,0.6956,0.7497,0.8034,0.8572,0.9109,0.9646,0.9150,1.0200,1.0750,1.0850,1.0400,0.9650,0.8750,0.7650,0.6500,0.5150,0.3700,0.2200,0.0700,-0.0700,-0.2200,-0.3700,-0.5100,-0.6250,-0.7350,-0.8400,-0.9100,-0.9450,-0.9450,-0.9100,-0.8500,-0.7400,-0.6600,-0.6750,-0.8500,-0.6900,0.0000";
    std::string alphaLUTCD =
      "0.0250,0.0550,0.1400,0.2300,0.3200,0.4200,0.5700,0.7550,0.9250,1.0850,1.2250,1.3500,1.4650,1.5550,1.6350,1.7000,1.7500,1.7800,1.8000,1.8000,1.7800,1.7350,1.6650,1.5750,1.4700,1.3450,1.2150,1.0750,0.9200,0.7450,0.5700,0.4730,0.4460,0.4200,0.3940,0.3690,0.3440,0.3200,0.2970,0.2740,0.2520,0.2310,0.2100,0.1900,0.1710,0.1520,0.1340,0.0760,0.0188,0.0203,0.0185,0.0170,0.0152,0.0140,0.0124,0.0114,0.0108,0.0104,0.0103,0.0103,0.0104,0.0108,0.0114,0.0124,0.0140,0.0152,0.0170,0.0185,0.0203,0.0188,0.0760,0.1340,0.1520,0.1710,0.1900,0.2100,0.2310,0.2520,0.2740,0.2970,0.3200,0.3440,0.3690,0.3940,0.4200,0.4460,0.4730,0.5700,0.7450,0.9200,1.0750,1.2150,1.3450,1.4700,1.5750,1.6650,1.7350,1.7800,1.8000,1.8000,1.7800,1.7500,1.7000,1.6350,1.5550,1.4650,1.3500,1.2250,1.0850,0.9250,0.7550,0.5700,0.4200,0.3200,0.2300,0.1400,0.0550,0.0250";
    avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTString,
        &LUT_NACA0012_alpha);
    avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTCL,
        &LUT_NACA0012_CL);
    avionics_sim::Bilinear_interp::get1DLUTelementsFromString(alphaLUTCD,
        &LUT_NACA0012_CD);

    // Environment Configuration
    PhysicsEnvironment environment = PhysicsEnvironment();

    // Body/Profile Configuration
    double area_m2 = 1.0;
    double area_lateral_m2 = 0.1;
    Airfoil airfoil = Airfoil(
      area_m2, area_lateral_m2,
      LUT_NACA0012_alpha, LUT_NACA0012_CL, LUT_NACA0012_CD);

    // Initialize lift drag model for new test
    lift_drag_model_ = avionics_sim::AerodynamicModel(airfoil, environment);
    ignition::math::Vector3d up(-1, 0, 0);
    ignition::math::Vector3d forward(0, 0, 1);
    lift_drag_model_.setBasisVectors(forward, up);
  }
};

// TEST_F(LiftDragModelTest, TestNullSettingLateralArea) {
//   //This is now just a death test to ensure that NaN is handled correctly.
//   ASSERT_DEATH({
//     lift_drag_model_.setLateralArea(std::nan);
//   }, "");
// }

// TEST_F(LiftDragModelTest, TestNullSettingLateralAreaVeloctiy) {
//   ASSERT_DEATH({
//     lift_drag_model_.setLateralVelocity(std::nan);
//   }, "");
// }

// TEST_F(LiftDragModelTest, TestNullSettingPlanarVelocity) {
//   ASSERT_DEATH({
//     lift_drag_model_.setPlanarVelocity(std::nan);
//   }, "");
// }

// TEST_F(LiftDragModelTest, TestNullSettingFreestreamVelocity) {
//   //This is now just a death test to ensure that NaN is handled correctly.
//   ASSERT_DEATH({
//     lift_drag_model_.setFreeStreamVelocity(std::nan);
//   }, "");
// }

TEST_F(LiftDragModelTest, TestDynamicPressure) {
  // Given: Planar Velocity and Air Density
  double velocity_m_per_s = 20;

  // When: Dynamic Pressure is Calculated
  double dynamicPressure_Pa = lift_drag_model_.calculateDynamicPressure_Pa(velocity_m_per_s);

  // Then: Dynamic Pressure should match expected
  ASSERT_NEAR(dynamicPressure_Pa, 244, tolerance);
}

TEST_F(LiftDragModelTest, TestCalculatingWindAngles) {
  // Given: Velocity in body
  ignition::math::Vector3d velocityInBody_m_per_s = ignition::math::Vector3d(0.81165, -0.90368, 9.977);

  // When: Wind Angles are Calculated
  std::pair<double, double> aeroAngles_deg = lift_drag_model_.calculateWindAngles(velocityInBody_m_per_s);

  // Then, Wind Angles should be correct
  ASSERT_NEAR(aeroAngles_deg.first, 4.651,
              tolerance);
  ASSERT_NEAR(aeroAngles_deg.second, -5.1587,
              tolerance);
}

TEST_F(LiftDragModelTest, TestCalculatingLocalVelocities) {
  // Given: Initial Pose In World and Velocity
  ignition::math::Pose3d poseInWorld_m_rad = ignition::math::Pose3d(0, 0, 0, -0.09, 1.48,
                                0.1);
  ignition::math::Vector3d velocityInWorld_m_per_s = ignition::math::Vector3d(10, 1, 0.1);


  // When: Local Velocities are calculate
  ignition::math::Vector3d velocityInBody_m_per_s = lift_drag_model_.transformToLocalVelocity(poseInWorld_m_rad, velocityInWorld_m_per_s);

  // Then: Velocities should match expected
  ASSERT_NEAR(velocityInBody_m_per_s.X(), 0.81165, tolerance);
  ASSERT_NEAR(velocityInBody_m_per_s.Y(), -0.90368, tolerance);
  ASSERT_NEAR(velocityInBody_m_per_s.Z(), 9.977, tolerance);
}

TEST_F(LiftDragModelTest, TestRotatingForcesToBody) {
  // Given: Initial Pose In World and Velocity
  double lift_N = 31.269;
  double drag_N = 0.821;
  double lateralForce_N = 0.005;
  double angleOfAttack_deg = 4.65089;
  double sideSlipAngle_deg = -5.15857;


  // When: Local Velocities are calculate
  ignition::math::Vector3d force_N = lift_drag_model_.rotateForcesToBody(lift_N, drag_N, lateralForce_N, angleOfAttack_deg, sideSlipAngle_deg);

  // Then: Velocities should match expected
  ASSERT_NEAR(force_N.X(), -31.232, tolerance);
  ASSERT_NEAR(force_N.Y(), 0.005, tolerance);
  ASSERT_NEAR(force_N.Z(), 1.717, tolerance);
}

TEST_F(LiftDragModelTest, TestRotatingForcesToBodyLargeAttack) {
  // Given: Initial Pose In World and Velocity
  double lift_N = -8.06931;
  double drag_N = 9.26476;
  double lateralForce_N = 0;
  double angleOfAttack_deg = 135;
  double sideSlipAngle_deg = -0.589718;

  // When: Local Velocities are calculate
  ignition::math::Vector3d force_N = lift_drag_model_.rotateForcesToBody(lift_N, drag_N, lateralForce_N, angleOfAttack_deg, sideSlipAngle_deg);

  // Then: Velocities should match expected
  ASSERT_NEAR(force_N.X(), -12.257, tolerance);
  ASSERT_NEAR(force_N.Y(), 0, tolerance);
  ASSERT_NEAR(force_N.Z(), 0.845313, tolerance);
}


