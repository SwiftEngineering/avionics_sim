#include <gtest/gtest.h>
// #include <gmock/gmock.h>

#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "Lift_drag_model.hpp"
#include "Lift_drag_model_exception.hpp"

class LiftDragModelTest : public ::testing::Test {
 protected:
  avionics_sim::Lift_drag_model lift_drag_model_;
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


    // Initialize lift drag model for new test
    lift_drag_model_ = avionics_sim::Lift_drag_model();

    // all the LUT generation to objects should be reduced down
    double area_m2 = 1.0;
    double area_lateral_m2 = 0.1;
    double airDensity_kg_per_m3 = 1.22;
    ignition::math::Vector3d up(-1, 0, 0);
    ignition::math::Vector3d forward(0, 0, 1);

    lift_drag_model_.setLUTs(LUT_NACA0012_alpha, LUT_NACA0012_CL, LUT_NACA0012_CD);

    // Environment Configuration
    lift_drag_model_.setAirDensity(airDensity_kg_per_m3);

    // Body/Profile Configuration
    lift_drag_model_.setBasisVectors(forward, up);
    lift_drag_model_.setArea(area_m2);
    lift_drag_model_.setLateralArea(area_lateral_m2);
    lift_drag_model_.setLUTs(LUT_NACA0012_alpha, LUT_NACA0012_CL, LUT_NACA0012_CD);
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
  lift_drag_model_.setPlanarVelocity(20);
  lift_drag_model_.setAirDensity(1.22);

  // When: Dynamic Pressure is Calculated
  lift_drag_model_.calculateDynamicPressure();

  // Then: Dynamic Pressure should match expected
  ASSERT_NEAR(lift_drag_model_.getDynamicPressure(), 244, tolerance);
}

TEST_F(LiftDragModelTest, TestLookupCD) {
// Given: Initial angle of attack
  double expectedCD = 0.013442;
  double angleOfAttack_deg = 4.651;
  lift_drag_model_.setAngleOfAttack(angleOfAttack_deg);

  // When: CL is looked up
  lift_drag_model_.updateCD();

  // Then: Resultant CL should match expected
  ASSERT_NEAR(lift_drag_model_.getCD(), expectedCD, tolerance);
}

TEST_F(LiftDragModelTest, TestLookupCL) {
  // Given: Initial angle of attack
  double expectedCL = 0.511614;
  double angleOfAttack_deg = 4.651;
  lift_drag_model_.setAngleOfAttack(angleOfAttack_deg);

  // When: CL is looked up
  lift_drag_model_.updateCL();

  // Then: Resultant CL should match expected
  ASSERT_NEAR(lift_drag_model_.getCL(), expectedCL, tolerance);
}

// TEST_F(LiftDragModelTest, TestCalculatingWindAngles) {
//   // Given: Initial Pose In World and Velocity
//   lift_drag_model_.setWorldPose(ignition::math::Pose3d(0, 0, 0, -0.09, 1.48,
//                                 0.1));
//   lift_drag_model_.setWorldVelocity(ignition::math::Vector3d(10, 1, 0.1));


//   // When: Wind Angles are Calculated
//   lift_drag_model_.updateWindAnglesAndLocalVelocities();

//   // Then, Wind Angles should be correct
//   ASSERT_NEAR(lift_drag_model_.getAngleOfAttack_deg(), 4.651,
//               tolerance);
//   ASSERT_NEAR(lift_drag_model_.getSideSlipAngle_deg(), 5.1587,
//               tolerance);
// }

TEST_F(LiftDragModelTest, TestCalculatingCalculatingLocalVelocities) {
  // Given: Initial Pose In World and Velocity
  lift_drag_model_.setWorldPose(ignition::math::Pose3d(0, 0, 0, -0.09, 1.48,
                                0.1));
  lift_drag_model_.setWorldVelocity(ignition::math::Vector3d(10, 1, 0.1));


  // When: Local Velocities are calculate
  lift_drag_model_.calculateLocalVelocities();

  // Then: Velocities should match expected
  ASSERT_NEAR(lift_drag_model_.getFreeStreamVelocity(), 10.0504, tolerance);
  ASSERT_NEAR(lift_drag_model_.getPlanarVelocity_m_per_s(), 10.01, tolerance);
  ASSERT_NEAR(lift_drag_model_.getLateralVelocity(), -0.90368, tolerance);
}