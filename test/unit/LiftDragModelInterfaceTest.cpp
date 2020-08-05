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

// #define ASSERT_PERCENT_NEAR(value, tolerance)

struct CalcForcesParams {
  std::string case_name;
  ignition::math::Vector3d position_world_m;
  ignition::math::Vector3d rotation_world_rad;
  ignition::math::Vector3d velocity_world_m_per_s;
  ignition::math::Vector3d expectedForce_body_N;
};



class LiftDragModelInterfaceTest : public ::testing::Test {
 protected:
  // Some expensive resource shared by all tests.
  // static T* shared_resource_;



  avionics_sim::Lift_drag_model lift_drag_model_;
  double tolerance = 0.1;

  /**
   * Sets up the test suite, called before all test cases are run
   */
  static void SetUpTestSuite() {
  }

  /**
   * Tears down the test suite, called after all test cases are run
   */
  static void TearDownTestSuite() {
    // delete shared_resource_;
  }

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

  // You can define per-test tear-down logic as usual.
  virtual void TearDown() {
    // delete lift_drag_model_;
  }

  // void LogStreamProperty(std::string key, std::ostream stream) {
  //   std::ostringstream string_store;
  //   string_store << stream;
  //   RecordProperty("Pose_world_m: ",  string_store.str());
  // }

  virtual void RecordSetup() {
    RecordProperty("--- Setup",  "------------------");
    // std::ostringstream string_store;

    RecordProperty("Area [m2]: ", lift_drag_model_.getArea());
    RecordProperty("Area_lateral [m2]: ", lift_drag_model_.getLateralArea());
    RecordProperty("Air Density [kg/m3]: ",
                   lift_drag_model_.getAirDensity_kg_per_m3());
    RecordProperty("Airfoil: ", "NACA0012");

    // string_store << lift_drag_model_.vecUpwd;
    // RecordProperty("Up Vector: ");
    // string_store.str("");
    // string_store << lift_drag_model_.vecFwd;
    // RecordProperty("Forward Vector: ");
  }

  virtual void RecordInput() {
    RecordProperty("--- Input",  "------------------");

    std::ostringstream string_store;
    // LogStreamProperty("Pose_world_m: ", "" << lift_drag_model_.getWorldPose() << "[m, m, m, rad, rad, rad]")
    string_store << lift_drag_model_.getWorldPose();
    RecordProperty("Pose_world[m, rad]: ",  string_store.str());
    string_store.str("");
    string_store << lift_drag_model_.getWorldVelocity();
    RecordProperty("Veloctiy_world [m/s]: ", string_store.str());
  }

  virtual void RecordTransient() {
    RecordProperty("--- Transient",  "------------------");
    RecordProperty("Angle of Attack[deg]: ",
                   std::to_string(lift_drag_model_.getAngleOfAttack_deg()));
    RecordProperty("Lift Coefficient: ", std::to_string(lift_drag_model_.getCL()));
    RecordProperty("Drag Coefficient: ", std::to_string(lift_drag_model_.getCD()));

    RecordProperty("Lift [N]: ", std::to_string(lift_drag_model_.getLift()));
    RecordProperty("Drag [N]: ", std::to_string(lift_drag_model_.getDrag()));
  }

  virtual void RecordFinal(ignition::math::Vector3d force_N,
                           ignition::math::Vector3d expectedForce_N) {
    RecordProperty("--- Final",  "------------------");
    std::ostringstream string_store;
    string_store << force_N;
    RecordProperty("Actual Force[N]: ", string_store.str());

    string_store.str("");
    string_store << expectedForce_N;
    RecordProperty("Expected Force[N]: ", string_store.str());
  }
};

class CalcForcesParamTest :
  public LiftDragModelInterfaceTest,
  public testing::WithParamInterface<CalcForcesParams> {

};

const std::vector<CalcForcesParams> params{
  {
    "Home Work Problem \340 = 4\370",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-0.09, 1.48, 0.1),
    ignition::math::Vector3d(10, 1, 0.1),
    ignition::math::Vector3d(-31.232, -0.005, 1.717)
  },
  {
    "Failure mode of atan, \340 = ± 90\370​",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(0, 1.5708, 0),
    ignition::math::Vector3d(0, 0, -10),
    ignition::math::Vector3d(-109.8, 0, 4.27)
  },
  {
    "At stall, \340 = ± 9\370",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-0.08, 1.48, 0.1),
    ignition::math::Vector3d(9.98, 0.99, -0.66),
    ignition::math::Vector3d(-51.668, -0.005, 6.927)
  },
  {
    "Post stall, \340 = 10\370",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-0.09, 1.48, 0.1),
    ignition::math::Vector3d(9.967, 0.986, -0.8355),
    ignition::math::Vector3d(-8.1747, -0.005, 0.2747)
  },
  {
    "Sideways, \340 = 4\370",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-1.5708, 1.5708, 0),
    ignition::math::Vector3d(0.904, 9.985, -0.698),
    ignition::math::Vector3d(-26.88, -0.005, 1.12)
  },
  {
    "Upside Down, \340 = 4\370",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-3.1416, 1.5708, 0),
    ignition::math::Vector3d(-9.985, 0.904, -0.698),
    ignition::math::Vector3d(-26.88, -0.005, 1.12)
  },
};

INSTANTIATE_TEST_CASE_P(LiftDragModelInterface,
                        CalcForcesParamTest,
                        ::testing::ValuesIn(params));

TEST_P(CalcForcesParamTest, AerodynamicForcesFromVelocityAndOrientation) {
  CalcForcesParams params = GetParam();
  RecordProperty("Test Case: ", params.case_name);
  RecordSetup();

  try {
    // Given:
    // Initial Position, Rotation and Velocity from params

    // Combined to form the pose:
    ignition::math::Pose3d pose_world(params.position_world_m,
                                      ignition::math::Quaterniond(params.rotation_world_rad));

    // And Set in the model
    lift_drag_model_.setWorldPose(pose_world);
    lift_drag_model_.setWorldVelocity(params.velocity_world_m_per_s);
    RecordInput();

    // When: // TODO reduce this down to one function call
    lift_drag_model_.updateForcesInBody_N(pose_world,
                                          params.velocity_world_m_per_s);
    RecordTransient();

    // Then:
    // The Resultant Forces should equal the expected forces
    ignition::math::Vector3d force_N = lift_drag_model_.getForceVector();
    // ignition::math::Vector3d force_N(-31.232, -0.005, 1.717);
    ignition::math::Vector3d expectedForce_N = params.expectedForce_body_N;
    // with minimal tolerance TODO: Change tolerance to percent based
    const double tolerance = 0.1;
    RecordFinal(force_N, expectedForce_N);



    ASSERT_NEAR(force_N.X(), expectedForce_N.X(), tolerance);
    ASSERT_NEAR(force_N.Y(), expectedForce_N.Y(), tolerance);
    ASSERT_NEAR(force_N.Z(), expectedForce_N.Z(), tolerance);
  } catch (const Lift_drag_model_exception &e) {
    ADD_FAILURE() << "Uncaught exception during HomeworkProblemTest" << std::endl <<
                  e.what() << std::endl;
  }
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/

struct PropWashCaseParams {
  std::string case_name;
  double planarVelocity_m_per_s;
  double angleOfAttack_deg;
  ignition::math::Vector3d expectedForce_body_N;
};

class CalcForcesInPropWashParamTest :
  public LiftDragModelInterfaceTest,
  public testing::WithParamInterface<PropWashCaseParams> {

 protected:
  virtual void RecordInput() {
    RecordProperty("--- Input",  "------------------");

    RecordProperty("Freestream Velocity [m/s]: ",
                   lift_drag_model_.getFreeStreamVelocity());
    RecordProperty("Angle of Attack [deg]: ",
                   lift_drag_model_.getAngleOfAttack_deg());
  }

};

const std::vector<PropWashCaseParams> propWashCasesParams{
  {
    "Home Work Problem, angle of attack = 4 [deg]",
    10.01,
    4.651,
    ignition::math::Vector3d(-31.232, -0.005, 1.717)
  },
  {
    "At vertical climb stall, when aircraft speed is 0",
    0,
    4.651,
    ignition::math::Vector3d(0, 0, 0)
  }
};

INSTANTIATE_TEST_CASE_P(LiftDragModelInterfaceTest,
                        CalcForcesInPropWashParamTest,
                        ::testing::ValuesIn(propWashCasesParams));

TEST_P(CalcForcesInPropWashParamTest,
       AerodynamicForcesFromPropWashAndSurfaceDeflection) {
  PropWashCaseParams params = GetParam();
  RecordProperty("Test Case: ", params.case_name);
  RecordSetup();

  try {
    // Given:
    double planarVelocity_m_per_s = params.planarVelocity_m_per_s;
    double angleOfAttack_deg = params.angleOfAttack_deg;

    // And Set in the model
    lift_drag_model_.setFreeStreamVelocity(planarVelocity_m_per_s);
    lift_drag_model_.setAngleOfAttack(angleOfAttack_deg);
    RecordInput();

    // When: // TODO reduce this down to one function call
    ignition::math::Vector3d force_N = lift_drag_model_.updateForcesInBody_N(
                                         planarVelocity_m_per_s,
                                         planarVelocity_m_per_s,
                                         angleOfAttack_deg);
    RecordTransient();

    // Then:
    // The Resultant Forces should equal the expected forces
    ignition::math::Vector3d expectedForce_N = params.expectedForce_body_N;
    RecordFinal(force_N, expectedForce_N);

    ASSERT_NEAR(force_N.X(), expectedForce_N.X(), tolerance);
    ASSERT_NEAR(force_N.Y(), expectedForce_N.Y(), tolerance);
    ASSERT_NEAR(force_N.Z(), expectedForce_N.Z(), tolerance);
  } catch (const Lift_drag_model_exception &e) {
    ADD_FAILURE() << "Uncaught exception calculating forces with propwash" <<
                  std::endl <<
                  e.what() << std::endl;
  }
}


