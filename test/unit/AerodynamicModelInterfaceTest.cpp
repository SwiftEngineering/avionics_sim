#include <gtest/gtest.h>
// #include <gmock/gmock.h>

#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "AerodynamicModel.hpp"
#include "Lift_drag_model_exception.hpp"

#define PERCENT_DIFF_100(a,b) ((a == 0) && (b==0)) ? 0 : 2.0 * abs(a-b) / (a+b) * 100.0
#define EXPECT_PERCENT_DIFF_LT(a, b, tolerance_100) EXPECT_LT(PERCENT_DIFF_100(a,b), tolerance_100)


using namespace avionics_sim;

struct WorldFrameCaseParams {
  std::string case_name;
  ignition::math::Vector3d position_world_m;
  ignition::math::Vector3d rotation_world_rad;
  double propWash_m_per_s;
  double controlAngle_rad;
  ignition::math::Vector3d velocity_world_m_per_s;
  ignition::math::Vector3d expectedForce_body_N;
};

class AerodynamicModelInterfaceTest : public ::testing::Test, IPhysicsEnvironment {
  public:
    virtual double get_air_density_kg_per_m3() {
      return 1.22;
    }

  protected:
  // Some expensive resource shared by all tests.
  // static T* shared_resource_;



  avionics_sim::AerodynamicModel aerodynamic_model_;
  double tolerance_100 = 0.5; /**< Allowable percent tolerance, note this is 0.5%, not 50% */

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
    Bilinear_interp::get1DLUTelementsFromString(alphaLUTString,
        &LUT_NACA0012_alpha);
    Bilinear_interp::get1DLUTelementsFromString(alphaLUTCL,
        &LUT_NACA0012_CL);
    Bilinear_interp::get1DLUTelementsFromString(alphaLUTCD,
        &LUT_NACA0012_CD);

    // Body/Profile Configuration
    double area_m2 = 1.0;
    double area_lateral_m2 = 0.1;
    Airfoil airfoil = Airfoil(
      area_m2, area_lateral_m2,
      LUT_NACA0012_alpha, LUT_NACA0012_CL, LUT_NACA0012_CD);

    // Initialize aerodynamic model for new test
    aerodynamic_model_ = avionics_sim::AerodynamicModel(airfoil, *this);
    ignition::math::Vector3d up(-1, 0, 0);
    ignition::math::Vector3d forward(0, 0, 1);
    aerodynamic_model_.setBasisVectors(forward, up);
  }

  // You can define per-test tear-down logic as usual.
  virtual void TearDown() {
    // delete aerodynamic_model_;
  }

  // void LogStreamProperty(std::string key, std::ostream stream) {
  //   std::ostringstream string_store;
  //   string_store << stream;
  //   RecordProperty("Pose_world_m: ",  string_store.str());
  // }

  virtual void RecordSetup() {
    RecordProperty("Setup",  "------------------");
    // std::ostringstream string_store;

    // RecordProperty("Area [m2]: ", aerodynamic_model_.getArea());
    // RecordProperty("Area_lateral [m2]: ", aerodynamic_model_.getLateralArea());
    // RecordProperty("Air Density [kg/m3]: ",
    //                aerodynamic_model_.getAirDensity_kg_per_m3());
    // RecordProperty("Airfoil: ", "NACA0012");
  }

  virtual void RecordInput() {
    RecordProperty("Input",  "------------------");

    // std::ostringstream string_store;
    // string_store << aerodynamic_model_.getWorldPose();
    // RecordProperty("Pose_world[m, rad]: ",  string_store.str());
    // string_store.str("");
    // string_store << aerodynamic_model_.getWorldVelocity();
    // RecordProperty("Veloctiy_world [m/s]: ", string_store.str());
  }

  virtual void RecordTransient() {
    RecordProperty("Transient",  "------------------");
    // RecordProperty("Angle of Attack[deg]: ",
    //                std::to_string(aerodynamic_model_.getAngleOfAttack_deg()));
    // RecordProperty("Lift Coefficient: ", std::to_string(aerodynamic_model_.getCL()));
    // RecordProperty("Drag Coefficient: ", std::to_string(aerodynamic_model_.getCD()));

    // RecordProperty("Lift [N]: ", std::to_string(aerodynamic_model_.getLift()));
    // RecordProperty("Drag [N]: ", std::to_string(aerodynamic_model_.getDrag()));
  }

  virtual void RecordFinal(ignition::math::Vector3d force_N,
                           ignition::math::Vector3d expectedForce_N) {
    RecordProperty("Final",  "------------------");
    // std::ostringstream string_store;
    // string_store << force_N;
    // RecordProperty("Actual Force[N]: ", string_store.str());

    // string_store.str("");
    // string_store << expectedForce_N;
    // RecordProperty("Expected Force[N]: ", string_store.str());
  }
};

class CalcForcesParamTest :
  public AerodynamicModelInterfaceTest,
  public testing::WithParamInterface<WorldFrameCaseParams> {

};

const std::vector<WorldFrameCaseParams> params{
  {
    "Home Work Problem alpha = 4 deg, beta = -5.1587 deg",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-0.09, 1.48, 0.1),
    0,
    0,
    ignition::math::Vector3d(10, 1, 0.1),
    ignition::math::Vector3d(-31.232, 0.005, 1.717)
  },
  {
    "Home Work Problem alpha = 4 deg, beta = 5.1587 deg",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-0.09, 1.48, 0.1),
    0,
    0,
    ignition::math::Vector3d(9.659, 2.775, 0.085),
    ignition::math::Vector3d(-31.232, -0.005, 1.717)
  },
  {
    "Failure mode of atan, alpha = ± 90 deg​",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(0, 1.5708, 0),
    0,
    0,
    ignition::math::Vector3d(0, 0, -10),
    ignition::math::Vector3d(-109.8, 0, 4.27)
  },
  {
    "At stall, alpha = ± 9 deg",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-0.08, 1.48, 0.1),
    0,
    0,
    ignition::math::Vector3d(9.98, 0.99, -0.66),
    ignition::math::Vector3d(-51.668, -0.005, 6.927)
  },
  // {
  //   "Post stall, alpha = 10 deg",
  //   ignition::math::Vector3d(0, 0, 0),
  //   ignition::math::Vector3d(-0.09, 1.48, 0.1),
  //   ignition::math::Vector3d(9.967, 0.986, -0.8355),
  //   ignition::math::Vector3d(-8.1747, -0.005, 0.2747)
  // },
  {
    "Sideways, alpha = 4 deg",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-1.5708, 1.5708, 0),
    0,
    0,
    ignition::math::Vector3d(0.904, 9.985, -0.698),
    ignition::math::Vector3d(-26.88, -0.005, 1.12)
  },
  {
    "Upside Down, alpha = 4 deg",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(-3.1416, 1.5708, 0),
    0,
    0,
    ignition::math::Vector3d(-9.985, 0.904, -0.698),
    ignition::math::Vector3d(-26.88, -0.005, 1.12)
  },
  {
    "Wind Behind No Control Surface",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(0.010431, 0.161863, -2.36131),
    0,
    0,
    ignition::math::Vector3d(0, 0, -20),
    ignition::math::Vector3d(-204.097, 0.000259, -1.76903)
  },
  {
    "Wind Behind with Control Surface No PropWash",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(0.010431, 0.161863, -2.36131),
    0,
    -0.785398,
    ignition::math::Vector3d(0, 0, -20),
    ignition::math::Vector3d(-384.905, 0.000259, 21.3478)
  },
  {
    "Wind Behind With Control Surface With Propwash",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(0.010431, 0.161863, -2.36131),
    24.1884,
    -0.785398,
    ignition::math::Vector3d(0, 0, -20),
    ignition::math::Vector3d(16.3525, 0, 0.075705)
  },
    {
    "Wind Behind With Control Surface With Little Propwash",
    ignition::math::Vector3d(0, 0, 0),
    ignition::math::Vector3d(0.010431, 0.161863, -2.36131),
    5,
    -0.785398,
    ignition::math::Vector3d(0, 0, -20),
    ignition::math::Vector3d(-216.501, 0.000259, 12.0077)
  }
};

INSTANTIATE_TEST_CASE_P(AerodynamicModelInterface,
                        CalcForcesParamTest,
                        ::testing::ValuesIn(params));

TEST_P(CalcForcesParamTest, AerodynamicForcesFromVelocityAndOrientation) {
  WorldFrameCaseParams params = GetParam();
  RecordProperty("Case", params.case_name);
  RecordSetup();

  // Given:
  // Initial Position, Rotation and Velocity from params

  // Combined to form the pose:
  ignition::math::Pose3d pose_world(params.position_world_m,
                                    ignition::math::Quaterniond(params.rotation_world_rad));
  // RecordInput();

  // When: // TODO reduce this down to one function call
  ignition::math::Vector3d force_N = aerodynamic_model_.updateForcesInBody_N(
                                      pose_world,
                                      params.velocity_world_m_per_s,
                                      params.propWash_m_per_s,
                                      params.controlAngle_rad);
  RecordTransient();

  // // Then:
  // // The Resultant Forces should equal the expected forces
  // ignition::math::Vector3d expectedForce_N = params.expectedForce_body_N;

  // RecordFinal(force_N, expectedForce_N);

  // EXPECT_PERCENT_DIFF_LT(force_N.X(), expectedForce_N.X(), tolerance_100);
  // EXPECT_PERCENT_DIFF_LT(force_N.Y(), expectedForce_N.Y(), tolerance_100);
  // EXPECT_PERCENT_DIFF_LT(force_N.Z(), expectedForce_N.Z(), tolerance_100);
}

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/

struct BodyFrameCaseParams {
  std::string case_name;
  double planarVelocity_m_per_s;
  double lateralVelocity_m_per_s;
  double angleOfAttack_deg;
  double sideSlipAngle_deg;
  ignition::math::Vector3d expectedForce_body_N;
};

class CalcForcesInPropWashParamTest :
  public AerodynamicModelInterfaceTest,
  public testing::WithParamInterface<BodyFrameCaseParams> {

 protected:
  virtual void RecordInput() {
    RecordProperty("Input",  "------------------");

    // RecordProperty("Freestream Velocity [m/s]: ",
    //                aerodynamic_model_.getFreeStreamVelocity());
    // RecordProperty("Angle of Attack [deg]: ",
    //                aerodynamic_model_.getAngleOfAttack_deg());
  }

};

const std::vector<BodyFrameCaseParams> propWashCasesParams{
  {
    "Home Work Problem, angle of attack = 4 [deg], side slip angle = -5.1587 [deg]",
    10.01,
    -0.9037,
    4.651,
    -5.1587,
    ignition::math::Vector3d(-31.232, 0.005, 1.717)
  },
  {
    "At vertical climb stall, when aircraft speed is 0",
    0,
    0,
    4.651,
    0,
    ignition::math::Vector3d(0, 0, 0)
  }
};

// INSTANTIATE_TEST_CASE_P(AerodynamicModelInterfaceTest,
//                         CalcForcesInPropWashParamTest,
//                         ::testing::ValuesIn(propWashCasesParams));

// TEST_P(CalcForcesInPropWashParamTest,
//        AerodynamicForcesFromPropWashAndSurfaceDeflection) {
//   BodyFrameCaseParams params = GetParam();
//   RecordProperty("Case", params.case_name);
//   RecordSetup();

//   // Given:
//   double planarVelocity_m_per_s = params.planarVelocity_m_per_s;
//   double lateralVelocity_m_per_s = params.lateralVelocity_m_per_s;
//   double angleOfAttack_deg = params.angleOfAttack_deg;
//   double sideSlipAngle_deg = params.sideSlipAngle_deg;

//   // And Set in the model
//   RecordInput();

//   // When: // TODO reduce this down to one function call
//   ignition::math::Vector3d force_N = aerodynamic_model_.updateForcesInBody_N(
//                                         planarVelocity_m_per_s,
//                                         lateralVelocity_m_per_s,
//                                         angleOfAttack_deg,
//                                         sideSlipAngle_deg);
//   RecordTransient();

//   // Then:
//   // The Resultant Forces should equal the expected forces
//   ignition::math::Vector3d expectedForce_N = params.expectedForce_body_N;
//   RecordFinal(force_N, expectedForce_N);

//   EXPECT_PERCENT_DIFF_LT(force_N.X(), expectedForce_N.X(), tolerance_100);
//   EXPECT_PERCENT_DIFF_LT(force_N.Y(), expectedForce_N.Y(), tolerance_100);
//   EXPECT_PERCENT_DIFF_LT(force_N.Z(), expectedForce_N.Z(), tolerance_100);
// }
