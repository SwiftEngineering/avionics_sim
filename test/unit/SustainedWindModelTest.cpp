#include "TestUtils.hpp"

#include "SustainedWindModel.hpp"

using namespace avionics_sim;

TEST(SustainedWindModelTest, Test_Blank_Initialization) {
  SustainedWindModel wind_model;

  WindRate wind_rate = wind_model.get_rates();

  ASSERT_TRUE(&wind_model != nullptr);
  EXPECT_NEAR(wind_rate.linear_rate.Length(), 0, epsilon);
}

TEST(SustainedWindModelTest, Test_Strength_Direction_Initialization) {
  SustainedWindModel wind_model(2, v3(0,2,0));

  WindRate wind_rate = wind_model.get_rates();

  EXPECT_NEAR(wind_rate.linear_rate.Length(), 2, epsilon);
  EXPECT_V3_NEAR(wind_rate.linear_rate.Normalize(), v3(0,1,0), epsilon);
}

TEST(SustainedWindModelTest, Test_Change_Of_Strength) {
  SustainedWindModel wind_model(2, v3(0,2,0));

  wind_model.set_strength(10);

  WindRate wind_rate = wind_model.get_rates();

  EXPECT_NEAR(wind_rate.linear_rate.Length(), 10, epsilon);
  EXPECT_V3_NEAR(wind_rate.linear_rate.Normalize(), v3(0,1,0), epsilon);
}


TEST(SustainedWindModelTest, Test_Change_Of_Direction) {
  SustainedWindModel wind_model(2, v3(0,2,0));

  wind_model.set_direction(v3(10,0,0));

  WindRate wind_rate = wind_model.get_rates();

  EXPECT_NEAR(wind_rate.linear_rate.Length(), 2, epsilon);
  EXPECT_V3_NEAR(wind_rate.linear_rate.Normalize(), v3(1,0,0), epsilon);
}
