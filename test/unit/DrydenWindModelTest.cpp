/**
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "TestUtils.hpp"

#include "DrydenWindModel.hpp"

namespace avionics_sim {

class MockProvider : public IDrydenProvider {
  public:
    virtual DrydenState get_dryden_input() {
        DrydenState state;
        state.velocity_m_per_s = v3(9.85, -0.72, 0.12);
        state.altitude_m = 457.2;
        return state;
    }
};

TEST(DrydenWindModelTest, Test_Blank_Initialization) {
    DrydenWindModel wind_model;

    WindRate wind_rate = wind_model.get_rates();

    ASSERT_TRUE(&wind_model != nullptr);
    EXPECT_NEAR(wind_rate.linear_rate.Length(), 0, epsilon);
}

TEST(DrydenWindModelTest, Test_Provider) {
    MockProvider provider;
    std::default_random_engine random_generator(1);

    DrydenWindModel wind_model(random_generator, (IDrydenProvider &) provider);

    WindRate wind_rate = wind_model.get_rates();

    EXPECT_NEAR(wind_rate.linear_rate.Length(), 0, epsilon);
}

TEST(DrydenWindModelTest, Test_Complete_Initialization) {
    MockProvider provider;
    std::default_random_engine random_generator(1);
    double wingspan_m = 3.96;
    double turbulence_intensity = MODERRATE_TURBULENCE_INTENSITY_m_per_s;

    DrydenWindModel wind_model(
        random_generator,
        (IDrydenProvider &) provider,
        wingspan_m,
        turbulence_intensity);

    // noise: -0.121966, 0.68429, -1.08682
    WindRate wind_rate = wind_model.get_rates();

    EXPECT_V3_NEAR(wind_rate.linear_rate, v3(-0.00189358, -0.0238626, 0.01724515), epsilon);
}

TEST(DrydenWindModelTest, Test_Simulation) {
    MockProvider provider;
    std::default_random_engine random_generator(1);
    double wingspan_m = 3.96;
    double turbulence_intensity = MODERRATE_TURBULENCE_INTENSITY_m_per_s;

    FILE *test_results_file = fopen("dryden_sim_linear_rates.csv", "w");

    DrydenWindModel wind_model(
        random_generator,
        (IDrydenProvider &) provider,
        wingspan_m,
        turbulence_intensity);

    std::vector<std::pair<double, WindRate>> wind_rates;

    double sample_period_s = 0.0125;
    double simulation_time_s = 1000;
    double num_steps = simulation_time_s / sample_period_s;

    wind_model.set_sample_period(sample_period_s);

    for (int i = 0; i < num_steps; i++) {
        double time_s = i * sample_period_s;
        WindRate wind_rate = wind_model.get_rates();
        wind_rates.push_back(std::pair<double, WindRate>(time_s, wind_rate));

        fprintf(test_results_file, "%f, %f, %f, %f\n", time_s, wind_rate.linear_rate.X(), wind_rate.linear_rate.Y(),
                wind_rate.linear_rate.Z());

        ASSERT_TRUE(!isnan(wind_rate.linear_rate.X()));
        ASSERT_TRUE(!isnan(wind_rate.linear_rate.Y()));
        ASSERT_TRUE(!isnan(wind_rate.linear_rate.Z()));
    }

    fclose(test_results_file);
}

}  // namespace avionics_sim
