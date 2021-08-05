/**
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */
#include "Differential_pressure_sensor_model.hpp"

#include <gtest/gtest.h>

// Test conversion from indicated airspeed to differential pressure for non-compressable flow
TEST(Differential_pressure_UnitTest, Test_diff_press_from_indicated) {
    avionics_sim::Differential_pressure_sensor_model dp(0.0);
    std::array<double, 6> indicated = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};
    double computed_diff_press;
    std::array<double, 6> expected = {0.0, 61.25, 245.0, 551.25, 980.0, 1531.25};

    for (size_t i = 0; i < indicated.size(); i++) {
        computed_diff_press = dp.diff_press_from_indicated_airspeed(indicated[i]);
        ASSERT_DOUBLE_EQ(computed_diff_press, expected[i]);
    }
}

// Test conversion from corrected airspeed to indicated airspeed
TEST(Differential_pressure_UnitTest, Test_indicated_from_corrected) {
    avionics_sim::Differential_pressure_sensor_model dp(0.0);
    std::array<double, 6> corrected = {0.0, 10.0, 20.0, 30.0, 40.0, 50.0};
    std::array<double, 9> correction = {-2.0, -1.5, -1.0, -0.5, 0, 0.5, 1.0, 1.5, 2.0};
    std::array<std::array<double, 6>, 9> expected = {{
            {-2.0, 8.0, 18.0, 28.0, 38.0, 48.0}, {-1.5, 8.5, 18.5, 28.5, 38.5, 48.5},
            {-1.0, 9.0, 19.0, 29.0, 39.0, 49.0}, {-0.5, 9.5, 19.5, 29.5, 39.5, 49.5},
            {-0.0, 10.0, 20.0, 30.0, 40.0, 50.0}, {0.5, 10.5, 20.5, 30.5, 40.5, 50.5},
            {1.0, 11.0, 21.0, 31.0, 41.0, 51.0}, {1.5, 11.5, 21.5, 31.5, 41.5, 51.5},
            {2.0, 12.0, 22.0, 32.0, 42.0, 52.0}
        }
    };
    double calculated_indicated;

    for (size_t i = 0; i < correction.size(); i++) {
        for (size_t j = 0; j < corrected.size(); j++) {
            dp.set_pitot_correction(correction[i]);
            calculated_indicated = dp.indicated_airspeed_from_corrected_airspeed(
                                       corrected[j]);
            ASSERT_DOUBLE_EQ(calculated_indicated, expected[i][j]);
        }
    }
}

// Test conversion from true airspeed to indicated airspeed
TEST(Differential_pressure_UnitTest,
     Test_corrected_airspeed_from_true_airspeed) {
    avionics_sim::Differential_pressure_sensor_model dp(0.0);
    std::array<double, 3> true_airspeed = {0.0, 25.0, 50.0};
    std::array<double, 2> static_pressure = {80000, 105000};
    std::array<double, 2> temperature = {-10.0, 40.0};
    double calculated_true;
    std::array<std::array<std::array<double, 3>, 2>, 2> expected =  {{{{{0.0, 23.243347457605907,  46.486694915211814},
                    {0, 26.628599782159611, 53.257199564319222}
                }
            },
            {   {   {0, 21.307090649235157, 42.614181298470314},
                    {0, 24.41033893485147,  48.820677869702941}
                }
            }
        }
    };

    for (size_t i = 0; i < true_airspeed.size(); i++) {
        for (size_t j = 0; j < static_pressure.size(); j++) {
            for (size_t k = 0; k < temperature.size(); k++) {
                calculated_true = dp.corrected_airspeed_from_true_airspeed(true_airspeed[i],
                                  static_pressure[j], temperature[k]);
                ASSERT_DOUBLE_EQ(expected[k][j][i], calculated_true);
            }
        }
    }
}

// Ensure compressable and incompressable flow equations are within 1% up to 50m/s
TEST(Differential_pressure_UnitTest, Test_compressable_vs_incompressable) {
    avionics_sim::Differential_pressure_sensor_model dp(0.0);
    std::array<double, 6> indicated_airspeed = {1.0, 10.0, 20.0, 30.0, 40.0, 50.0};
    double comp_flow, incomp_flow;

    for (size_t i = 0; i < indicated_airspeed.size(); i++) {
        comp_flow = dp.diff_press_from_indicated_airspeed_compressable_flow(
                        indicated_airspeed[i]);
        incomp_flow = dp.diff_press_from_indicated_airspeed(indicated_airspeed[i]);
        EXPECT_NEAR((comp_flow - incomp_flow) / incomp_flow, 0, 0.01);
    }
}

// Test conversion from true airspeed to differential press
TEST(Differential_pressure_UnitTest, Test_diff_press_from_true_airspeed) {
    avionics_sim::Differential_pressure_sensor_model dp(0.0);
    std::array<double, 3> true_airspeed = {0.0, 25.0, 50.0};
    std::array<double, 2> static_pressure = {80000, 105000};
    std::array<double, 2> temperature = {-10.0, 40.0};
    std::array<std::array<std::array<double, 3>, 2>, 2> expected =  {{
            {   {
                    {0.0,  330.905085633934448, 1323.620342535737791},
                    {0.0,  434.312924894538980, 1737.251699578155922}
                }
            },
            {   {   {0.0, 278.070168560018658, 1112.280674240074632},
                    {0.0, 364.967096235024428, 1459.868384940097712}
                }
            }
        }
    };

    double calculated_diff_press;

    for (size_t i = 0; i < true_airspeed.size(); i++) {
        for (size_t j = 0; j < static_pressure.size(); j++) {
            for (size_t k = 0; k < temperature.size(); k++) {
                calculated_diff_press = dp.diff_press_from_true_airspeed(true_airspeed[i],
                                        static_pressure[j], temperature[k]);
                ASSERT_DOUBLE_EQ(expected[k][j][i], calculated_diff_press);
            }
        }
    }
}
#if(0)
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif
