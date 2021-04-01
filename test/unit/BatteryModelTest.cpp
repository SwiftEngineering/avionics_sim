#include <gtest/gtest.h>

#include <string>
#include <sstream>
#include <boost/array.hpp>
#include <stdio.h>

#include "Battery_model.hpp"

namespace
{
    static const double epsilon = 1e-6;

    void load_discharge_curves(avionics_sim::Battery_model *bm)
    {
        avionics_sim::Battery_model::discharge_curve dc;
        dc.load = 0.46;
        dc.c[0] = 4.20180995475114;
        dc.c[1] = -0.000643100918689175;
        dc.c[2] = 6.26071232688909e-07;
        dc.c[3] = -7.14597902097917e-10;
        dc.c[4] = 3.78145139174554e-13;
        dc.c[5] = -7.3058069381599e-17;
        bm->add_discharge_curves(dc);

        dc.load = 15;
        dc.c[0] = 3.84579185520363;
        dc.c[1] = -0.00128169683257922;
        dc.c[2] = 2.30634898190049e-06;
        dc.c[3] = -2.35276442307693e-09;
        dc.c[4] = 1.08349830316742e-12;
        dc.c[5] = -1.84707296380089e-16;
        bm->add_discharge_curves(dc);

        dc.load = 35;
        dc.c[0] = 3.48964932126698;
        dc.c[1] = -0.00192302164747022;
        dc.c[2] = 3.73104882250109e-06;
        dc.c[3] = -3.45033872377626e-09;
        dc.c[4] = 1.53237158062526e-12;
        dc.c[5] = -2.69548925339366e-16;
        bm->add_discharge_curves(dc);
    }

    void load_internal_resistance_curves(avionics_sim::Battery_model *bm)
    {
        avionics_sim::Battery_model::internal_resistance_curve ir;
        ir.soh = 1.0;
        ir.harness_resistance = 0.040;
        ir.c[0] = 0.521273828846724;
        ir.c[1] = -16.5195364175862;
        ir.c[2] = 225.962184259985;
        ir.c[3] = -1632.78483786197;
        ir.c[4] = 6938.58530566924;
        ir.c[5] = -18370.5497580968;
        ir.c[6] = 31118.4418493407;
        ir.c[7] = -33702.3045169079;
        ir.c[8] = 22555.2090615421;
        ir.c[9] = -8490.74044658212;
        ir.c[10] = 1374.20478941936;
        bm->set_internal_resistance_coefficients(ir);
    }
}

TEST(Battery_model_UnitTest, Test_constant_current_discharge_no_interpolation_low)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    double voltage, soc;

    load_discharge_curves(&bm);

    double current_time = 0;
    double time_step = 60;	// 1 minute time steps

    for (uint16_t i = 0; i < 150; i++)
    {
        bm.update_soc(0.46, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.15, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(44.614096296052345, voltage, epsilon);

    for (uint16_t i = 0; i < 150; i++)
    {
        bm.update_soc(0.46, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(2.3, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(38.638385309543395,voltage, epsilon);

}

// Test constant current discharge along curve
TEST(Battery_model_UnitTest, Test_constant_current_discharge_no_interpolation_mid)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    double voltage, soc;

    load_discharge_curves(&bm);

    double current_time = 0;
    double time_step = 60;	// 60 second time steps

    for (uint16_t i = 0; i < 4; i++)
    {
        bm.update_soc(15.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.0, soc, epsilon);				// State of charge (Ah)
    EXPECT_NEAR(40.997647058823617,voltage, epsilon);

    for (uint16_t i = 0; i < 3; i++)
    {
            bm.update_soc(15.0, time_step, &voltage, &soc);

            current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.75, soc, epsilon);				// State of charge (Ah)
    EXPECT_NEAR(38.244979248047414,voltage, epsilon);

}

// Test constant current discharge along curve
TEST(Battery_model_UnitTest, Test_constant_current_discharge_no_interpolation_high)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    double voltage, soc;

    load_discharge_curves(&bm);

    double current_time = 0;
    double time_step = 1;	// 1 second time steps

    for (uint16_t i = 0; i < 100; i++)
    {
        bm.update_soc(35.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(0.97222222222222099, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(37.330874395361604,voltage, epsilon);

    for (uint16_t i = 0; i < 80; i++)
    {
        bm.update_soc(35.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.75, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(36.08254893909762,voltage, epsilon);

}


// Test interpolated rundown
TEST(Battery_model_UnitTest, Test_interpolated_constant_current_discharge_low)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    double voltage, soc;

    load_discharge_curves(&bm);

    double current_time = 0;
    double time_step = 60;	// 60 second time steps

    for (uint16_t i=0; i<7; i++)
    {
        bm.update_soc(2.6 + (15.0 - 2.6) / 2.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.0266666666666667, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(43.352704133001275,voltage, epsilon);

    for (uint16_t i=0; i<7; i++)
    {
        bm.update_soc(2.6 + (15.0 - 2.6) / 2.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(2.0533333333333333, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(39.134378092264775,voltage, epsilon);

}

// Test interpolated rundown
TEST(Battery_model_UnitTest, Test_interpolated_constant_current_discharge_high)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    double voltage, soc;

    load_discharge_curves(&bm);

    double current_time = 0;
    double time_step = 1;	// 60 second time steps

    for (uint16_t i = 0; i < 150; i++)
    {
        bm.update_soc(15.0 + (35.0 - 15.0) / 2.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.0416666666666667, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(39.070800747423945,voltage, epsilon);

    for (uint16_t i = 0; i < 150; i++)
    {
        bm.update_soc(15.0 + (35.0 - 15.0) / 2.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(2.0833333333333333, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(35.039741323038932,voltage, epsilon);

}

// Test OCV with low power consumption
TEST(Battery_model_UnitTest, Test_constant_current_discharge_ocv_low)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    double voltage, soc;

    load_discharge_curves(&bm);
    load_internal_resistance_curves(&bm);

    double current_time = 0;
    double time_step = 60;	// 1 minute time steps

    for (uint16_t i = 0; i < 150; i++)
    {
        bm.update_soc_ocv(0.46, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.15, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(50.385815160448387928, voltage, epsilon);

    for (uint16_t i = 0; i < 150; i++)
    {
        bm.update_soc_ocv(0.46, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(2.3, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(50.378431664725596306,voltage, epsilon);

}

// Test OCV with medium power consumption
TEST(Battery_model_UnitTest, Test_constant_current_discharge_ocv_mid)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    double voltage, soc;

    load_discharge_curves(&bm);
    load_internal_resistance_curves(&bm);

    double current_time = 0;
    double time_step = 60;	// 60 second time steps

    for (uint16_t i = 0; i < 4; i++)
    {
        bm.update_soc_ocv(15.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.0, soc, epsilon);				// State of charge (Ah)
    EXPECT_NEAR(49.465729079548033553,voltage, epsilon);

    for (uint16_t i = 0; i < 3; i++)
    {
            bm.update_soc_ocv(15.0, time_step, &voltage, &soc);

            current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.75, soc, epsilon);				// State of charge (Ah)
    EXPECT_NEAR(49.384206235564455767,voltage, epsilon);

}

// Test OCV with high power consumption
TEST(Battery_model_UnitTest, Test_constant_current_discharge_ocv_high)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    double voltage, soc;

    load_discharge_curves(&bm);
    load_internal_resistance_curves(&bm);

    double current_time = 0;
    double time_step = 1;	// 1 second time steps

    for (uint16_t i = 0; i < 100; i++)
    {
        bm.update_soc_ocv(35.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(0.97222222222222222, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(48.170382531814574634, voltage, epsilon);

    for (uint16_t i = 0; i < 80; i++)
    {
        bm.update_soc_ocv(35.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_NEAR(1.75, soc, epsilon);		// State of charge (Ah)
    EXPECT_NEAR(48.018831483085087086, voltage, epsilon);

}

// Test internal resistance
TEST(Battery_model_UnitTest, Test_internal_resistance)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    std::vector<double> soc = {1, 0.5, 0.2, 0.08, 0.05, 0.03};
    std::vector<double> ir = {0.065368, 0.067053, 0.070461, 0.041378, 0.09419, 0.19016};
    double epsilon = 0.0001;

    load_internal_resistance_curves(&bm);

    for (auto i : soc)
    {
        EXPECT_NEAR(ir[i], bm.compute_internal_resistance(soc[i], 1.0), epsilon);
    }
}

// Test max power available
TEST(Battery_model_UnitTest, Test_max_power_available)
{
    avionics_sim::Battery_model bm;
    bm.initialize(0.0, 3.4, 12, 0.0, 0.050);

    std::vector<double> volts = {50.0, 48.0, 46.0, 44.0, 42.0, 40.0, 38.0};
    std::vector<double> ohms = {0.020, 0.040, 0.060, 0.080, 0.100, 0.120, 0.140, 0.160};

    for (auto i : volts)
    {
        for (auto j : ohms)
        {
            EXPECT_NEAR(volts[i]*volts[i]/(4*ohms[j]), bm.compute_max_power_available(volts[i], ohms[j]), epsilon);
        }
    }
}
