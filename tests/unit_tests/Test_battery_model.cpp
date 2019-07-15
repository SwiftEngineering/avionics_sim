// test_diff_press.cpp
#include "avionics_sim/Battery_model.hpp"

#include <gtest/gtest.h>

namespace
{
    static const double epislon = 1e-13;

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
}

// Test constant current discharge along curve
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

    EXPECT_DOUBLE_EQ(1.150000000000003, soc);		// State of charge (Ah)
    EXPECT_NEAR(44.614096296052345, voltage, epislon);

    for (uint16_t i = 0; i < 150; i++)
    {
        bm.update_soc(0.46, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_DOUBLE_EQ(2.3000000000000096, soc);		// State of charge (Ah)
    EXPECT_NEAR(38.638385309543395,voltage, epislon);

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

    EXPECT_DOUBLE_EQ(1.0, soc);				// State of charge (Ah)
    EXPECT_NEAR(40.997647058823617,voltage, epislon);

    for (uint16_t i = 0; i < 3; i++)
    {
            bm.update_soc(15.0, time_step, &voltage, &soc);

            current_time += time_step/3600.0;

    }

    EXPECT_DOUBLE_EQ(1.75, soc);				// State of charge (Ah)
    EXPECT_NEAR(38.244979248047414,voltage, epislon);

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

    EXPECT_DOUBLE_EQ(0.97222222222222099, soc);		// State of charge (Ah)
    EXPECT_NEAR(37.330874395361604,voltage, epislon);

    for (uint16_t i = 0; i < 80; i++)
    {
        bm.update_soc(35.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_DOUBLE_EQ(1.749999999999996, soc);		// State of charge (Ah)
    EXPECT_NEAR(36.08254893909762,voltage, epislon);

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

    EXPECT_DOUBLE_EQ(1.0266666666666668, soc);		// State of charge (Ah)
    EXPECT_NEAR(43.352704133001275,voltage, epislon);

    for (uint16_t i=0; i<7; i++)
    {
        bm.update_soc(2.6 + (15.0 - 2.6) / 2.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_DOUBLE_EQ(2.0533333333333337, soc);		// State of charge (Ah)
    EXPECT_NEAR(39.134378092264775,voltage, epislon);

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

    EXPECT_DOUBLE_EQ(1.0416666666666641, soc);		// State of charge (Ah)
    EXPECT_NEAR(39.070800747423945,voltage, epislon);

    for (uint16_t i = 0; i < 150; i++)
    {
        bm.update_soc(15.0 + (35.0 - 15.0) / 2.0, time_step, &voltage, &soc);

        current_time += time_step/3600.0;

    }

    EXPECT_DOUBLE_EQ(2.0833333333333295, soc);		// State of charge (Ah)
    EXPECT_NEAR(35.039741323038932,voltage, epislon);

}
