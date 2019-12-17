//___________________________________________________________________
//-------------------------------------------------------------------
/*!
 \file		Test_US_1976_atmosphere.cpp

 \brief		Unit tests source file for ISA atmopsphere implementation

 \copyright  Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 */
//___________________________________________________________________
//-------------------------------------------------------------------

#include "avionics_sim/US_1976_atmosphere.hpp"

// gtest
#include "gtest/gtest.h"

TEST(US_1976_atmosphere_UnitTest, get_layer)
{
	using namespace avionics_sim;

	//layer 0
	EXPECT_EQ(US_1976_atmosphere::get_layer(-10000.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::TROPOSPHERE);
	EXPECT_EQ(US_1976_atmosphere::get_layer(-5000.0  / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::TROPOSPHERE);
	EXPECT_EQ(US_1976_atmosphere::get_layer(0.0      / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::TROPOSPHERE);
	EXPECT_EQ(US_1976_atmosphere::get_layer(5000.0   / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::TROPOSPHERE);
	EXPECT_EQ(US_1976_atmosphere::get_layer(10000.0  / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::TROPOSPHERE);
	EXPECT_EQ(US_1976_atmosphere::get_layer(10999.0  / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::TROPOSPHERE);

	//layer 1
	EXPECT_EQ(US_1976_atmosphere::get_layer(11000.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::TROPOPAUSE);

	//layer 2
	EXPECT_EQ(US_1976_atmosphere::get_layer(20000.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::STRATOSPHERE1);

	//layer 3
	EXPECT_EQ(US_1976_atmosphere::get_layer(32000.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::STRATOSPHERE2);

	//layer 4
	EXPECT_EQ(US_1976_atmosphere::get_layer(47000.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::STRATOPAUSE);

	//layer 5
	EXPECT_EQ(US_1976_atmosphere::get_layer(51000.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::MESOSPHERE1);

	//layer 6
	EXPECT_EQ(US_1976_atmosphere::get_layer(71000.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::MESOSPHERE2);

	//layer 6
	EXPECT_EQ(US_1976_atmosphere::get_layer(84851.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::MESOSPHERE2);

	//layer 7
	EXPECT_EQ(US_1976_atmosphere::get_layer(84852.0 / 1000.0), US_1976_atmosphere::ATMOSPHERE_LAYER::MESOPAUSE);
}

TEST(US_1976_atmosphere_UnitTest, get_pressure)
{
	using namespace avionics_sim;

	// test the common points in geo potential height, sig figs from NASA table
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(-5000.0), 	17768e1, 	10);
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(0.0), 		101325e0, 	1);
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(5000.0), 	54019, 		1);

	EXPECT_NEAR(US_1976_atmosphere::get_pressure(11000.0), 	22632e0, 	1);
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(15000.0), 	12044e0,	1);

	EXPECT_NEAR(US_1976_atmosphere::get_pressure(20000.0), 	5474.8, 	0.1);
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(25000.0), 	2511.0, 	0.1);

	EXPECT_NEAR(US_1976_atmosphere::get_pressure(32000.0), 	868.01, 	0.01);
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(40000.0), 	277.52, 	0.01);

	EXPECT_NEAR(US_1976_atmosphere::get_pressure(47000.0), 	110.90, 	0.01);
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(49000.0), 	86.162, 	0.01);

	EXPECT_NEAR(US_1976_atmosphere::get_pressure(51000.0), 	66.938, 	0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(61000.0), 	17.660, 	0.001);

	EXPECT_NEAR(US_1976_atmosphere::get_pressure(71000.0), 	3.9564, 	5.0*0.0001); //precision loss, should be 0.0001
	EXPECT_NEAR(US_1976_atmosphere::get_pressure(77000.0), 	1.4809, 	5.0*0.0001); //precision loss, should be 0.0001

	EXPECT_NEAR(US_1976_atmosphere::get_pressure(84500.0),  0.39814, 	5.0*0.00001);//precision loss, should be 0.00001
}

TEST(US_1976_atmosphere_UnitTest, get_temperature)
{
	using namespace avionics_sim;

	//test the common points in geo potential height, sig figs from NASA table
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(-5000.0), 	320.650, 0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(0.0), 		288.150, 0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(5000.0),	255.650, 0.001);
	
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(11000.0), 	216.650, 0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(15000.0), 	216.650, 0.001);

	EXPECT_NEAR(US_1976_atmosphere::get_temperature(20000.0), 	216.650, 0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(25000.0), 	221.650, 0.001);

	EXPECT_NEAR(US_1976_atmosphere::get_temperature(32000.0), 	228.650, 0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(40000.0), 	251.050, 0.001);

	EXPECT_NEAR(US_1976_atmosphere::get_temperature(47000.0), 	270.650, 0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(49000.0), 	270.650, 0.001);

	EXPECT_NEAR(US_1976_atmosphere::get_temperature(51000.0), 	270.650, 0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(61000.0), 	242.650, 0.001);

	EXPECT_NEAR(US_1976_atmosphere::get_temperature(71000.0), 	214.650, 0.001);
	EXPECT_NEAR(US_1976_atmosphere::get_temperature(77000.0), 	202.650, 0.001);

	EXPECT_NEAR(US_1976_atmosphere::get_temperature(84500.0),	187.650, 0.001);
}

TEST(US_1976_atmosphere_UnitTest, get_mass_density)
{
	using namespace avionics_sim;
	
	//test the common points in geo potential height, sig figs from NASA table
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(-5000.0),	1.9305, 	0.0001);
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(0.0),		1.2250, 	0.0001);
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(5000.0),	7.3612e-1, 	0.0001e-1);


	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(11000.0), 	3.6392e-1,	0.0001e-1);
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(15000.0), 	1.9367e-1,	0.0001e-1);

	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(20000.0), 	8.8035e-2,	0.0001e-2);
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(25000.0), 	3.9466e-2,	0.0001e-2);

	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(32000.0), 	1.3225e-2,	0.0001e-2);
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(40000.0), 	3.8510e-3,	0.0001e-2);

	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(47000.0), 	1.4275e-3,	0.0001e-3);
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(49000.0), 	1.1090e-3,	0.0001e-3);

	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(51000.0), 	8.6160e-4,	0.0001e-4);
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(61000.0), 	2.5355e-4,	0.0001e-4);

	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(71000.0), 	6.4211e-5,	5.0*0.0001e-5);//precision loss, should be 0.0001e-5
	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(77000.0), 	2.5458e-5,	5.0*0.0001e-5);//precision loss, should be 0.0001e-5

	EXPECT_NEAR(US_1976_atmosphere::get_mass_density(84500.0),	7.3914e-6,	6.0*0.0001e-6);//precision loss, should be 0.0001e-6
}