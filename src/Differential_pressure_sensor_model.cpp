/**
 * @brief       Differential_pressure_sensor_model
 * @file        Differential_pressure_sensor_model.cpp
 * @author      Richard Kirby <rkirby@kspresearch.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc. All rights reserved.
 * @license     Internal Use Only.
 */

#include "avionics_sim/Differential_pressure_sensor_model.hpp"
#include "avionics_sim/US_1976_atmosphere.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <random>
#include <stdexcept>


// From equation 3.10 in https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19800015804.pdf
double Differential_pressure_sensor_model::diff_press_from_indicated_airspeed(const double indicated_airspeed)
{
	return indicated_airspeed*indicated_airspeed*US_1976_atmosphere::CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C*0.5;
}

// From equation 3.12 in https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19800015804.pdf
double Differential_pressure_sensor_model::diff_press_from_indicated_airspeed_compressable_flow(const double indicated_airspeed)
{
	return (pow((indicated_airspeed/CONSTANTS_SPEED_OF_SOUND_AT_SEA_LEVEL)*(indicated_airspeed/CONSTANTS_SPEED_OF_SOUND_AT_SEA_LEVEL)/5.0 + 1.0, 7.0/2.0) - 1.0) * US_1976_atmosphere::CONSTANTS_STATIC_AIR_PRESSURE_AT_SEA_LEVEL;
}

double Differential_pressure_sensor_model::indicated_airspeed_from_corrected_airspeed(const double corrected_airspeed)
{
	return corrected_airspeed + m_correction;
}

double Differential_pressure_sensor_model::corrected_airspeed_from_true_airspeed(const double true_airspeed, const double static_pressure, const double temperature_celsius)
{
	return true_airspeed / sqrt((US_1976_atmosphere::CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C/get_air_density(static_pressure, temperature_celsius)));
}

double Differential_pressure_sensor_model::diff_press_from_true_airspeed(const double true_airspeed, const double static_pressure, const double temperature_celsius)
{
	double corrected_airspeed = corrected_airspeed_from_true_airspeed(true_airspeed, static_pressure, temperature_celsius);
	double indicated_airspeed = indicated_airspeed_from_corrected_airspeed(corrected_airspeed);

	return diff_press_from_indicated_airspeed(indicated_airspeed);
}

double Differential_pressure_sensor_model::get_air_density(const double static_pressure, const double temperature_celsius)
{
	return static_pressure / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius - CONSTANTS_ABSOLUTE_NULL_CELSIUS));
}
