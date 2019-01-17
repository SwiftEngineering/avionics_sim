/**
 * @brief       US_1976_atmosphere
 * @file        US_1976_atmosphere.hpp
 * @author      Jacob Schloss <jschloss@swiftengineering.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

//References

//US Standard Atmosphere 1976
//NASA-TM-X-74335
//https://ntrs.nasa.gov/search.jsp?R=19770009539
//https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf

//Defining constants, equations, and abbreviated tables of the 1975 US Standard Atmosphere
//NASA TR R-459
//https://ntrs.nasa.gov/search.jsp?R=19760017709
//https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19760017709.pdf

#include <cstddef>

namespace avionics_sim
{

//stores geopotential and geometric height and converts between them
//Geometric height is linear meters above MSL
class Geopotential_height
{
public:

	constexpr static Geopotential_height from_geomet_height(const double g_m)
	{
		return Geopotential_height(g_m, get_geopot_height(g_m));
	}

	constexpr static Geopotential_height from_geopot_height(const double g_p)
	{
		return Geopotential_height(get_geomet_height(g_p), g_p);
	}

	//NASA TR R-459 eq 18
	constexpr static double get_geopot_height(const double geomet_height)
	{
		return (r0 * geomet_height) / (r0 + geomet_height);
	}
	double get_geopot_height() const
	{
		return m_geopotential_height;
	}

	//NASA TR R-459 eq 19
	constexpr static double get_geomet_height(const double geopot_height)
	{
		return (r0 * geopot_height) / (r0 - geopot_height);
	}
	double get_geomet_height() const
	{
		return m_geometric_height;
	}

protected:

	constexpr Geopotential_height(const double geomet_height, const double geopot_height) : m_geometric_height(geomet_height), m_geopotential_height(geopot_height)
	{

	}

	double m_geometric_height;
	double m_geopotential_height;

	//m, value from NASA TR R-459 pg 15
	constexpr static double r0 = 6356766.0;
	
	//m/s/s, value from NASA TR R-459 pg 15
	constexpr static double g0 = 9.80665;
};

///
/// Implementation of the US 1976 atmosphere
/// Performance above MESOSPHERE2 is degraded, about 5 ULP of error (5/10000 Pa for pressure, 5/10000 kg/m^3 for density)

class US_1976_atmosphere
{
public:

	enum class ATMOSPHERE_LAYER : size_t
	{
		TROPOSPHERE = 0,
		TROPOPAUSE = 1,
		STRATOSPHERE1 = 2,
		STRATOSPHERE2 = 3,
		STRATOPAUSE = 4,
		MESOSPHERE1 = 5,
		MESOSPHERE2 = 6,
		MESOPAUSE = 7
	};

	static ATMOSPHERE_LAYER get_layer(const double geopot_height);

	//NASA-TM-X-74335 eq 24
	//Kelvin
	static double get_temperature(const double geopot_height);

	//NASA-TM-X-74335 eq 33a / 33b
	//Pascal
	static double get_pressure(const double geopot_height);

	//NASA-TM-X-74335 eq 42
	//kg/m^3
	static double get_mass_density(const double geopot_height);

	static constexpr double CONSTANTS_STATIC_AIR_PRESSURE_AT_SEA_LEVEL = 101325;	// Pa
	static constexpr double CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C = 1.225;			// kg/m^3

protected:

	constexpr static double base_geopot_height_km[] = {0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.8520};//in km
	constexpr static double base_temp[] = {288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946};//in Kelvin
	constexpr static double base_pressure[] = {101325.0, 22632.06, 5474.889, 868.0187, 110.9063, 66.93887, 3.956720, 0.3733836};//in Pa

	constexpr static double R_star = 8.31432e3;//m/(kmol K) - NASA TR R-459 pg 4 - nonstandard definition for US atmosphere 1976 used by convention
	constexpr static double M_0 = 28.9644;//kg/kmol - NASA TR R-459 pg 17 - 
	constexpr static double g_0 = 9.80665;//m/s/s - NASA TR R-459 pg 4 - 1901 value for "45 degrees" / but really closer to 45deg 32min 33s
	constexpr static double GMR = g_0 * M_0 / R_star * 1000.0;//g_o' * M_0 / R* in g K/s/s, ~34.163195
	constexpr static double MR = M_0 / R_star;//M_0 / R* in kg K/m, ~0.0034837

	//degC/km
	constexpr static double lapse_rate[] = {-6.5, 0, 1.0, 2.8, 0, -2.8, -2.0, 0};
};

}