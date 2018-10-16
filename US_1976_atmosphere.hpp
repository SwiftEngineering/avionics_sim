#pragma once

//US STandard Atmosphere 1976
//NASA-TM-X-74335
//https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf

//
//NASA TR R-459
//

#include <cstddef>

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

	class Geopotential_height
	{
	public:
		
		static Geopotential_height from_geometric_height(const double g_m);

	protected:

		double m_geopotential_height;

	}

	static ATMOSPHERE_LAYER get_layer(const double geopot_height);

	//NASA TR R-459 eq 18
	static double get_geopot_height(const double geometric_height);

	//NASA TR R-459 eq 19
	static double get_geometric_height(const double geopot_height);

	//NASA-TM-X-74335 eq 24
	static double get_temperature(const double geopot_height);

	//NASA-TM-X-74335 eq 33a / 33b
	static double get_pressure(const double geopot_height);

	//NASA-TM-X-74335 eq 42
	static double get_mass_density(const double geopot_height);

protected:

	constexpr static double base_geopot_height_km[] = {0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.8520};
	constexpr static double base_temp[] = {288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946};
	constexpr static double base_pressure[] = {101325.0, 22632.06, 5474.889, 868.0187, 110.9063, 66.93887, 3.956720, 0.3733836};

	
	constexpr static double GMR = 34.163195;//g_o' * M_0 / R* in g K/s/s
	constexpr static double R_star = 8.31432e3;//m/(kmol K) - NASA TR R-459 pg 4 - nonstandard definition for US atmosphere
	constexpr static double M_0 = 28.9644;//kg/kmol - NASA TR R-459 pg 17 - 
	constexpr static double g_0 = 9.80665;//m/s/s - NASA TR R-459 pg 4 - 1901 value for "45 degrees" / 45deg 32min 33s

	//degC/km
	constexpr static double lapse_rate[] = {-6.5, 0, 1.0, 2.8, 0, -2.8, -2.0, 0};
};
