
//US Standard Atmosphere 1976
//NASA-TM-X-74335
//https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf

//Defined constats, equations, and abbreviated tables of the 1945 US Std Atmosphere
//NASA TR R-459
//https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19760017709.pdf

//http://www.pdas.com/atmosdownload.html
#include "US_1976_atmosphere.hpp"
#include <exception>
#include <stdexcept>
#include <cmath>
#include <iostream>

constexpr double US_1976_atmosphere::base_geopot_height_km[];
constexpr double US_1976_atmosphere::base_temp[];
constexpr double US_1976_atmosphere::base_pressure[];
constexpr double US_1976_atmosphere::lapse_rate[];

US_1976_atmosphere::ATMOSPHERE_LAYER US_1976_atmosphere::get_layer(const double geopot_height)
{
	const double geopot_height_km = geopot_height / 1000.0;

	if(geopot_height_km > base_geopot_height_km[(size_t)ATMOSPHERE_LAYER::MESOPAUSE])
	{
		throw std::domain_error("geopot_height must be below MESOPAUSE");
	}

	ATMOSPHERE_LAYER layer = ATMOSPHERE_LAYER::TROPOSPHERE;

	for(size_t i = 0; i < 7; i++)
	{
		if(geopot_height_km < base_geopot_height_km[i+1])
		{
			return static_cast<ATMOSPHERE_LAYER>(i);
			break;
		}
	}

	if(geopot_height_km == base_geopot_height_km[(size_t)ATMOSPHERE_LAYER::MESOPAUSE])
	{
		return ATMOSPHERE_LAYER::MESOPAUSE;
	}
}

double US_1976_atmosphere::get_geopot_height(const double geometric_height)
{
	//m
	const double r0 = 6356766.0;
	//m/s/s
	const double g0 = 9.80665;

	double h = (r0 * geometric_height) / (r0 + geometric_height);

	return h;
}

double US_1976_atmosphere::get_geometric_height(const double geopot_height)
{
	//m
	const double r0 = 6356766.0;
	//m/s/s
	const double g0 = 9.80665;

	double h = (r0 * geopot_height) / (r0 - geopot_height);

	return h;
}

double US_1976_atmosphere::get_temperature(const double geopot_height)
{
	const ATMOSPHERE_LAYER layer = get_layer(geopot_height);

	const double Lmb = lapse_rate[(size_t)layer];
	const double Tmb = base_temp[(size_t)layer];
	const double Hb = base_geopot_height_km[(size_t)layer];
	
	const double geopot_height_km = geopot_height / 1000.0;

	double Tm = Tmb + Lmb * (geopot_height_km - Hb);

	return Tm;
}

double US_1976_atmosphere::get_pressure(const double geopot_height)
{
	const ATMOSPHERE_LAYER layer = get_layer(geopot_height);

	const double Lmb = lapse_rate[(size_t)layer];
	const double Tmb = base_temp[(size_t)layer];
	const double Pb = base_pressure[(size_t)layer];
	const double Hb = base_geopot_height_km[(size_t)layer];
	
	const double geopot_height_km = geopot_height / 1000.0;

	double P = 0.0;

	if(Lmb == 0.0)
	{
		P = Pb * exp(-GMR * (geopot_height_km - Hb) / Tmb);
	}
	else
	{
		P = Pb * pow(Tmb / (Tmb + Lmb*(geopot_height_km - Hb)), GMR / Lmb);
	}

	return P;
}