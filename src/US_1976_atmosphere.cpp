/**
 * @brief       US_1976_atmosphere
 * @file        US_1976_atmosphere.cpp
 * @author      Jacob Schloss <jschloss@swiftengineering.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

//US Standard Atmosphere 1976
//NASA-TM-X-74335
//https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770009539.pdf

//Defined constats, equations, and abbreviated tables of the 1945 US Std Atmosphere
//NASA TR R-459
//https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19760017709.pdf

//http://www.pdas.com/atmosdownload.html
#include "US_1976_atmosphere.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>

namespace avionics_sim
{

constexpr double US_1976_atmosphere::base_geopot_height_km[];
constexpr double US_1976_atmosphere::base_temp[];
constexpr double US_1976_atmosphere::base_pressure[];
constexpr double US_1976_atmosphere::lapse_rate[];

US_1976_atmosphere::ATMOSPHERE_LAYER US_1976_atmosphere::get_layer(const double geopot_height_km)
{
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

	//we already throw above if you are higher than the mesopause
	return ATMOSPHERE_LAYER::MESOPAUSE;
}

double US_1976_atmosphere::get_temperature(const double geopot_height)
{
	const double geopot_height_km = geopot_height / 1000.0;

	const ATMOSPHERE_LAYER layer = get_layer(geopot_height_km);

	const double Lmb = lapse_rate[(size_t)layer];
	const double Tmb = base_temp[(size_t)layer];
	const double Hb = base_geopot_height_km[(size_t)layer];


	double Tm = Tmb + Lmb * (geopot_height_km - Hb);

	return Tm;
}

double US_1976_atmosphere::get_pressure(const double geopot_height)
{
	const double geopot_height_km = geopot_height / 1000.0;

	const ATMOSPHERE_LAYER layer = get_layer(geopot_height_km);

	const double Lmb = lapse_rate[(size_t)layer];
	const double Tmb = base_temp[(size_t)layer];
	const double Pb = base_pressure[(size_t)layer];
	const double Hb = base_geopot_height_km[(size_t)layer];

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

double US_1976_atmosphere::get_mass_density(const double geopot_height)
{
	const double P = get_pressure(geopot_height);
	const double Tm = get_temperature(geopot_height);

	double rho = P * MR / Tm;

	return rho;
}

}
