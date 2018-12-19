/**
 * @brief       Exponential_smoothing_filter
 * @file        Exponential_smoothing_filter.cpp
 * @author      Jacob Schloss <jschloss@swiftengineering.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc. All rights reserved.
 * @license     Internal Use Only.
 */

#include "avionics_sim/Exponential_smoothing_filter.hpp"

#include <cmath>
#include <limits>

Exponential_smoothing_filter::Exponential_smoothing_filter(const double f_3db, const double dT)
{
	m_dT = dT;
	m_f_3db = f_3db;

	m_alpha = calculate_alpha(f_3db, m_dT);
	m_onelessalpha = 1.0 - m_alpha;

	m_last_output = std::numeric_limits<double>::quiet_NaN();
}

void Exponential_smoothing_filter::reset()
{
	m_last_output = std::numeric_limits<double>::quiet_NaN();	
}

void Exponential_smoothing_filter::set_last_output(const double y_last)
{
	m_last_output = y_last;
}

void Exponential_smoothing_filter::set_f_3db(const double f_3db, const double dT)
{
	m_dT = dT;
	m_f_3db = f_3db;

	m_alpha = calculate_alpha(f_3db, dT);
	m_onelessalpha = 1.0 - m_alpha;
}

double Exponential_smoothing_filter::calculate_alpha(const double f_3db, const double dT)
{
	//return dT / (1.0 / (2.0 * M_PI * f_3db) + dT);

	const double tau = get_tau(f_3db);
	
	return 1.0 - exp(-dT / tau);
}

double Exponential_smoothing_filter::next_y_n(const double x_n)
{
	//first input short circuits
	if( !std::isfinite(m_last_output) )
	{
		m_last_output = x_n;
		return x_n;
	}

	// a / (s + a)

	// y[n] = (1-alpha)*y[n-1] + alpha * x[n]
	const double y_n = m_onelessalpha * m_last_output + m_alpha * x_n;

	//store y[n] as next iteration's y[n-1]
	m_last_output = y_n;

	return y_n;
}

double Exponential_smoothing_filter::get_tau(const double f_3db)
{
	return 1.0 / (2.0 * M_PI * f_3db);
}
double Exponential_smoothing_filter::get_gain(const double f) const
{
	const double omega_tau = 2.0 * M_PI * f * get_tau(m_f_3db);

	return 1.0 / sqrt(1.0 + omega_tau * omega_tau);
}

double Exponential_smoothing_filter::get_phase(const double f) const
{
	const double omega_tau = 2.0 * M_PI * f * get_tau(m_f_3db);

	return -atan(omega_tau);
}