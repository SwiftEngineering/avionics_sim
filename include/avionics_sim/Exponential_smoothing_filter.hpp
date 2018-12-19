/**
 * @brief       Exponential_smoothing_filter
 * @file        Exponential_smoothing_filter.hpp
 * @author      Jacob Schloss <jschloss@swiftengineering.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc. All rights reserved.
 * @license     Internal Use Only.
 */

#pragma once

// implements a digital first order LPF
// X(s) = a / (s+a)
// as
// y[n] = (1-alpha)*y[n-1] + alpha * x[n]
// alpha = 1 - exp(-dT / tau)
// dT is the size of the time step
// tau is the time constant
class Exponential_smoothing_filter
{
	public:

	///
	/// Init filter
	///	Set cut off frequency based on sample period
	///
	Exponential_smoothing_filter(const double f_3db, const double dT);

	///
	/// Reset the internal state, ie clear y[n-1]
	///
	void reset();

	///
	///	Set cut off frequency based on sample period
	/// This does not reset the filter's state
	///
	void set_f_3db(const double f_3db, const double dT);

	///
	///	Force y[n-1] to be a certain value, to init the filter to a known state
	///
	void set_last_output(const double y_last);

	///
	/// Get next output as function of input and history
	///
	/// First x_n is returned unfiltered to init the filter
	///
	double next_y_n(const double x_n);

	/// Get gain at a certain frequency
	double get_gain(const double f) const;

	/// Get phase at a certain frequency
	double get_phase(const double f) const;


protected:

	static double get_tau(const double f_3db);
	static double calculate_alpha(const double f_3db, const double dT);

	double m_dT;
	double m_f_3db;

	double m_alpha;
	double m_onelessalpha;

	double m_last_output;

};
