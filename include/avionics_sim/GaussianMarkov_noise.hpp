/**
 * @brief       GaussianMarkov_noise
 * @file        GaussianMarkov_noise.hpp
 * @author      Jacob Schloss <jschloss@swiftengineering.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc. All rights reserved.
 * @license     Internal Use Only.
 */

#pragma once

#include <random>
#include <sstream>

namespace avionics_sim
{

class GaussianMarkov_noise
{
public:

	///
	/// Init class
	/// Picks a random rng seed from a random device
	/// \param  [in] tau - GM time constant, in seconds
	/// \param  [in] sigma - GM std dev, if 0 GM will return 0
	/// \param  [in] initial_output - GM inital value
	GaussianMarkov_noise(const double tau, const double sigma, const double initial_output);
	
	///
	/// Init class
	/// \param  [in] tau - GM time constant, in seconds
	/// \param  [in] sigma - GM std dev, if 0 GM will return 0
	/// \param  [in] initial_output - GM inital value
	/// \param  [in] seed - RNG seed
	/// \param  [in] seed_len - Length of RNG seed
	///
	GaussianMarkov_noise(const double tau, const double sigma, const double initial_output, const uint32_t seed[], const size_t seed_len);

	///
	/// Reset distribution and RNG to initial state
	///
	void reset();

	///
	/// Time since reset
	/// \return return next value in sequence
	///
	double update(const double dT);

protected:

	void initialize(const double tau, const double sigma, const double initial_output, const uint32_t seed[], const size_t seed_len);

	double m_tau;
	double m_sigma;
	double m_initial_output;

	std::mt19937_64 m_rand_gen;
	std::normal_distribution<double> m_normal_dist;

	double m_last_output;

	std::stringstream m_init_rng_state;
};

}