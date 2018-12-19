/**
 * @brief       NPolynomialFit class
 * @file        NPolynomial_Fit.hpp
 * @author      Nuno Marques <nuno.marques@dronesolutions.io>
 * @copyright   Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 * @license     Internal Use Only.
 */

#pragma once

#include <boost/array.hpp>
#include <vector>
#include <boost/math/tools/rational.hpp>

namespace avionics_sim
{

/// \brief NPolynomialFit class that implements a N-degree polynomial fit
template <class T, size_t n>
class NPolynomialFit {
public:
	/// \brief Gets the output from an input value and the poly indeterminates kept on the m_fit_curves set
	T get_output_from_poly(const size_t &poly_index, const double &input) {
		BOOST_ASSERT_MSG(poly_index <= (m_fit_curves.size() - 1), "The chosen polynomial curve does not exist!");
		return boost::math::tools::evaluate_polynomial(m_fit_curves[poly_index], input);
	}

	/// \brief Gets the output from an input value and input poly indeterminates
	T get_output_from_poly(const boost::array<T, n> &poly, const double &input) {
		return boost::math::tools::evaluate_polynomial(poly, input);
	}

	/// \brief Loads a polynomial fit curve to be used
	void load_poly_fit_curve(const boost::array<T, n> &poly) {
		m_fit_curves.push_back(poly);
	}

protected:
	/// \brief Array of polynomial fit curves
	std::vector<boost::array<T, n> > m_fit_curves;
};	// class NPolynomialFit

}