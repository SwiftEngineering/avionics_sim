/**
 * @brief       PolyFit3rdDegree class
 * @file        Poly_fit_3rd_degree.cpp
 * @author      Nuno Marques <nuno.marques@dronesolutions.io>
 * @copyright   Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 */

#ifndef POLY_FIT_3RD_DEGREE_HPP
#define POLY_FIT_3RD_DEGREE_HPP

#include <array>

/// \brief PolyFit3rdDegree class that implements a 3rd degree polynomial fit
class PolyFit3rdDegree {
public:
	PolyFit3rdDegree() {};
	virtual ~PolyFit3rdDegree() = default;

	/// \brief Gets the output from an input value and the poly indeterminates
	double get_output_from_poly(const std::array<double, 3> &poly, const double &input);
};	// class PolyFit3rdDegree

#endif	// POLY_FIT_3RD_DEGREE
