/**
 * @brief       Implementation for PolyFit3rdDegree class functions
 * @file        Poly_fit_3rd_degree.cpp
 * @author      Nuno Marques <nuno.marques@dronesolutions.io>
 * @copyright   Copyright(C) 2018, Swift Engineering Inc. All rights reserved.
 */

#include "avionics_sim/Poly_fit_3rd_degree.hpp"

#include <cmath>

double PolyFit3rdDegree::get_output_from_poly(const std::array<double, 3> &poly, const double &input) {
	return poly[0] * std::pow(input, 3) + poly[1] * std::pow(input, 2) + poly[2] * input;
}
