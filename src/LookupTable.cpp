/**
 * @brief       LookupTable class. This class holds LUT information used in determining coefficients of lift and drag.
 * @file        LookupTable.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "LookupTable.hpp"
#include <assert.h>

namespace avionics_sim {

LookupTable::LookupTable::LookupTable(
    std::vector<std::string> colNames,
    std::vector<std::vector<double>> cols) {

    // Number of column names must match the number of columns
    assert(colNames.size() == cols.size());

    // Populate LUT Map
    for (int i = 0; i < colNames.size(); i++) {
        lutMap.insert(std::make_pair(colNames[i], cols.at(i)));
    }
}

double LookupTable::lookup(double valA, std::string fromA, std::string inB) {
    double valB;
    avionics_sim::Bilinear_interp::interpolate(lutMap.at(fromA), lutMap.at(inB),
            valA, &valB);

    return valB;
}

}  // namespace avionics_sim
