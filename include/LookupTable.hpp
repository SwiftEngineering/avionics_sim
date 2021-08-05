/**
 * @brief       LookupTable class. This class holds LUT information used in determining coefficients of lift and drag.
 * @file        LookupTable.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <stdio.h>
#include <assert.h>

#include "Bilinear_interp.hpp"

namespace avionics_sim {

class LookupTable {
  public:
    ///
    /// \brief      Constructor
    /// \details    Creates instance of lookup table.
    /// \param[in]  colNames (std::vector containing the names of the LUTs)
    /// \param[in]  cols (std::vector of std::vector<double> containing LUT values)
    /// \return     Instance of lookup table
    ///
    LookupTable(std::vector<std::string> colNames,
                std::vector<std::vector<double>> cols);

    ///
    /// \brief      Lookup function
    /// \details    Retrieves lookup
    /// \param[in]  valA (value input to source table)
    /// \param[in]  fromA (Source table)
    /// \param[in]  inB (Destination table)
    /// \return     Interpolated value
    ///
    double lookup(double valA, std::string fromA, std::string inB);

  private:
    /// \brief Hash table containing LUTs.
    std::unordered_map<std::string, std::vector<double>> lutMap;
};

}  // namespace avionics_sim
