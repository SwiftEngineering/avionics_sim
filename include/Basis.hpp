/**
 * @brief       Basis class. This class holds air foil direction vectors.
 * @file        Basis.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <ignition/math.hh>

namespace avionics_sim {

class Basis {
  public:
    ///
    /// \brief      Constructor
    /// \details    Creates instance of lookup table.
    /// \param[in]  forward (forward vector)
    /// \param[in]  upward (upward vector)
    /// \return     Instance of Basis
    ///
    Basis();

    ///
    /// \brief      Calculates side vector as cross product
    /// \details    Side vector is cross product of up and forward vectors.
    /// \return     N/A
    ///
    void calculateSideVector();

    ignition::math::Vector3d up;
    ignition::math::Vector3d forward;
    ignition::math::Vector3d side;
};

}  // namespace avionics_sim
