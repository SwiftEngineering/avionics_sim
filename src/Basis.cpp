/**
 * @brief       Basis class. This class holds air foil direction vectors.
 * @file        Basis.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "Basis.hpp"


namespace avionics_sim {

Basis::Basis() :
    up(ignition::math::Vector3d(0.0, 0.0, 0.0)),
    forward(ignition::math::Vector3d(0.0, 0.0, 0.0)),
    side(ignition::math::Vector3d(0.0, 0.0, 0.0))
   {}

  void Basis::calculateSideVector()
  {
    side=forward.Cross(up);
  }

}
