/**
 * @brief       PhysicsEnvironment class. This class holds air foil direction vectors.
 * @file        PhysicsEnvironment.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "PhysicsEnvironment.hpp"


namespace avionics_sim {

/*
For now, just return the constant of 1.225. Once a mechanism for reading dynamic air pressure in from a sensor has been developed, implement here.
*/

double PhysicsEnvironment::getAirDensity_kg_per_m3() {
    double airDensity;
    airDensity=1.22;
    return airDensity;
  }

}
