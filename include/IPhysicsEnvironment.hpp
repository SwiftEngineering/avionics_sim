/**
 * @brief       PhysicsEnvironment class. This class holds information about the physical environment outside the aircraft.
 * Its initial purpose is to provide an interface by which dynamic air pressure can be received.
 * @file        PhysicsEnvironment.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

namespace avionics_sim {

class IPhysicsEnvironment {
  public:
    virtual ~IPhysicsEnvironment() {}

    virtual double get_air_density_kg_per_m3() = 0;
};

}  // namespace avionics_sim
