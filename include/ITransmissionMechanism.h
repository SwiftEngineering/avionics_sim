/**
 * @brief       Transmission Mechanism Interface
 * @file        ITransmissionMechanism.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2021, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace avionics_sim {

class ITransmissionMechanism {
  public:
    virtual double transmit(double input) = 0;
    virtual ~ITransmissionMechanism() {}
};

}  // namespace avionics_sim
