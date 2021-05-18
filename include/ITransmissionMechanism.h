#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace avionics_sim {

class ITransmissionMechanism {
  public:
    virtual double transmit(double input) = 0;
    virtual ~ITransmissionMechanism() {};
};




}
