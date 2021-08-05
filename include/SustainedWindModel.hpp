/**
 * @brief       SustainedWindModel
 * @file        SustainedWindModel.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <random>
#include "IWindModel.hpp"

namespace avionics_sim {

class SustainedWindModel : public IWindModel {
  public:
    SustainedWindModel();

    explicit SustainedWindModel(v3 linear_rate);

    SustainedWindModel(double strength, v3 direction);

    virtual ~SustainedWindModel();

    virtual WindRate get_rates();

    void set_strength(double strength);

    void set_direction(v3 direction);

    void set_linear_rate(double strength, v3 direction);


  protected:
    v3 linear_rate_;

  private:
    // keep seperate from the aggregate vector for independent setting
    // but prevent having to multiply every time wind rates is called
    double  _strength;
    v3      _direction;
};
}  // namespace avionics_sim
