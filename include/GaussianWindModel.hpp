/**
 * @brief       GaussianWindModel
 * @file        GaussianWindModel.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <random>
#include "IWindModel.hpp"
#include "Airfoil.hpp"

namespace avionics_sim {

class GaussianWindModel : public IWindModel {
  public:
    GaussianWindModel();

    GaussianWindModel(std::default_random_engine &strength_gen, std::default_random_engine &dir_gen);

    GaussianWindModel(
        double strength_m_per_s, double strength_variance,
        v3 direction, v3 dir_variance);

    GaussianWindModel(
        std::default_random_engine &strength_gen, std::default_random_engine &dir_gen,
        double strength_m_per_s, double strength_variance,
        v3 direction, v3 dir_variance);

    virtual ~GaussianWindModel();

    virtual WindRate get_rates();

    void set_strength_m_per_s(double strength_m_per_s, double strength_variance);

    void set_direction(v3 direction, double variance);

    void set_direction(v3 direction, v3 variance);

  protected:
    double generate_random_wind_strength();

    v3 generate_random_wind_direction();

    double strength_max_;

    std::default_random_engine strength_generator_;
    std::normal_distribution<double> strength_distribution_;

    std::default_random_engine direction_generator_;
    std::normal_distribution<double> direction_distribution_X_;
    std::normal_distribution<double> direction_distribution_Y_;
    std::normal_distribution<double> direction_distribution_Z_;
};
}  // namespace avionics_sim
