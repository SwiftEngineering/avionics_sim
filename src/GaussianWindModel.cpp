/**
 * @brief       GaussianWindModel
 * @file        GaussianWindModel.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "GaussianWindModel.hpp"
#include "Coordinate_Utils.hpp"

namespace avionics_sim {

GaussianWindModel::GaussianWindModel() {
}

GaussianWindModel::GaussianWindModel(
  std::default_random_engine& strength_gen,
  std::default_random_engine& dir_gen) :
  strength_generator_(strength_gen),
  direction_generator_(dir_gen) {
}

GaussianWindModel::GaussianWindModel(
  double strength_m_per_s, double strength_variance,
  v3 direction, v3 dir_variance) :
  strength_generator_(std::default_random_engine()),
  direction_generator_(std::default_random_engine()) {
    set_strength_m_per_s(strength_m_per_s, strength_variance);
    set_direction(direction, dir_variance);
}

GaussianWindModel::GaussianWindModel(
  std::default_random_engine& strength_gen, std::default_random_engine& dir_gen,
  double strength_m_per_s, double strength_variance,
  v3 direction, v3 dir_variance) :
  strength_generator_(strength_gen),
  direction_generator_(dir_gen) {
    set_strength_m_per_s(strength_m_per_s, strength_variance);
    set_direction(direction, dir_variance);
}

GaussianWindModel::~GaussianWindModel() {
}

WindRate GaussianWindModel::get_rates() {

    double strength_m_per_s = generate_random_wind_strength();

    v3 direction = generate_random_wind_direction();

    v3 linear_rate_m_per_s = strength_m_per_s * direction;

    return {
      linear_rate_m_per_s,
      v3(0, 0, 0)
    };

}

void GaussianWindModel::set_strength_m_per_s(double strength_m_per_s, double strength_variance) {
  double standard_deviation = sqrt(strength_variance);

  // Set max to within 3 standard deviations by default, 99.7%
  strength_max_ = strength_m_per_s + 3 * standard_deviation;

  strength_distribution_ = std::normal_distribution<double>(strength_m_per_s, standard_deviation);
}

void GaussianWindModel::set_direction(v3 direction , double variance) {
  set_direction(direction, {variance, variance, variance});
}

void GaussianWindModel::set_direction(v3 direction, v3 variance) {
  v3 dir_norm = direction.Normalize();

  direction_distribution_X_ = std::normal_distribution<double>(dir_norm.X(), sqrt(variance.X()));
  direction_distribution_Y_ = std::normal_distribution<double>(dir_norm.Y(), sqrt(variance.Y()));
  direction_distribution_Z_ = std::normal_distribution<double>(dir_norm.Z(), sqrt(variance.Z()));
}

double GaussianWindModel::generate_random_wind_strength() {
    double strength = strength_distribution_(strength_generator_);

    strength = (strength > strength_max_) ? strength_max_ : strength;

    return strength;
}

v3 GaussianWindModel::generate_random_wind_direction() {
    v3 direction;

    direction.X() = direction_distribution_X_(direction_generator_);
    direction.Y() = direction_distribution_Y_(direction_generator_);
    direction.Z() = direction_distribution_Z_(direction_generator_);

    return direction.Normalize();

}


}
