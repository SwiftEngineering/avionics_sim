/**
 * @brief       DrydenWindModel
 * @file        DrydenWindModel.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "DrydenWindModel.hpp"

#include "Coordinate_Utils.hpp"
#include "Bilinear_interp.hpp"

namespace avionics_sim {

DrydenWindModel::DrydenWindModel(){

}

DrydenWindModel::DrydenWindModel(std::default_random_engine& random_generator, IDrydenProvider& provider) :
  provider_(&provider),
  strength_generator_(random_generator),
  wingspan_m_(0),
  launch_turbulence_intensity_(0) {

}

DrydenWindModel::DrydenWindModel(
        std::default_random_engine& random_generator,
        IDrydenProvider& provider,
        double wingspan_m,
        double launch_turbulence_intensity) :
  provider_(&provider),
  strength_generator_(random_generator),
  wingspan_m_(wingspan_m),
  launch_turbulence_intensity_(launch_turbulence_intensity) {

}

DrydenWindModel::DrydenWindModel(
      IDrydenProvider& provider,
      double wingspan_m,
      double launch_turbulence_intensity) :
  provider_(&provider),
  strength_generator_(std::default_random_engine()),
  wingspan_m_(wingspan_m),
  launch_turbulence_intensity_(launch_turbulence_intensity) {

}

DrydenWindModel::~DrydenWindModel() {

}

void DrydenWindModel::set_sample_period(double dt_s) {
  if (dt_s > 0.0) {
    _sample_period_s = dt_s;
  }
}

void DrydenWindModel::set_intensity(double intensity_m_per_s) {
  // There is no direct relationship between the max gust as defined by a METERs
  // definition and the max gust genereated from the Dryden model. What can be observed
  // is that the magnitide of the dryden rate tends to be within half of the
  // launch_turbulence_intensity magnitude; therefore we will double the desired intensity
  // and define the launch turbulence intesnsity to be such
  launch_turbulence_intensity_ = 2 * intensity_m_per_s;
}

WindRate DrydenWindModel::get_rates() {
  if(provider_ == nullptr) {
    return {
      v3(0, 0, 0),
      v3(0, 0, 0)
    };
  }

  DrydenState state = provider_->get_dryden_input();

  update_scale_length_and_intensities(state.altitude_m);

  double velocity_m_per_s = state.velocity_m_per_s.Length();

  WindFrame linear_rate_noise = generate_linear_rate_noise();

  WindFrame step_velocity_scale = calculate_step_velocity_scale(_sample_period_s, velocity_m_per_s, _scale_length);

  _linear_rate = calculate_step_linear_rate(_linear_rate, step_velocity_scale, linear_rate_noise, _intensity);

  return {
      v3(_linear_rate.u, _linear_rate.v, _linear_rate.w),
      v3(0, 0, 0)
    };
}

WindFrame DrydenWindModel::calculate_step_linear_rate(
  WindFrame rate_prev, WindFrame step_velocity_scale, WindFrame noise, WindFrame intensity){
  return {
      rate_prev.u * (1 - step_velocity_scale.u) + noise.u * intensity.u * sqrt(2 * step_velocity_scale.u),
      rate_prev.v * (1 - step_velocity_scale.v) + noise.v * intensity.v * sqrt(4 * step_velocity_scale.v),
      rate_prev.w * (1 - step_velocity_scale.w) + noise.w * intensity.w * sqrt(4 * step_velocity_scale.w)
  };
}

WindFrame DrydenWindModel::calculate_step_velocity_scale(double sample_period_s, double velocity_m_per_s, WindFrame scale_length) {
  return {
    sample_period_s * velocity_m_per_s / scale_length.u,
    sample_period_s * velocity_m_per_s / scale_length.v,
    sample_period_s * velocity_m_per_s / scale_length.w};
}

WindFrame DrydenWindModel::generate_linear_rate_noise() {
  return {
    strength_distribution_(strength_generator_),
    strength_distribution_(strength_generator_),
    strength_distribution_(strength_generator_)};
}

void DrydenWindModel::update_scale_length_and_intensities(double altitude_m) {

  WindFrame scale_length;
  WindFrame intensity;

  if (altitude_m < 304.8) {
    scale_length.u  = altitude_m / pow(0.177 + 0.0027 * altitude_m,1.2);
    scale_length.v  = scale_length.u;
    scale_length.w  = altitude_m;
  } else if( altitude_m > 609.6) {
    scale_length.u  = 762;
    scale_length.v  = 762;
    scale_length.w  = 762;
  } else {
    double low = 304.8;
    double high = 762;

    Bilinear_interp::interpolate( {304.8, 609.6}, {low, high}, altitude_m, &scale_length.u);
    Bilinear_interp::interpolate( {304.8, 609.6}, {low, high}, altitude_m, &scale_length.v);
    Bilinear_interp::interpolate( {304.8, 609.6}, {low, high}, altitude_m, &scale_length.w);
  }


  intensity.u  = 0.06*launch_turbulence_intensity_ / pow(0.177 + 0.0027*altitude_m,0.4);
  intensity.v  = intensity.u;
  intensity.w  = 0.06*launch_turbulence_intensity_;

  _scale_length = scale_length;
  _intensity = intensity;
}


}
