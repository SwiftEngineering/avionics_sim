/**
 * @brief       DrydenWindModel
 * @file        DrydenWindModel.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <math.h>
#include <random>
#include "IWindModel.hpp"
#include "Airfoil.hpp"

namespace avionics_sim {
static const double KTS_TO_MPERS = 0.514444;
static const double VELOCITY_THRESHOLD_M_PER_S = 15.0;
static const double LIGHT_TURBULENCE_INTENSITY_m_per_s = 15 * KTS_TO_MPERS;
static const double MODERRATE_TURBULENCE_INTENSITY_m_per_s = 30 * KTS_TO_MPERS;
static const double SEVERE_TURBULENCE_INTENSITY_m_per_s = 45 * KTS_TO_MPERS;

struct DrydenState {
  v3 velocity_m_per_s;
  double altitude_m;
};

struct WindFrame {
  double u; // longitude
  double v; // lateral
  double w; // vertical
};

class IDrydenProvider {
 public:

  virtual DrydenState get_dryden_input() = 0;
};

class DrydenWindModel : public IWindModel {
  public:
    DrydenWindModel();

    DrydenWindModel(
      std::default_random_engine& random_generator,
      IDrydenProvider& provider);

    DrydenWindModel(
      std::default_random_engine& random_generator,
      IDrydenProvider& provider,
      double wingspan_m,
      double launch_turbulence_intensity);

    DrydenWindModel(
      IDrydenProvider& provider,
      double wingspan_m,
      double launch_turbulence_intensity);

    virtual ~DrydenWindModel();

    virtual WindRate get_rates();

    void set_sample_period(double dt_s);

    void set_intensity(double instensity_m_per_s);

  protected:

    void update_scale_length_and_intensities(double altitude_m);
    WindFrame generate_linear_rate_noise();
    WindFrame calculate_step_velocity_scale(double sample_period_s, double velocity_m_per_s, WindFrame scale_length);
    WindFrame calculate_step_linear_rate(WindFrame rate_prev, WindFrame step_velocity_scale, WindFrame noise, WindFrame intensity);

    IDrydenProvider * provider_ = nullptr;

    double wingspan_m_;
    double launch_turbulence_intensity_;

    std::default_random_engine strength_generator_;
    std::normal_distribution<double> strength_distribution_;

  private:

    double _sample_period_s = 0.01;

    WindFrame _linear_rate = {0, 0, 0};
    WindFrame _scale_length;
    WindFrame _intensity;
};
}
