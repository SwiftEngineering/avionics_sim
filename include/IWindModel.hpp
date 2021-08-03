/**
 * @brief       IWindModel
 * @file        IWindModel.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <ignition/math.hh>

namespace avionics_sim {

typedef ignition::math::Vector3d v3;
typedef ignition::math::Pose3d pose;

struct WindRate {
    v3 linear_rate;
    v3 angular_rate;

    struct WindRate &operator+=(const WindRate &rhs) {
        linear_rate += rhs.linear_rate;
        angular_rate += rhs.angular_rate;
        return *this;
    }
};

class IWindModel {
  public:
    virtual WindRate get_rates() = 0;
    virtual ~IWindModel() {}

    virtual void enable() {
        is_enabled_ = true;
    }
    virtual void disable() {
        is_enabled_ = false;
    }
    virtual bool is_enabled() {
        return is_enabled_;
    }


  protected:
    bool is_enabled_ = true;
};
}  // namespace avionics_sim
