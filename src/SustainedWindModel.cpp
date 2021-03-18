/**
 * @brief       SustainedWindModel
 * @file        SustainedWindModel.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "SustainedWindModel.hpp"

namespace avionics_sim {

SustainedWindModel::SustainedWindModel() :
    linear_rate_(0,0,0),
    _strength(0),
    _direction(0,0,0) {
}

SustainedWindModel::SustainedWindModel(v3 linear_rate) :
    linear_rate_(linear_rate),
    _strength(linear_rate.Length()),
    _direction(linear_rate.Normalize()) {
}

SustainedWindModel::SustainedWindModel(double strength, v3 direction) {
    _strength = strength;
    _direction = direction.Normalize();
    linear_rate_ = direction.Normalize() * strength;
}

SustainedWindModel::~SustainedWindModel() {
}

WindRate SustainedWindModel::get_rates() {
    return {
      linear_rate_,
      v3(0, 0, 0)
    };
}

void SustainedWindModel::set_strength(double strength) {
    set_linear_rate(strength, _direction);
}

void SustainedWindModel::set_direction(v3 direction) {
    set_linear_rate(_strength, direction);
}

void SustainedWindModel::set_linear_rate(double strength, v3 direction){
    _strength = strength;
    _direction = direction.Normalize();

    linear_rate_ = _direction * _strength;
}

}
