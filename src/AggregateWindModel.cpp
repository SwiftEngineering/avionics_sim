/**
 * @brief       AggregateWindModel
 * @file        AggregateWindModel.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "AggregateWindModel.hpp"
#include <iostream>

namespace avionics_sim {

AggregateWindModel::AggregateWindModel() {
}

AggregateWindModel::~AggregateWindModel() {
}

void AggregateWindModel::add_model(IWindModel &wind_model) {
    _wind_models.push_back(&wind_model);
}

WindRate AggregateWindModel::get_rates() {
    WindRate wind_rate = {
        v3(0, 0, 0),
        v3(0, 0, 0)
    };

    for (IWindModel *wind_model : _wind_models) {
        if (wind_model->is_enabled()) {
            wind_rate += wind_model->get_rates();
        }
    }

    return wind_rate;
}

}  // namespace avionics_sim
