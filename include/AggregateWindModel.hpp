/**
 * @brief       AggregateWindModel
 * @file        AggregateWindModel.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <random>
#include "IWindModel.hpp"
#include "Airfoil.hpp"
#include "PhysicsEnvironment.hpp"

namespace avionics_sim {

class AggregateWindModel : public IWindModel {
    public:
        AggregateWindModel();

        virtual ~AggregateWindModel();

        virtual WindRate get_rates();

        void add_model(IWindModel & wind_model);

    protected:
        std::vector<IWindModel*> _wind_models;
};
}
