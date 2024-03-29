/**
 * @brief       Battery_model
 * @file        Battery_model.cpp
 * @author      Richard Kirby <rkirby@kspresearch.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "Battery_model.hpp"

#include <boost/math/tools/rational.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace avionics_sim {

bool Battery_model::initialize(const double initial_soc, const double capacity, const uint8_t num_cells, const double C,
                               const double R) {
    // R must be > 0 or divide by zero error, set C = 0 to turn off transient response.
    if (!(R <= 0.0 || C < 0.0)) {
        m_c = C;
        m_r = R;
    }

    m_num_discharge_curves = 0;
    m_initial_soc = initial_soc;
    m_soc = m_initial_soc;  // in Ah
    m_capacity = capacity;  // in Ah
    m_num_cells = num_cells;
    m_low_current_curve_current = std::numeric_limits<double>::infinity();
    m_high_current_curve_current = -1;
    return true;
}

void Battery_model::update_soc(const double current, const double timestep, double *const voltage, double *const soc) {
    // simple coluomb counter
    if (timestep > 0.0) {
        m_soc += current * timestep / 3600;  // Time step is in seconds, need hours for mAhrs
        *voltage = get_voltage(m_soc * 1000, current, timestep);  // curves are in mA hours
        *soc = m_soc;
    } else {
        *voltage = m_last_voltage;
        *soc = m_soc;
    }
}

void Battery_model::update_soc_ocv(const double current, const double timestep, double *const voltage,
                                   double *const soc) {
    // simple coluomb counter
    if (timestep > 0.0 && current > 0.0) {
        m_soc += current * timestep / 3600;  // Time step is in seconds, need hours for mAhrs
        *voltage = get_voltage_ocv(m_soc, current, timestep);
        *soc = m_soc;
    } else {
        *voltage = m_last_voltage_ocv;
        *soc = m_soc;
    }
}

// discharge curves are in mAhrs
double Battery_model::get_voltage(const double soc, const double current_in, const double timestep) {
    double current = current_in;

    // Ensure that there are at least two discharge curves
    if (m_num_discharge_curves < 2) {
        return m_num_cells * m_last_voltage;
    }

    // Ensure current is between the two discharge curves
    if (current > m_high_current_curve_current) {
        current = m_high_current_curve_current;
    }

    if (current < m_low_current_curve_current) {
        current = m_low_current_curve_current;
    }

    // Find the curves that bracket the load
    int8_t low_current_discharge_curve = -1, high_current_discharge_curve = -1;
    double delta1 = std::numeric_limits<double>::infinity(), delta2 = std::numeric_limits<double>::infinity();

    for (uint8_t i = 0 ; i < m_num_discharge_curves ; i++) {
        if (m_discharge_curves[i].load <= current) {
            if (delta1 > (current - m_discharge_curves[i].load)) {
                delta1 = current - m_discharge_curves[i].load;
                low_current_discharge_curve = i;
            }
        }

        if (m_discharge_curves[i].load >= current) {
            if (delta2 > (m_discharge_curves[i].load - current)) {
                delta2 = m_discharge_curves[i].load - current;
                high_current_discharge_curve = i;
            }
        }
    }

    // The low voltage comes from the high discharge current and vice versa
    double voltage_low = compute_voltage(soc, high_current_discharge_curve);
    double voltage_high = compute_voltage(soc, low_current_discharge_curve);

    // If the load is bracketed by the same curve, then the load is equal to that curve otherwise interpolate
    double voltage;

    if (low_current_discharge_curve == high_current_discharge_curve) {
        voltage = voltage_low;
    } else {
        voltage = (
                      (current - m_discharge_curves[low_current_discharge_curve].load) /
                      (
                          m_discharge_curves[high_current_discharge_curve].load
                          - m_discharge_curves[low_current_discharge_curve].load
                      )
                  )
                  * (voltage_high - voltage_low) + voltage_low;
    }

    // add transient response
    if (timestep > 0) {
        voltage = voltage - (m_c / m_r) * (voltage - m_last_voltage) / (timestep);
        m_last_voltage = voltage;
    }

    return m_num_cells * voltage;
}

// discharge curves are in mAhrs
double Battery_model::get_voltage_ocv(const double soc, const double current_in, const double timestep) {
    double current = current_in;

    // get open circuit voltage, OCV uses curve 0
    double voltage = compute_voltage(soc, 0);   // returns voltage per cell

    // compensate for internal resistance drop
    float soc_percent = (m_capacity - soc) / m_capacity;
    double ir = compute_internal_resistance(soc_percent, 1.0);
    voltage = voltage - (ir * current) / m_num_cells;

    // add transient response
    if (timestep > 0) {
        voltage = voltage - (m_c / m_r) * (voltage - m_last_voltage_ocv) / (timestep);
        m_last_voltage_ocv = voltage;
    }

    return  m_num_cells * voltage;
}

bool Battery_model::add_discharge_curves(const discharge_curve &dc) {
    if (m_num_discharge_curves > max_num_discharge_curves) {
        return false;
    }

    m_discharge_curves.push_back(dc);

    if (m_discharge_curves[m_num_discharge_curves].load < m_low_current_curve_current) {
        m_low_current_curve_current = m_discharge_curves[m_num_discharge_curves].load;
        m_index_of_low_current_curve = m_num_discharge_curves;
    }

    if (m_discharge_curves[m_num_discharge_curves].load > m_high_current_curve_current) {
        m_high_current_curve_current = m_discharge_curves[m_num_discharge_curves].load;
        m_index_of_high_current_curve = m_num_discharge_curves;
    }

    m_num_discharge_curves++;
    return true;
}

bool Battery_model::set_internal_resistance_coefficients(const internal_resistance_curve &ir_coeff) {
    if (m_internal_resistance_curve.size() > 0) {
        return false;    // currently only one internal resistance curve is supported, but
    }

    // using a vector allows support for multiple internal resistance curves, which will change with state of health
    m_internal_resistance_curve.push_back(ir_coeff);
    return true;
}

double  Battery_model::compute_voltage(double soc, const uint8_t curve) {
    return boost::math::tools::evaluate_polynomial(m_discharge_curves[curve].c, soc);
}

double Battery_model::compute_internal_resistance(double soc, double soh) {
    // allows adding additional internal resistance curves for different states of health
    // currently only one curve is supported
    const uint8_t curve =  0;

    return (boost::math::tools::evaluate_polynomial(m_internal_resistance_curve[curve].c, soc,
            m_internal_resistance_num_coef) + m_internal_resistance_curve[curve].harness_resistance);
}

double Battery_model::compute_max_power_available(const double v_volts, const double ir_ohms) {
    return v_volts * v_volts / (4 * ir_ohms);
}

double Battery_model::get_soc_percent() {
    return (m_capacity - m_soc) / m_capacity;
}

}  // namespace avionics_sim
