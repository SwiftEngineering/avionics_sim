/**
 * @brief       Battery_model
 * @file        Battery_model.cpp
 * @author      Richard Kirby <rkirby@kspresearch.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc. All rights reserved.
 * @license     Internal Use Only.
 */

#include "avionics_sim/Battery_model.hpp"

#include <boost/math/tools/rational.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace avionics_sim
{

bool Battery_model::initialize(const double initial_soc, const double capacity, const uint8_t num_cells, const double C, const double R)
{
    if (R <= 0.0 || C < 0.0)
        return false;		// R must be > 0 or divide by zero error, set C = 0 to turn off transient response.
    m_num_discharge_curves = 0;
    m_initial_soc = initial_soc;
    m_soc = m_initial_soc;		// in Ah
    m_capacity = capacity;		// in Ah
    m_num_cells = num_cells;
    m_last_voltage = 4.2;
    m_c = C;
    m_r = R;
    m_low_current_curve_current = std::numeric_limits<double>::infinity();
    m_high_current_curve_current = -1;
    return true;
}

void Battery_model::update_soc(const double current, const double timestep, double* const voltage, double* const soc)
{
    // simple coluomb counter
    if (timestep > 0.0)
    {
        m_soc += current * timestep/3600;				// Time step is in seconds, need hours for mAhrs
        *voltage = get_voltage(m_soc * 1000, current, timestep);	// curves are in mA hours
        *soc = m_soc;
    }
    else
    {
        *voltage = m_last_voltage;
        *soc = m_soc;
    }
}

// discharge curves are in mAhrs 
double Battery_model::get_voltage(const double soc, const double current_in, const double timestep)
{
    double current = current_in;

    // Ensure that there are at least two discharge curves
    if(m_num_discharge_curves < 2)
        return m_num_cells*m_last_voltage;

    // Ensure current is between the two discharge curves
    if(current > m_high_current_curve_current)
        current = m_high_current_curve_current;
    if(current < m_low_current_curve_current)
        current = m_low_current_curve_current;

    // Find the curves that bracket the load
    int8_t low_current_discharge_curve = -1, high_current_discharge_curve = -1;
    double delta1 = std::numeric_limits<double>::infinity(), delta2 = std::numeric_limits<double>::infinity();
    for (uint8_t i = 0 ; i < m_num_discharge_curves ; i++)
    {
        if (m_discharge_curves[i].load <= current)
        {
            if (delta1 > (current - m_discharge_curves[i].load))
            {
                delta1 = current - m_discharge_curves[i].load;
                low_current_discharge_curve = i;
            }
        }

        if (m_discharge_curves[i].load >= current)
        {
            if (delta2 > (m_discharge_curves[i].load - current))
            {
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
    if(low_current_discharge_curve == high_current_discharge_curve)
        voltage = voltage_low;
    else
        voltage = ((current - m_discharge_curves[low_current_discharge_curve].load)/
            ( m_discharge_curves[high_current_discharge_curve].load - m_discharge_curves[low_current_discharge_curve].load))
            * (voltage_high - voltage_low) + voltage_low;

    // add transient response
    voltage = voltage - (m_c/m_r) * (voltage - m_last_voltage)/(timestep);
    m_last_voltage = voltage;

    return m_num_cells*voltage;
}

bool Battery_model::add_discharge_curves(const discharge_curve& dc)
{
    if (m_num_discharge_curves > max_num_discharge_curves)
        return false;
    m_discharge_curves.push_back(dc);

    if (m_discharge_curves[m_num_discharge_curves].load < m_low_current_curve_current)
    {
        m_low_current_curve_current = m_discharge_curves[m_num_discharge_curves].load;
        m_index_of_low_current_curve = m_num_discharge_curves;
    }

    if (m_discharge_curves[m_num_discharge_curves].load > m_high_current_curve_current)
    {
        m_high_current_curve_current = m_discharge_curves[m_num_discharge_curves].load;
        m_index_of_high_current_curve = m_num_discharge_curves;
    }

    m_num_discharge_curves++;
    return true;
}

double  Battery_model::compute_voltage(const double soc, const uint8_t curve)
{
    return boost::math::tools::evaluate_polynomial(m_discharge_curves[curve].c, soc);
}

}