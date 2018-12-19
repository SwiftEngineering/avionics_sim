/**
 * @brief       Battery_model
 * @file        Battery_model.hpp
 * @author      Richard Kirby <rkirby@kspresearch.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc. All rights reserved.
 */

#pragma once

/* 	
 *	RC battery model for transient behavior from L. Gao, S. Liu, and R. Dougal, "Dynamic Lithium-Ion Battery Model for System Simulation"
 * 	in: IEEE Transactions on Components and Packaging Technologies, Vol 25, No. 3, September 2002. 
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <array>
#include <vector>

class Battery_model
{
public:


    Battery_model() = default;

    /// \brief Structure for battery curve load and coefficients, load is in Amps, x axis of curves is in mAh
    struct discharge_curve
    {
        double load;		///< Constant load at which curve voltage values were taken in A
        double c[6];
    };

    static const uint8_t max_num_discharge_curves = 10;	///< Maximum number of discharge curves


    /// \brief Initialize the battery
    /// \param [in] initial_soc     	Initial state of charge in Ah.
    /// \param [in] capacity	     	Battery capacity in Ah.
    /// \param [in] num_cells     		Number of cells in series in battery pack.
    /// \param [in] C     			Equivelent capacitor value in Farads for transient response
    /// \param [in] R     			Equivelent resister value for transient response in ohms
    /// \return             		Returns true if successful false otherwise.
    bool initialize(const double initial_soc, const double capacity, const uint8_t num_cells, const double C, const double R);

    /// \brief Update the state of charge
    /// \param [in] current     		Load current in Amps
    /// \param [in] timestep     		Time since last update in seconds
    /// \param [out] voltage		Voltage at end of time step in volts
    /// \param [out] soc			SOC in Ah at end of time step
    /// \return             		Returns none.
    void update_soc(const double current, const double timestep, double *voltage, double *soc);

    /// \brief Returns expected voltage at a given state of charge and current
    /// \param [in] soc     		State of charge in mAh
    /// \param [in] current     		Load in Amps
    /// \param [in] timestep     		Timestep in seconds
    /// \return             		Returns battery pack voltage.
    double get_voltage(const double soc, const double current, const double timestep);

    /// \brief Add a state of charge (discharge) vs. voltage curve
    /// \param [in] dc     			pointer to a discharge_curve structure
    /// \return             		Returns true if successful false otherwise.

    bool add_discharge_curves(const discharge_curve& dc);

    /// \brief Compute voltage at a state of charge on particular constant discharge curve
    /// \param [in] soc    			state of charge
    /// \param [in] curve                   index of discharge curve
    /// \return             		Returns voltage corresponding to state of charge on discharge curve at index value.
    double compute_voltage(const double soc, const uint8_t curve);


protected:

    double m_initial_soc;	///< Initial value of state of charge in mAh
    double m_capacity;          ///< Battery capacity in Ah
    double m_soc;		///< Current state of charge in Ah
    double m_r;                 ///< Equivelent internal resistance for transient response in ohms, must not be zero
    double m_c;                 ///< Equivelent capacitance for transient response in Farads
    double m_last_voltage;	///< Voltage at end of previous time step
    double m_index_of_low_current_curve;    ///< Index of the lowest current discharge curve
    double m_index_of_high_current_curve;   ///< Index of the highest current discharge curve
    double m_low_current_curve_current;     ///< Constant current discharge value for lowest current discharge curve
    double m_high_current_curve_current;    ///< Constant current discharge value for highest current discharge curve
    uint8_t m_num_cells;                    ///< Number of cells in battery pack
    std::vector<discharge_curve> m_discharge_curves;       ///< Array of discharge curves
    uint8_t m_num_discharge_curves;                                                 ///< Number of discharge curves that have been initialized.

};
