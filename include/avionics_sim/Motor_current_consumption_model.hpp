#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <array>
#include <vector>

class Motor_current_consumption_model
{
public:

    enum MODEL_TYPE
    {
        TORQUE  =   0,
        RPM     =   1
    };

    /// \brief Constructor for computing current from rpm
    /// \param [in] ccm     			vector of coefficients from c0 -> c??  should have at least two
    // TODO do we want to set a valid range?
    Motor_current_consumption_model(const std::vector<double>& current_consumption_model_coeff);

    /// \brief Constructor for computing current from torque with dynamic response
    /// \param [in] ccm     			vector of coefficients from c0 -> c??  should have at least two
    // TODO do we want to set a valid range?
    Motor_current_consumption_model(const std::vector<double>& current_consumption_model_coeff, const double C, const double R);

    /// \brief Returns expected current at a given rpm or torque based on the current consumption model, does not handle dynamic response
    /// \param [in] value                       Rotational velocity in RPM or torque in in-lbs
    /// \return                                 Returns current in Amps.
    double get_current(const double value);

    /// \brief Returns expected current at a given rpm or torque based on the current consumption model, includes dynamic response
    /// \param [in] value                       Rotational velocity in RPM or torque in in-lbs
    /// \return                                 Returns current in Amps.
    double get_current(const double value, const double timestep);

protected:

    std::vector<double> m_current_consumption_model_coeff;          ///< Array of discharge curves
    MODEL_TYPE m_model_type;                                        ///< Type of input used by this instance
    double m_r;                 ///< Equivelent resistance for RC type transient response, determine experimentally by fitting step data
    double m_c;                 ///< Equivelent capacitance for RC type transient response, determine experimentally by fitting step data
    double m_last_current;      ///< Previous value of current, used for dynamic response
};
