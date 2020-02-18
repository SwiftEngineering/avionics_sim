#include "avionics_sim/Motor_current_consumption_model.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <cstdint>
#include <boost/math/tools/rational.hpp>

Motor_current_consumption_model::Motor_current_consumption_model(const std::vector<double>& current_consumption_model_coeff)
{
    m_current_consumption_model_coeff = current_consumption_model_coeff;
    m_model_type = RPM;
    m_c = 0;
    m_r = 1;
    m_last_current = 0;
}

Motor_current_consumption_model::Motor_current_consumption_model(const std::vector<double>& current_consumption_model_coeff, const double C, const double R)
{
    if(R<=0)
        throw("R must be greater than 0");
    if(C<0)
        throw("C must be greater than or equal to 0");
    m_current_consumption_model_coeff = current_consumption_model_coeff;
    m_model_type = TORQUE;
    m_c = C;
    m_r = R;
    m_last_current = 0;
    //TODO throw and exception if ?? is zero
}

double  Motor_current_consumption_model::get_current(const double value)
{
    return boost::math::tools::evaluate_polynomial(m_current_consumption_model_coeff.data(), value, m_current_consumption_model_coeff.size());
}

double  Motor_current_consumption_model::get_current(const double value, const double timestep)
{
    double current = get_current(value);
    current = current - (m_c/m_r) * (current - m_last_current)/(timestep);  // for RPM the dynamic response component equals 0
    m_last_current = current;
    return current;
}
