#pragma once

#include "ITransmissionMechanism.h"
#include "Math_util.hpp"

namespace avionics_sim{
class FourBarMechanism : public ITransmissionMechanism {
	public:
        FourBarMechanism(const double frame_crank_ratio, const double frame_rocker_ratio);

        /**
         * Takes a normalized input from -1 to 1 which maps to a
         * crank angle of -180 to 180 respectively. Input outside
         * this range is saturated to the bounds
         */
        virtual double transmit(double normalized_input);

    private:
        double crank_angle_from_normalized_input(double);
        double bound_normalized_input(double);
        double bound_output_rad(double);

        double _k1; ///< Frame / Crank Ratio
        double _k2; ///< Frame / Rocker Ratio

        static constexpr double MAX_DEFLECTION_RAD = DEG2RAD(24.98); // positioned at 24.98 degrees max
        static constexpr double MIN_DEFLECTION_RAD = DEG2RAD(-24.98);


};
}
