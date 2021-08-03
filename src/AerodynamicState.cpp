/**
 * @brief       AerodynamicState class. This class holds aero state data such as freestream velocity, angle of attack, etc.
 * @file        AerodynamicState.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "AerodynamicState.hpp"

namespace avionics_sim {

AerodynamicState::AerodynamicState() {
    angleOfAttack_deg = 0.0;
    sideSlipAngle_deg = 0.0;

    poseWorld_m_rad = ignition::math::Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    velocityWorld_m_per_s = ignition::math::Vector3d(0.0, 0.0, 0.0);

    velocityBody_m_per_s = ignition::math::Vector3d(0.0, 0.0, 0.0);

    lateralVelocity_m_per_s = 0.0;

    dynamicPressurePlanar_Pa = 0.0;
    dynamicPressureLateral_Pa = 0.0;

    liftCoeff = 0.0;
    dragCoeff = 0.0;
    lateralDragCoeff = 0.0;

    lift_N = 0.0;
    drag_N = 0.0;
    lateralForce_N = 0.0;
    force_N = ignition::math::Vector3d(0.0, 0.0, 0.0);
}



void AerodynamicState::print() {
    std::cerr
            << "Angle of Attack[deg]" << angleOfAttack_deg << '\n'
            << "SideSlipAngle[deg]" << sideSlipAngle_deg << '\n'
            << "Lateral Vel: " << lateralVelocity_m_per_s << '\n'
            << "DP Planar: " << dynamicPressurePlanar_Pa << '\n'
            << "DP Lateral: " << dynamicPressureLateral_Pa << '\n'
            << "CL: " << liftCoeff << '\n'
            << "CD: " << dragCoeff << '\n'
            << "Lift[N]: " <<  lift_N << '\n'
            << "Drag[N]: " << drag_N << '\n'
            << "Force[N]: " << force_N << '\n';
}

}  // namespace avionics_sim
