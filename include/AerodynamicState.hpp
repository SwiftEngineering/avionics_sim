/**
 * @brief       AerodynamicState class. This class holds aero state data such as freestream velocity, angle of attack, etc.
 * @file        AerodynamicState.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <ignition/math.hh>
#include "Property.hpp"
#include <cmath>

namespace avionics_sim {

/**
 * @brief       AerodynamicState class. This class holds aero state of airfoil (angle of attack, etc).
 * @file        AerodynamicState.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

class AerodynamicState {
 public:

  AerodynamicState();

  void print();

  /// \brief World Pose
  Property <ignition::math::Pose3d> poseWorld_m_rad;

  /// \brief World Velocity
  Property <ignition::math::Vector3d> velocityWorld_m_per_s;

  /// \brief World Velocity
  Property <ignition::math::Vector3d> velocityBody_m_per_s;

  Property <double> angleOfAttack_deg;
  /// \Value of beta (side slip angle)
  Property <double> sideSlipAngle_deg;

  Property <double> planarVelocity_m_per_s;
  Property <double> lateralVelocity_m_per_s;

  Property <double> dynamicPressurePlanar_Pa;
  Property <double> dynamicPressureLateral_Pa;

  Property <double> liftCoeff;
  Property <double> dragCoeff;
  Property <double> lateralDragCoeff;

  Property <double> lift_N;
  Property <double> drag_N;
  Property <double> lateralForce_N;

  Property <ignition::math::Vector3d> force_N;

};

}
