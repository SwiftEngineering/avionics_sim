/**
 * @brief       AerodynamicState class. This class holds aero state data such as freestream velocity, angle of attack, etc.
 * @file        AerodynamicState.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <ignition/math.hh>

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
  ignition::math::Pose3d poseWorld_m_rad;

  /// \brief World Velocity
  ignition::math::Vector3d velocityWorld_m_per_s;

  /// \brief World Velocity
  ignition::math::Vector3d velocityBody_m_per_s;

  double angleOfAttack_deg;
  /// \Value of beta (side slip angle)
  double sideSlipAngle_deg;

  double planarVelocity_m_per_s;
  double lateralVelocity_m_per_s;

  double dynamicPressurePlanar_Pa;
  double dynamicPressureLateral_Pa;

  double liftCoeff;
  double dragCoeff;
  double lateralDragCoeff;

  double lift_N;
  double drag_N;
  double lateralForce_N;

  ignition::math::Vector3d force_N;

};

}
