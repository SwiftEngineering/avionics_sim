/**
 * @brief       AerodynamicModel
 * @file        AerodynamicModel.hpp
 * @author      Evan Johnson <erjohnson227@gmail.com>, Conrad McGreal <cmcgreal@swiftengineering.com>, Mike Lyons <mlyons@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>, Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <vector>
#include <string>
#include <ignition/math.hh>
#include "Bilinear_interp.hpp"
#include "Math_util.hpp"
#include "LookupTable.hpp"
#include "AerodynamicState.hpp"
#include "Basis.hpp"
#include "Airfoil.hpp"
#include "PhysicsEnvironment.hpp"

namespace avionics_sim {

// TODO: Move this out to its own file
/**
 * \brief Interface class to the lift drag model object.
 *
 * \details In practice, communication with the model should be limited by the interface, specifying the desired usage of the model through architecture.
 * Ideal use of this interface is to handle the two cases an airfoil undergoes:
 * 1) Aircraft Velocity is dominating aerodynamics: updateForcesInBody_N
 * 2) Propwash is dominating aerodynamics: updateForcesInBody_N
 */
class IAerodynamicModel {
 public:
  /**
   * \brief Updates the model with a pose and velocity of an airfoil and calculates the resultant lift and drag forces from it.
   */
  virtual ignition::math::Vector3d updateForcesInBody_N(
      ignition::math::Pose3d poseInWorld_m_rad,
      ignition::math::Vector3d velocityInWorld_m_per_s,
      double propWash_m_per_s,
      double controlAngle_rad) = 0;

  /**
   * \brief A function called from within updateLiftDragInBody_N that is useful to extend access to for cases of evaluating aerodynamics under propwash conditions.
   */
  virtual ignition::math::Vector3d updateForcesInBody_N(
      double planarVelocity_m_per_s,
      double lateralVelocity_m_per_s,
      double angleOfAttack_deg,
      double sideSlipAngle_deg)  = 0;

  virtual ~IAerodynamicModel() {};
};

class AerodynamicModel : public IAerodynamicModel {
 public:

  AerodynamicModel(); ///< Default constructor

  AerodynamicModel(Airfoil airfoil, PhysicsEnvironment environment);

  // Destructor.
  ///
  /// \brief      Destructor
  /// \details    Made virtual to allow for subclass destructor handling.
  /// \param[in]  N/A
  /// \return     N/A
  ///
  virtual ~AerodynamicModel();

  virtual ignition::math::Vector3d updateForcesInBody_N(ignition::math::Pose3d
      poseInWorld_m_rad,
      ignition::math::Vector3d velocityInWorld_m_per_s,
      double propWash_m_per_s,
      double controlAngle_rad);

  virtual ignition::math::Vector3d updateForcesInBody_N(
      double planarVelocity_m_per_s,
      double lateralVelocity_m_per_s,
      double angleOfAttack_deg,
      double sideSlipAngle_deg);

  // Function to calculate dynamic pressure.
  ///
  /// \brief      Calculates dynamic pressure (q)
  ///
  /// \details    Call this function once speed and rho have been set.
  /// \param[in]  N/A
  /// \return     N/A
  double calculateDynamicPressure_Pa(double velocity_m_per_s);

  ///
  /// \brief Sets the basis vectors from only the fwd and upward vectors
  ///
  /// \details    N/A
  /// \param[in]  forward Vector3d representing forward vector
  /// \param[in]  upward Vector3d representing upward vector
  /// \return     N/A
  ///
  void setBasisVectors(ignition::math::Vector3d fwd,
                       ignition::math::Vector3d upward);

  ignition::math::Vector3d transformToLocalVelocity(
      ignition::math::Pose3d poseInWorld_m_rad,
      ignition::math::Vector3d velocityInWorld_m_per_s);

  double transformBodyToWindPlanar(ignition::math::Vector3d velocityInBody_m_per_s);
  double transformBodyToWindLateral(ignition::math::Vector3d velocityInBody_m_per_s);

  // Function to calculate lift.
  ///
  /// \brief      Calculates value of lift.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for lift.
  double calculateLift_N(double liftCoefficient, double dynamicPressure_Pa);

  // Function to calculate drag.
  ///
  /// \brief      Calculates value of drag.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for drag.
  double calculateDrag_N(double dragCoefficient, double dynamicPressure_Pa);

  // Function to calculate lateral force.
  ///
  /// \brief      Calculates lateral force.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for lift.
  double calculateLateralForce_N(double lateralDragCoefficient, double lateralVelocity_m_per_s);

  ///
  /// /brief Calculates alpha and beta angles from wing pose and world velocity.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     N/A
  ///
  std::pair<double,double> calculateWindAngles(ignition::math::Vector3d velocityInBody_m_per_s);

  ///
  /// \brief      Function to calculate force vector with direction.
  ///
  /// \details     N/A
  /// \param[in]  calculashteRotatedForces flag determining whether or not to negate drag through multiplying by -vecFwd (do not want to do this if rotated drag has been calculated)
  /// \return      N/A
  ///
  ignition::math::Vector3d rotateForcesToBody(double lift_N, double drag_N, double lateralForce_N, double angleOfAttack_deg, double sideSlipAngle_deg);

 private:

  /// \brief Forward vector
  ignition::math::Vector3d vecFwd;

  /// \brief Upward vector
  ignition::math::Vector3d vecUpwd;

  /// \brief Port vector
  ignition::math::Vector3d vecPort;

  Airfoil _airfoil;

  AerodynamicState _state;

  PhysicsEnvironment _environment;

};
}
