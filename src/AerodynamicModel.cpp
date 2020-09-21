/**
 * @brief       AerodynamicModel
 * @file        AerodynamicModel.cpp
 * @author      Evan Johnson <erjohnson227@gmail.com>, Conrad McGreal <cmcgreal@swiftengineering.com>, Mike Lyons <mlyons@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>, Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "AerodynamicModel.hpp"
#include "Coordinate_Utils.hpp"
#include <functional>
#include <errno.h>
#include <algorithm>

#include <sstream>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iterator>
#include <cmath>

namespace avionics_sim {
AerodynamicModel::AerodynamicModel() :
  vecFwd(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  vecUpwd(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  vecPort(ignition::math::Vector3d(0.0, 0.0, 0.0)) {
}

AerodynamicModel::AerodynamicModel(Airfoil airfoil, PhysicsEnvironment environment) {
  _airfoil = airfoil;
  _environment = environment;
}

AerodynamicModel::~AerodynamicModel() {
}

ignition::math::Vector3d AerodynamicModel::updateForcesInBody_N(
  ignition::math::Pose3d poseInWorld_m_rad,
  ignition::math::Vector3d velocityInWorld_m_per_s,
  double propWash_m_per_s,
  double controlAngle_rad) {

    _state = AerodynamicState();
    _state.poseWorld_m_rad = poseInWorld_m_rad;
    _state.velocityWorld_m_per_s = velocityInWorld_m_per_s;

    _state.velocityBody_m_per_s = transformToLocalVelocity(poseInWorld_m_rad, velocityInWorld_m_per_s);

    double planarVelocity_m_per_s = transformBodyToWindPlanar(_state.velocityBody_m_per_s );
    double lateralVelocity_m_per_s = transformBodyToWindLateral(_state.velocityBody_m_per_s );

    std::pair<double, double> aeroAngles_deg = calculateWindAngles( _state.velocityBody_m_per_s );

    if(!isnan(propWash_m_per_s)) {
      if(planarVelocity_m_per_s < 0) {
        planarVelocity_m_per_s += propWash_m_per_s;
      } else {
        planarVelocity_m_per_s = propWash_m_per_s;
      }
      lateralVelocity_m_per_s = 0;
    }

    if(!isnan(controlAngle_rad)) {
      aeroAngles_deg.first = RAD2DEG(controlAngle_rad);

      // Flip angles if wind is coming from behind/going backwards
      if(planarVelocity_m_per_s < 0) {
        aeroAngles_deg.first -= RAD2DEG(M_PI) * ignition::math::sgn(aeroAngles_deg.first);
      }
    }

  return updateForcesInBody_N(
    planarVelocity_m_per_s,
    lateralVelocity_m_per_s,
    aeroAngles_deg.first,
    aeroAngles_deg.second);
}

/**
 * Calculates the lift and drag for a body in the aerodynamic
 */
ignition::math::Vector3d AerodynamicModel::updateForcesInBody_N(
    double planarVelocity_m_per_s,
    double lateralVelocity_m_per_s,
    double angleOfAttack_deg,
    double sideSlipAngle_deg) {

  _state.angleOfAttack_deg = angleOfAttack_deg;
  _state.sideSlipAngle_deg = sideSlipAngle_deg;

  _state.planarVelocity_m_per_s = planarVelocity_m_per_s;
  _state.lateralVelocity_m_per_s = lateralVelocity_m_per_s;

  _state.dynamicPressurePlanar_Pa = calculateDynamicPressure_Pa(_state.planarVelocity_m_per_s);
  _state.dynamicPressureLateral_Pa = calculateDynamicPressure_Pa(_state.lateralVelocity_m_per_s);

  _state.liftCoeff = _airfoil.calculateLiftCoefficient(_state.angleOfAttack_deg);
  _state.dragCoeff = _airfoil.calculateDragCoefficient(_state.angleOfAttack_deg);
  _state.lateralDragCoeff = _airfoil.calculateSideSlipCoefficient(_state.sideSlipAngle_deg);

  _state.lift_N = calculateLift_N(_state.liftCoeff, _state.dynamicPressurePlanar_Pa);
  _state.drag_N = calculateDrag_N(_state.dragCoeff, _state.dynamicPressurePlanar_Pa);
  _state.lateralForce_N = calculateLateralForce_N(_state.lateralDragCoeff, _state.dynamicPressureLateral_Pa);

  _state.force_N = rotateForcesToBody(_state.lift_N, _state.drag_N, _state.lateralForce_N, _state.angleOfAttack_deg, _state.sideSlipAngle_deg);

  return _state.force_N;

}

ignition::math::Vector3d AerodynamicModel::rotateForcesToBody(double lift_N, double drag_N, double lateralForce_N, double angleOfAttack_deg, double sideSlipDrag_deg) {
  double angleOfAttacks_rad = DEG2RAD(angleOfAttack_deg);
  double rotated_lift_N = (lift_N * cos(angleOfAttacks_rad)) + (drag_N * sin(angleOfAttacks_rad));
  double rotated_drag_N = (lift_N * sin(angleOfAttacks_rad)) - (drag_N * cos(angleOfAttacks_rad));

  double oriented_lateral_N = ignition::math::sgn(sideSlipDrag_deg) * lateralForce_N;

  //Add the forces together.
  ignition::math::Vector3d force_N = rotated_lift_N * vecUpwd + rotated_drag_N * vecFwd + oriented_lateral_N * vecPort;

  return force_N;
}


void AerodynamicModel::setBasisVectors(ignition::math::Vector3d forward,
                                      ignition::math::Vector3d upward) {
  vecFwd = forward;
  vecUpwd = upward;

  //Eliminate any NaN values.
  vecFwd.Correct();
  vecUpwd.Correct();

  vecPort =  vecFwd.Cross(vecUpwd);
}

double AerodynamicModel::calculateDynamicPressure_Pa(double velocity_m_per_s) {
  return 0.5 * _environment.getAirDensity_kg_per_m3() * velocity_m_per_s * velocity_m_per_s;
}

std::pair<double,double> AerodynamicModel::calculateWindAngles(ignition::math::Vector3d velocityInBody_m_per_s) {
  //Calculate velocity in each direction.
  double u = vecUpwd.Dot(velocityInBody_m_per_s);
  double v = vecPort.Dot(velocityInBody_m_per_s);
  double w = vecFwd.Dot(velocityInBody_m_per_s);

  double angleOfAttack_rad = atan2(-u, w);
  double sideSlipAngle_rad = asin(-v / velocityInBody_m_per_s.Length());

  return {RAD2DEG(angleOfAttack_rad), RAD2DEG(sideSlipAngle_rad)};
}

ignition::math::Vector3d AerodynamicModel::transformToLocalVelocity(
    ignition::math::Pose3d poseInWorld_m_rad,
    ignition::math::Vector3d velocityInWorld_m_per_s) {

      ignition::math::Vector3d velocityInBody_m_per_s;

      avionics_sim::Coordinate_Utils::project_vector_global(
        poseInWorld_m_rad,
        velocityInWorld_m_per_s,
        &velocityInBody_m_per_s);

      return velocityInBody_m_per_s;
}

double AerodynamicModel::transformBodyToWindPlanar(ignition::math::Vector3d velocityInBody_m_per_s) {
  ignition::math::Vector3d planarVelocity_m_per_s = ignition::math::Vector3d(
                                          velocityInBody_m_per_s.X(),
                                          0,
                                          velocityInBody_m_per_s.Z());

  float directionInBody = ignition::math::sgn(velocityInBody_m_per_s.Z());

  return directionInBody*planarVelocity_m_per_s.Length();
}

double AerodynamicModel::transformBodyToWindLateral(ignition::math::Vector3d velocityInBody_m_per_s) {
  return -velocityInBody_m_per_s.Y();
}
double AerodynamicModel::calculateLift_N(double liftCoefficient, double dynamicPressure_Pa) {
  return liftCoefficient * dynamicPressure_Pa * _airfoil.getArea_m2();
}

double AerodynamicModel::calculateDrag_N(double dragCoefficient, double dynamicPressure_Pa) {
  return dragCoefficient * dynamicPressure_Pa * _airfoil.getArea_m2();
}

double AerodynamicModel::calculateLateralForce_N(double lateralDragCoefficient, double dynamicPressureLateral_Pa) {
  return dynamicPressureLateral_Pa * lateralDragCoefficient * _airfoil.getLateralArea_m2();
}


}
