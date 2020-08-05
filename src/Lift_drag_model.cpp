/**
 * @brief       Lift_drag_model
 * @file        Lift_drag_model.cpp
 * @author      Evan Johnson <erjohnson227@gmail.com>, Conrad McGreal <cmcgreal@swiftengineering.com>, Mike Lyons <mlyons@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>, Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "Lift_drag_model.hpp"
#include "Coordinate_Utils.hpp"
#include <functional>
#include <errno.h>
#include <algorithm>

/*
Added so can now throw exceptions. Plugin should do a try/catch. Depending on exception message,
Lift Drag Plugin should be able to report a warning or critical error.
*/
#include "Lift_drag_model_exception.hpp"

#include <sstream>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iterator>
#include <cmath>

#define DEGREES_IN_RADIANS 57.2958

namespace avionics_sim {
Lift_drag_model::Lift_drag_model() :
  angleOfAttack_deg(0.0),
  vInf(0.0),
  vPlanar_m_per_s(0.0),
  airDensity_kg_per_m3(1.225),
  area(1.0),
  cl(0.0),
  cd(0.0),
  lift(0.0),
  drag(0.0),
  lateral_force(0.0),
  q(0.0),
  AeroInterp(avionics_sim::Bilinear_interp()),
  beta(0.0),
  lateralArea(0.1),
  lateral_velocity(0.0),
  force(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  wingFrameVelocity(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  worldPose(ignition::math::Pose3d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
  worldVel(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  vecFwd(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  vecUpwd(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  vecPort(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  radialSymmetry(false),
  vInLDPlane_v(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  vInFreestreamPlane_v(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  liftVec(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  dragVec(ignition::math::Vector3d(0.0, 0.0, 0.0)),
  lateralForceVec(ignition::math::Vector3d(0.0, 0.0, 0.0)) {
}

Lift_drag_model::~Lift_drag_model() {
}

void Lift_drag_model::testUpdateForcesInBody_N(
  ignition::math::Pose3d poseInWorld_m_rad,
  ignition::math::Vector3d velocityInWorld_m_per_s,
  double propWashVelocity_m_per_s,
  double controlAngle_rad) {

  setWorldPose(poseInWorld_m_rad);
  setWorldVelocity(velocityInWorld_m_per_s);

  updateWindAnglesAndLocalVelocities();

  if(!isnan(propWashVelocity_m_per_s)) {
    if (propWashVelocity_m_per_s > vInf) {
      setFreeStreamVelocity(propWashVelocity_m_per_s);
      setPlanarVelocity(propWashVelocity_m_per_s);
    }

    setAngleOfAttack(0,true);
  }

  bool rotateForces = true;
  if(!isnan(controlAngle_rad)) {
    setAngleOfAttack(controlAngle_rad, true);
    rotateForces=false;
  }

  updateLiftDragModelValues(rotateForces);
}

ignition::math::Vector3d Lift_drag_model::updateForcesInBody_N(
  ignition::math::Pose3d poseInWorld_m_rad,
  ignition::math::Vector3d velocityInWorld_m_per_s) {

  setWorldPose(poseInWorld_m_rad);
  setWorldVelocity(velocityInWorld_m_per_s);

  updateWindAnglesAndLocalVelocities();

  updateForcesInBody_N(
    getFreeStreamVelocity(),
    getPlanarVelocity_m_per_s(),
    getAngleOfAttack_deg());
}

/**
 * Calculates the lift and drag for a body in the aerodynamic
 */
ignition::math::Vector3d Lift_drag_model::updateForcesInBody_N(
    double freestreamVelocity_m_per_s,
    double planarVelocity_m_per_s,
    double angleOfAttack_deg) {

  setFreeStreamVelocity(freestreamVelocity_m_per_s);
  setPlanarVelocity(planarVelocity_m_per_s);
  setAngleOfAttack(angleOfAttack_deg);

  calculateDynamicPressure();

  updateCL();

  updateCD();

  updateLift();

  updateDrag();

  updateLateralForce();

  rotateForcesToBody();

  return getForceVector();

}

void Lift_drag_model::updateLiftDragModelValues(bool rotateForces) {

  calculateDynamicPressure();

        updateCL();

        updateCD();

        updateLift();

        updateDrag();

        updateLateralForce();

        if (rotateForces)
        {
            double alphaInRadians=DEG2RAD(angleOfAttack_deg);
            double rotated_lift = (lift*cos(alphaInRadians))+(drag*sin(alphaInRadians));
            double rotated_drag = (lift*sin(alphaInRadians))-(drag*cos(alphaInRadians));
            lift=rotated_lift;
            drag=rotated_drag;
        }

        liftVec = lift * vecUpwd;

        if (!rotateForces) {
            dragVec = drag * -vecFwd;
        } else {
            dragVec = drag * vecFwd;
        }

        //Get direction of lateral force.
        lateralForceVec = lateral_force * vecPort;

        //Add the forces together.
        force = liftVec + lateralForceVec + dragVec;

        //Correct any NaN values.
        force.Correct();
}

void Lift_drag_model::rotateForcesToBody() {
  double angleOfAttacks_rad = DEG2RAD(angleOfAttack_deg);
  double rotated_lift = (lift * cos(angleOfAttacks_rad)) + (drag * sin(
                          angleOfAttacks_rad));
  double rotated_drag = (lift * sin(angleOfAttacks_rad)) - (drag * cos(
                          angleOfAttacks_rad));

  // Get direction of lift by multiplying lift by upward vector.
  liftVec = rotated_lift * vecUpwd;

  /* Get direction of drag. This should be in the opposite direction of velocity.
  If the drag has been calculated as rotated, do not take -vecFwd as direction of drag, as this has been factored in by the rotation. Otherwise, use -vecFwd to get direction of drag.
  */
  dragVec = rotated_drag * vecFwd;

  //Get direction of lateral force.
  lateralForceVec = lateral_force * vecPort;

  //Add the forces together.
  force = liftVec + lateralForceVec + dragVec;

  //Correct any NaN values.
  force.Correct();
}


void Lift_drag_model::setBasisVectors(ignition::math::Vector3d forward,
                                      ignition::math::Vector3d upward) {
  vecFwd = forward;
  vecUpwd = upward;

  //Eliminate any NaN values.
  vecFwd.Correct();
  vecUpwd.Correct();

  vecPort =  vecFwd.Cross(vecUpwd);
}


void Lift_drag_model::setAngleOfAttack(double _alpha, bool isInRadians) {
  //Assert that value is not a NaN.
  assert(!isnan(_alpha));

  if (isInRadians) {
    angleOfAttack_deg = RAD2DEG(_alpha);
  } else {
    angleOfAttack_deg = _alpha;
  }
}

const double Lift_drag_model::getAngleOfAttack_deg() {
  return angleOfAttack_deg;
}

void Lift_drag_model::setSideSlipAngle_deg(double _beta, bool isInRadians) {
  //Assert that value is not a NaN.
  assert(!isnan(_beta));

  if (isInRadians) {
    beta = RAD2DEG(_beta);
  } else {
    beta = _beta;
  }
}

double Lift_drag_model::getSideSlipAngle_deg() {
  return beta;
}

void Lift_drag_model::setFreeStreamVelocity(double _fsVel,
    bool overridePlanarAndFreestream) {
  assert(!isnan(_fsVel));

  vInf = _fsVel;

  if (overridePlanarAndFreestream) {
    //Because the model uses vPlanar in calculating pressure only, update the calculated vPlanar velocity as well.
    setPlanarVelocity(vInf);
  }
}

double Lift_drag_model::getFreeStreamVelocity() {
  return vInf;
}

ignition::math::Vector3d Lift_drag_model::getFreeStreamVelocityVec() {
  return vInFreestreamPlane_v;
}

void Lift_drag_model::setPlanarVelocity(double _planarVel) {
  assert(!isnan(_planarVel));

  vPlanar_m_per_s = _planarVel;

}

const double Lift_drag_model::getPlanarVelocity_m_per_s() {
  return vPlanar_m_per_s;
}

void Lift_drag_model::setArea(double _area) {
  assert((!isnan(_area)) && (_area > 0));

  area = _area;
}

const double Lift_drag_model::getArea() {
  return area;
}

void Lift_drag_model::setLateralArea(double _area) {

  assert((!isnan(_area)) && (_area > 0));

  lateralArea = _area;
}

const double Lift_drag_model::getLateralArea() {
  return lateralArea;
}

void Lift_drag_model::setLUTs(const std::vector<double> &angleOfAttacks_deg,
                              const std::vector<double> &cls, const std::vector<double> &cds) {
  airfoilProfileLUT = LookupTable({ANGLE_OF_ATTACK_ID, CL_ID, CD_ID}, {angleOfAttacks_deg, cls, cds});
}

void Lift_drag_model::setAirDensity(double _rho) {
  assert(!isnan(_rho));

  airDensity_kg_per_m3 = _rho;
}

const double Lift_drag_model::getAirDensity_kg_per_m3() {
  return airDensity_kg_per_m3;
}

const double Lift_drag_model::getCL() {
  return cl;
}

const double Lift_drag_model::getCD() {
  return cd;
}


const double Lift_drag_model::getLift() {
  return lift;
}

const double Lift_drag_model::getDrag() {
  return drag;
}

const double Lift_drag_model::getLateralDrag() {
  return lateral_force;
}

void Lift_drag_model::calculateDynamicPressure() {
  q = 0.5 * airDensity_kg_per_m3 * vPlanar_m_per_s * vPlanar_m_per_s;
}

double Lift_drag_model::getDynamicPressure() {
  return q;
}

void Lift_drag_model::updateWindAngles() {
  //Calculate velocity in each direction.
  double u = vecUpwd.Dot(wingFrameVelocity);
  double v = vecPort.Dot(wingFrameVelocity);
  double w = vecFwd.Dot(wingFrameVelocity);

  // Calculate alpha
  double y = -u;

  double x = w;

  errno = 0;

  double angleOfAttack_rad = atan2(y, x);

  //Assert that there are no domain errors.
  if (math_errhandling & MATH_ERRNO) {
    assert(errno != EDOM);
  }

  //Set alpha
  setAngleOfAttack(angleOfAttack_rad, true);

  //Calculate beta
  y = -v;

  double beta_rad = DEG2RAD(beta);

  if (vInf > 0) {
    beta_rad = asin(y / vInf);
  }

  //Assert that there are no domain errors.
  if (math_errhandling & MATH_ERRNO) {
    assert(errno != EDOM);
  }

  setSideSlipAngle_deg(beta_rad, true);
}

void Lift_drag_model::setLateralVelocity(double vel) {
  assert(!isnan(vel));
  lateral_velocity = vel;
}

void Lift_drag_model::calculateLocalVelocities() {
  double vInLDPlane_s; // Scalar magnitude of speed in LD plane.

  std::string exceptions = "Lift_drag_model::calculateLocalVelocities:\n";

  bool exceptionOccurred = false;
  bool criticalExceptionOccurred = false;

  ignition::math::Vector3d vWorldAircraftVelocity = ignition::math::Vector3d(
        worldVel.X(), worldVel.Y(), worldVel.Z());

  // Calculate world linear velocity in wing coordinate frame.
  avionics_sim::Coordinate_Utils::project_vector_global(worldPose,
      vWorldAircraftVelocity, &wingFrameVelocity);

  vInFreestreamPlane_v = ignition::math::Vector3d(wingFrameVelocity.X(),
                         wingFrameVelocity.Y(), wingFrameVelocity.Z());

  double vInFreestream_s = vInFreestreamPlane_v.Length();

  // Remove spanwise and vertical component
  vInLDPlane_v = ignition::math::Vector3d(wingFrameVelocity.X(), 0,
                                          wingFrameVelocity.Z());

  vInLDPlane_s = vInLDPlane_v.Length(); // Calculate scalar

  setFreeStreamVelocity(vInFreestream_s);

  // Set planar velocity.
  setPlanarVelocity(vInLDPlane_s);

  //Set lateral velocity
  setLateralVelocity(wingFrameVelocity.Y());
}

// TODO: Investigate necessity of try catch blocks, seperate into function calls
void Lift_drag_model::updateWindAnglesAndLocalVelocities() {
  AvionicsSimTryCatchBlock cldmvBlock;

  cldmvBlock.tryCatch(this,
                      &avionics_sim::Lift_drag_model::calculateLocalVelocities);

  cldmvBlock.tryCatch(this, &avionics_sim::Lift_drag_model::updateWindAngles);

  //If any exceptions occurred along the way, throw a new exception that has the combined exception history of all the prior exceptions.
  if (cldmvBlock.exceptionHasOccurred()) {
    Lift_drag_model_exception e(cldmvBlock.getExceptionMessage(),
                                cldmvBlock.exceptionIsCritical());
    throw e;
  }
}

void Lift_drag_model::setWorldPose(ignition::math::Pose3d pose) {
  worldPose = pose;

  //Correct NaN values (Calling Correct() fixes any NaN issues.). See ignition documentation for more information.
  worldPose.Correct();
}

void Lift_drag_model::setWorldVelocity(ignition::math::Vector3d vel) {
  worldVel = vel;

  //Correct NaN values (Calling Correct() fixes any NaN issues.). See ignition documentation for more information.
  worldVel.Correct();
}

void Lift_drag_model::updateCL() {
  cl = airfoilProfileLUT.lookup(angleOfAttack_deg, ANGLE_OF_ATTACK_ID, CL_ID);
}

void Lift_drag_model::updateCD() {
  cd = airfoilProfileLUT.lookup(angleOfAttack_deg, ANGLE_OF_ATTACK_ID, CD_ID);
}

void Lift_drag_model::updateLift() {
  lift = cl * q * getArea();
}

void Lift_drag_model::updateDrag() {
  drag = cd * q * getArea();
}

void Lift_drag_model::updateLateralForce() {
  double q_lat = 0.5 * airDensity_kg_per_m3 * lateral_velocity * lateral_velocity;
  lateral_force = q_lat * coefficientLateralForce * getLateralArea();
}


}
