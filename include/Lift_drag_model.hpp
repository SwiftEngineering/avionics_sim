/**
 * @brief       Lift_drag_model
 * @file        Lift_drag_model.hpp
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

namespace avionics_sim {

static const std::string ANGLE_OF_ATTACK_ID = "alpha";
static const std::string CL_ID = "CL";
static const std::string CD_ID = "CD";

// TODO: Move this out to its own file
/**
 * \brief Interface class to the lift drag model object.
 *
 * \details In practice, communication with the model should be limited by the interface, specifying the desired usage of the model through architecture.
 * Ideal use of this interface is to handle the two cases an airfoil undergoes:
 * 1) Aircraft Velocity is dominating aerodynamics: updateForcesInBody_N
 * 2) Propwash is dominating aerodynamics: updateForcesInBody_N
 */
class Lift_Drag_Model_Interface {
 public:
  /**
   * \brief Updates the model with a pose and velocity of an airfoil and calculates the resultant lift and drag forces from it.
   */
  virtual ignition::math::Vector3d updateForcesInBody_N(
      ignition::math::Pose3d poseInWorld_m_rad,
      ignition::math::Vector3d velocityInWorld_m_per_s) = 0;

  /**
   * \brief A function called from within updateLiftDragInBody_N that is useful to extend access to for cases of evaluating aerodynamics under propwash conditions.
   */
  virtual ignition::math::Vector3d updateForcesInBody_N(
      double freestreamVelocity_m_per_s,
      double planarVelocity_m_per_s,
      double angleOfAttack_deg)  = 0;

  virtual ~Lift_Drag_Model_Interface() {};
};

class Lift_drag_model : public Lift_Drag_Model_Interface {
 public:

  Lift_drag_model(); ///< Default constructor

  // Destructor.
  ///
  /// \brief      Destructor
  /// \details    Made virtual to allow for subclass destructor handling.
  /// \param[in]  N/A
  /// \return     N/A
  ///
  virtual ~Lift_drag_model();

  virtual ignition::math::Vector3d updateForcesInBody_N(ignition::math::Pose3d
      poseInWorld_m_rad,
      ignition::math::Vector3d velocityInWorld_m_per_s);

  virtual ignition::math::Vector3d updateForcesInBody_N(
      double freestreamVelocity_m_per_s,
      double planarVelocity_m_per_s,
      double angleOfAttack_deg);

  void testUpdateForcesInBody_N(
    ignition::math::Pose3d poseInWorld_m_rad,
    ignition::math::Vector3d velocityInWorld_m_per_s,
    double propWashVelocity_m_per_s,
    double deflectionAngle_deg);

  void updateLiftDragModelValues(bool rotateForces);

  // Function to get value of the most recently determined angle of attack.
  ///
  /// \brief      Returns value of angleOfAttack_deg.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for angleOfAttack_deg in degrees.
  const double getAngleOfAttack_deg();

  // Function to set value of speed (v infinity).
  ///
  /// \brief      Sets value of freestream velocity.
  ///
  /// \details    N/A
  /// \param[in]  _speed    value for freestream velocity.
  /// \param[in]  overridePlanarAndFreestream    flag indicating whether or not to also override planar velocity
  /// (this flag should only be set to true when motor exit velocity exceeds freestream)
  /// \return      N/A
  void setFreeStreamVelocity(double _fsVel,
                             bool overridePlanarAndFreestream = false);

  // Function to get value of freestream velocity.
  ///
  /// \brief      Returns value of freestream velocity.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for freestream velocity.
  double getFreeStreamVelocity();

  // Function to get value of freestream velocity vector.
  ///
  /// \brief      Returns value of freestream velocity.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for freestream velocity.
  ignition::math::Vector3d getFreeStreamVelocityVec();

  // Function to get value of planar velocity.
  ///
  /// \brief      Returns value of planar velocity.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for planar velocity.
  const double getPlanarVelocity_m_per_s();

  // Function to set value of air density (rho).
  ///
  /// \brief      Sets value of rho.
  ///
  /// \details    N/A
  /// \param[in]  _rho    value for air density (rho)
  /// \return      N/A
  void setAirDensity(double _rho);

  // Function to get value of rho.
  ///
  /// \brief      Returns value of air density (rho).
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for air density (rho).
  const double getAirDensity_kg_per_m3();

  // Function to set value of surface area.
  ///
  /// \brief      Sets value of surface area.
  ///
  /// \details    This function is provided for convenience. A default of 1.0 will be presumed.
  /// \param[in]  _area    value for surface area
  /// \return      N/A
  void setArea(double _area);

  // Function to get value of surface area.
  ///
  /// \brief      Returns value of surface area.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for surface area.
  const double getArea();

  // Function to set lateral area.
  ///
  /// \brief      Sets value of lateral area.
  ///
  /// \details    It is presumed that the value of _alpha is in radians. Method will convert value to degrees otherwise.
  /// \param[in]  area       Lateral area in feet squared.
  /// \return      N/A
  ///
  void setLateralArea(double area);

  // Function to get value of lateral area.
  ///
  /// \brief      Returns value of lateral area.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for lateral area.
  const double getLateralArea();

  // Function to set LUTs.
  ///
  /// \brief      Sets value of CL, CD, and Alpha look up tables.
  ///
  /// \details    It is presumed that the value of _alpha is in radians. Method will convert value to degrees otherwise.
  /// \param[in]  angleOfAttacks_deg       reference to vector containing alpha values.
  /// \param[in]  cls           reference to vector containing lift coefficients.
  /// \param[in]  cds           reference to vector containing drag coefficients.
  /// \return      N/A
  ///
  void setLUTs(const std::vector<double> &angleOfAttacks_deg,
               const std::vector<double> &cls,
               const std::vector<double> &cds);

  // Function to calculate dynamic pressure.
  ///
  /// \brief      Calculates dynamic pressure (q)
  ///
  /// \details    Call this function once speed and rho have been set.
  /// \param[in]  N/A
  /// \return     N/A
  void calculateDynamicPressure();

  // Function to retrieve dynamic pressure.
  ///
  /// \brief      Returns dynamic pressure (q)
  ///
  /// \details    Call this function once speed and rho have been set.
  /// \param[in]  N/A
  /// \return     N/A
  double getDynamicPressure();



  // Function to get the lift coefficient.
  ///
  /// \brief      Returns value of lift coefficient.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for lift coefficient.
  const double getCL();

  // Function to get the drag coefficient.
  ///
  /// \brief      Returns value of drag coefficient.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for drag coefficient.
  const double getCD();

  // Function to get lift.
  ///
  /// \brief      Returns value of lift.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for lift.
  const double getLift();

  // Function to get drag.
  ///
  /// \brief      Returns value of drag.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for drag.
  const double getDrag();

  // Function to get lateral force.
  ///
  /// \brief      Returns value of lateral force.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for lateral force.
  const double getLateralDrag();

  /// \brief Drag coefficient for lateral drag
  //Until a range of values are provided for coefficient of lateral force, a value of 0.1 will be presumed.
  constexpr static double coefficientLateralForce = 0.1;

  ///
  /// /brief Returns value of beta (side slip angle).
  ///
  /// \param[in] N/A
  /// \param[out] Value for beta (side slip angle) in degrees
  ///
  double getSideSlipAngle_deg();

  // Function to get value of lateral velocity.
  ///
  /// \brief      Returns value of current lateral velocity.
  ///
  /// \details    Returns lateral velocity.
  /// \param[in]  N/A
  /// \return     Double
  ///
  double getLateralVelocity() {
    return lateral_velocity;
  };

  // Function to get local aircraft velocity.
  ///
  /// \brief      Returns value of local aircraft velocity.
  ///
  /// \details    Returns local aircraft velocity.
  /// \param[in]  N/A
  /// \return     ignition::math::Vector3d
  ///
  ignition::math::Vector3d getLocalAircraftVelocity() {
    return vLocalAircraftVelocity;
  };

  // Function to get resultant force.
  ///
  /// \brief      Returns value of resultant force.
  ///
  /// \details    Returns resultant force.
  /// \param[in]  N/A
  /// \return     ignition::math::Vector3d
  ///
  ignition::math::Vector3d getForceVector() {
    return force;
  };

  // Function to set world pose.
  ///
  /// \brief      Sets world pose.
  ///
  /// \details    Takes in given pose as world pose.
  /// \param[in]  pose  A chosen pose
  /// \return      N/A
  ///
  void setWorldPose(ignition::math::Pose3d pose);

  // Function to get value of world pose.
  ///
  /// \brief      Returns value of world pose.
  ///
  /// \details    Returns world pose.
  /// \param[in]  N/A
  /// \return     ignition::math::Pose3d
  ///
  ignition::math::Pose3d getWorldPose() {
    return worldPose;
  };

  // Function to set world velocity.
  ///
  /// \brief      Sets world velocity.
  ///
  /// \details    Takes in given pose as world velocity.
  /// \param[in]  pose  A chosen pose
  /// \return      N/A
  ///
  void setWorldVelocity(ignition::math::Vector3d vel);

  // Function to get value of world pose.
  ///
  /// \brief      Returns value of world pose.
  ///
  /// \details    Returns world pose.
  /// \param[in]  N/A
  /// \return     ignition::math::Vector3d
  ///
  const ignition::math::Vector3d getWorldVelocity() {
    return worldVel;
  };



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


  ///
  ///
  /// \brief      Get forward unit vector
  ///
  /// \details     N/A
  /// \param[in]   N/A
  /// \return      N/A
  ///
  const ignition::math::Vector3d getForwardVector() {
    return vecFwd;
  };

  ///
  /// \brief      Get upward unit vector
  ///
  /// \details     N/A
  /// \param[in]   N/A
  /// \return      N/A
  ///
  const ignition::math::Vector3d getUpwardVector() {
    return vecUpwd;
  };

  ///
  ///
  /// \brief      Get port unit vector
  ///
  /// \details     N/A
  /// \param[in]   N/A
  /// \return      N/A
  ///
  const ignition::math::Vector3d getPortVector() {
    return vecPort;
  };

  // Function to set value of speed (v planar).
  ///
  /// \brief      Sets value of planar velocity.
  ///
  /// \details    N/A
  /// \param[in]  _speed    value for planar velocity.
  /// \return      N/A
  void setPlanarVelocity(double _planarVel);

  // Function to set value of angle of attack.
  ///
  /// \brief      Sets value of alpha.
  ///
  /// \details    It is presumed that the value of _alpha is in degrees. Method will convert value to degrees otherwise.
  /// \param[in]  _alphaRad    value for alpha
  /// \return      N/A
  ///
  void setAngleOfAttack(double _alpha, bool isInRadians = false);

// Function to calculate local velocities.
  ///
  /// \brief      Calculates local velocities. Must be called before updateWindAngles.
  ///
  /// \details     N/A
  /// \param[in]   N/A
  /// \return      N/A
  ///
  void calculateLocalVelocities();

  // Function to set value of beta.
  ///
  /// \brief      Sets value of beta.
  ///
  /// \details    It is presumed that the value of _beta is in degrees. Method will convert value to degrees otherwise.
  /// \param[in]  _alphaRad    value for alpha
  /// \param[in]  isInRadians  flag indicating whether or not angle is in radians
  /// \return      N/A
  ///
  void setSideSlipAngle_deg(double _beta, bool isInRadians = false);

// Function to set value of lateral velocity.
  ///
  /// \brief      Sets value of lateral velocity using pose and world velocity to calculate value.
  ///
  /// \details    Takes in world velocity, uses project_vector_global to get body frame, then takes value from projection. In future refactor, this method should be called in calculateAlpha to streamline.
  /// \param[in] wingPose  ignition Pose3d of wing coordinate system.
  /// \param[in] worldVel  ignition Vector3d of world linear velocity.
  /// \return      N/A
  ///
  void setLateralVelocity(ignition::math::Pose3d wingPose,
                          ignition::math::Vector3d worldVel);

// Function to set value of lateral velocity.
  ///
  /// \brief      Sets value of lateral velocity to a specified value.
  ///
  /// \details    Takes in given velocity as lateral velocity.
  /// \param[in]  vel  A chosen lateral velocity
  /// \return      N/A
  ///
  void setLateralVelocity(double vel);

  // Function to get calculate cl, cd, lift, and drag.
  ///
  /// \brief      Calculates important output values of the model (cl, cd, forces)
  ///
  /// \details    N/A
  /// \param[in]  calculateRotatedForces flag determining whether or not to calculate rotated forces
  /// \return     N/A.
  void calculateLiftDragModelValues(bool calculateRotatedForces = true);


//  protected:

  // Function to get coefficient of lift from LUT.
  ///
  /// \brief      Calculate/retrieve coefficient of lift from LUTs.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     N/A.
  void updateCL();

  // Function to get coefficient of drag from LUT.
  ///
  /// \brief      Calculate/retrieve coefficient of drag from LUTs.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     N/A.
  void updateCD();

  // Function to calculate lift.
  ///
  /// \brief      Calculates value of lift.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for lift.
  void updateLift();

  // Function to calculate drag.
  ///
  /// \brief      Calculates value of drag.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for drag.
  void updateDrag();

  // Function to calculate lateral force.
  ///
  /// \brief      Calculates lateral force.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     Value for lift.
  void updateLateralForce();

  ///
  /// /brief Calculates wind angles and local velocities
  ///
  /// \details    Calls both calculateLocalVelocities and updateWindAngles as one wrapper function.
  /// \param[in]  N/A
  /// \return     N/A
  ///
  void updateWindAnglesAndLocalVelocities();

  ///
  /// /brief Calculates alpha and beta angles from wing pose and world velocity.
  ///
  /// \details    N/A
  /// \param[in]  N/A
  /// \return     N/A
  ///
  void updateWindAngles();

  ///
  /// \brief      Function to calculate force vector with direction.
  ///
  /// \details     N/A
  /// \param[in]  calculashteRotatedForces flag determining whether or not to negate drag through multiplying by -vecFwd (do not want to do this if rotated drag has been calculated)
  /// \return      N/A
  ///
  void rotateForcesToBody();

 private:

  /// \Value of alpha (angle of attack)
  double angleOfAttack_deg;

  /// \Aircraft freestream velocity (v infinity)
  double vInf;

  /// \Planar body velocity (v planar)
  double vPlanar_m_per_s;

  /// \Value of air density, rho
  double airDensity_kg_per_m3;

  /// \Value of area
  double area;

  /// Lift coefficient
  double cl;

  /// Drag coefficient
  double cd;

  ///Lift
  double lift;

  ///Drag
  double drag;

  //Lateral force
  double lateral_force;

  /// dynamic pressure
  double q;

  //Tolerance for floating point comparisons (double)
  constexpr static double tolerance = 0.0001; //0.000000000001

  //Tolerance for floating point comparisons (float)
  constexpr static double floatTolerance = 0.0001; //0.000000000001

  /// Bilinear interpolator instance
  avionics_sim::Bilinear_interp AeroInterp;

  //Math utility object (used for floating point comparison, display of digits to a certain precision)
  Math_util mu;

  /// \Value of beta (side slip angle)
  double beta;

  /// \brief Lateral area of aircraft (in ft squared)
  double lateralArea;

  /// \brief Y component of wingframe (body) velocity. Used in calculating lateral force.
  double lateral_velocity;

  /// \brief Local aircraftVelocity (aka vInfBar)
  ignition::math::Vector3d vLocalAircraftVelocity;

  /// \brief Resultant force calculated by model.
  ignition::math::Vector3d force;

  /// \brief Wing frame velocity
  ignition::math::Vector3d wingFrameVelocity;

  /// \brief World Pose
  ignition::math::Pose3d worldPose;

  /// \brief World Velocity
  ignition::math::Vector3d worldVel;

  /// \brief Forward vector
  ignition::math::Vector3d vecFwd;

  /// \brief Upward vector
  ignition::math::Vector3d vecUpwd;

  /// \brief Port vector
  ignition::math::Vector3d vecPort;

  /// \brief Flag indicating whether or not link has radial symmetry. Not yet used.
  bool radialSymmetry;

  /// \brief Vector of velocity in LD plane.
  ignition::math::Vector3d vInLDPlane_v;

  /// \brief Vector of freestream.
  ignition::math::Vector3d vInFreestreamPlane_v;

  /// \brief lift force vector
  ignition::math::Vector3d liftVec;

  /// \brief drag force vector
  ignition::math::Vector3d dragVec;

  /// \brief lateral force vector
  ignition::math::Vector3d lateralForceVec;

  /// \brief Smallest value for alpha (smallest value in LUT)
  double minAlpha;

  /// \brief Largest value for alpha (smallest value in LUT)
  double maxAlpha;

  /// LUTs for control surface.
  LookupTable airfoilProfileLUT = LookupTable({
    ANGLE_OF_ATTACK_ID, CL_ID, CD_ID
  }, {{0}, {0}, {0}});

};
}
