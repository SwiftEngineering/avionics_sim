/**
 * @brief       Airfoil class. This class holds physical information about the airfoil (surface area, etc), as well as objects
 * pertaining to its aerodynamic state, LUT values, and direction vectors.
 * @file        Airfoil.hpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <string>
#include <ignition/math.hh>
#include "LookupTable.hpp"

namespace avionics_sim {

// Should be moved as static const in airfoil
static const std::string _ANGLE_OF_ATTACK_ID = "alpha";
static const std::string _CL_ID = "CL";
static const std::string _CD_ID = "CD";

class Airfoil {
  public:
    Airfoil();
    Airfoil(
      double area_m2,
      double lateralArea_m2,
      const std::vector<double> &angleOfAttacks_deg,
      const std::vector<double> &cls,
      const std::vector<double> &cds);

    double calculateLiftCoefficient(double angleOfAttack_deg);
    double calculateDragCoefficient(double angleOfAttack_deg);
    double calculateSideSlipCoefficient(double angleOfAttack_deg);

    double getArea_m2();
    double getLateralArea_m2();

  protected:
    double _area_m2;
    double _lateralArea_m2;
    LookupTable _aeroLUT_deg = LookupTable({
    _ANGLE_OF_ATTACK_ID, _CL_ID, _CD_ID
  }, {{0}, {0}, {0}});

  private:
    //Until a range of values are provided for coefficient of lateral force, a value of 0.1 will be presumed.
    constexpr static double _sideSlipCoefficient = 0.1;
  //   static const std::string _ANGLE_OF_ATTACK_ID;
  //   static const std::string _CL_ID;
  //   static const std::string _CD_ID;

};



// const std::string Airfoil::_ANGLE_OF_ATTACK_ID = "alpha";
// const std::string Airfoil::_CL_ID = "CL";
// const std::string Airfoil::_CD_ID = "CD";

}
