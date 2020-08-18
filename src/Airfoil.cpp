/**
 * @brief       AirfoilLiftDragModel class.
 * @file        AirfoilLiftDragModel.cpp
 * @author      Nicholas Luzuriaga <nluzuriaga@swiftengineering.com>, Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2020, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "Airfoil.hpp"

namespace avionics_sim {

	Airfoil::Airfoil() :
		_area_m2(0),
		_lateralArea_m2(0),
		_aeroLUT_deg({
			_ANGLE_OF_ATTACK_ID, _CL_ID, _CD_ID
			}, {{0}, {0}, {0}}) {

	}

	Airfoil::Airfoil(
		double area_m2,
		double lateralArea_m2,
		const std::vector<double> &angleOfAttacks_deg,
		const std::vector<double> &cls,
		const std::vector<double> &cds) {

		_area_m2 = area_m2;
		_lateralArea_m2 = lateralArea_m2;
  		_aeroLUT_deg = LookupTable(
			  {_ANGLE_OF_ATTACK_ID, _CL_ID, _CD_ID}, {angleOfAttacks_deg, cls, cds});
	}

	double Airfoil::calculateLiftCoefficient(double angleOfAttack_deg) {
		return _aeroLUT_deg.lookup(angleOfAttack_deg, _ANGLE_OF_ATTACK_ID, _CL_ID);
	}

	double Airfoil::calculateDragCoefficient(double angleOfAttack_deg) {
		return _aeroLUT_deg.lookup(angleOfAttack_deg, _ANGLE_OF_ATTACK_ID, _CD_ID);
	}


	double Airfoil::calculateSideSlipCoefficient(double sideSlipAngle_deg) {
		return _sideSlipCoefficient;

	}

	double Airfoil::getArea_m2() {
		return _area_m2;
	}

	double Airfoil::getLateralArea_m2() {
		return _area_m2;
	}

}
