/**
 * @brief       Differential_pressure_sensor_model
 * @file        Differential_pressure_sensor_model.hpp
 * @author      Richard Kirby <rkirby@kspresearch.com>
 * @copyright   Copyright (c) 2018, Swift Engineering Inc. All rights reserved.
 * @license     Internal Use Only.
 */

#pragma once

/* 	From: https://en.wikipedia.org/wiki/Airspeed
 *
 *  Indicated airspeed (IAS) is the airspeed indicator reading (ASIR) uncorrected for instrument,
 *  position, and other errors. From current EASA definitions: Indicated airspeed means the speed
 *  of an aircraft as shown on its pitot static airspeed indicator calibrated to reflect standard
 *  atmosphere adiabatic compressible flow at sea level uncorrected for airspeed system errors.
 *
 *  Calibrated airspeed (CAS) is indicated airspeed corrected for instrument errors, position error
 *  (due to incorrect pressure at the static port) and installation errors.
 *
 *  Equivalent airspeed (EAS) is defined as the airspeed at sea level in the International Standard
 *  Atmosphere at which the (incompressible) dynamic pressure is the same as the dynamic pressure at
 *  the true airspeed (TAS) and altitude at which the aircraft is flying.
 *
 *  The true airspeed (TAS; also KTAS, for knots true airspeed) of an aircraft is the speed of the
 *  aircraft relative to the air mass in which it is flying. The true airspeed and heading of an
 *  aircraft constitute its velocity relative to the atmosphere.  The equation used assumes < 100 knots
 *  (e.g. uncompressable)
 *
 *  Also see: https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19800015804.pdf and
 *  https://web.calpoly.edu/~rcumming/Airspeed.pdf
 *
 */

#include <cstddef>
#include <cstdint>

namespace avionics_sim
{

class Differential_pressure_sensor_model
{
public:

	Differential_pressure_sensor_model()
	{
		m_correction = 0.0;
	}

	Differential_pressure_sensor_model(const double pitot_correction)
	{
		m_correction = pitot_correction;
	}

	enum SENSOR_TYPE
	{
		HSC = 0,
		SDP3X = 1
	};

	//CONSTANTS_AIR_GAS_CONST
	static constexpr double CONSTANTS_AIR_GAS_CONST = 287.1;					// J/(kg*K)
	static constexpr double CONSTANTS_ABSOLUTE_NULL_CELSIUS = -273.15;				// Add to degrees C to get Kelvin
	static constexpr double CONSTANTS_PMIN_HSC = 1244.2;						// Pa.  1244.2Pa = 5in H2O
	static constexpr double CONSTANTS_PMAX_HSC = 0.0;						// Pa
	static constexpr double CONSTANTS_SPEED_OF_SOUND_AT_SEA_LEVEL = 340.294;			// m/s

	/// \brief Sets the pitot tube correction to compensate for pitot tube misalignment
	/// \param [in] pitot_correction     	Correction factor in m/s.
	/// \return             		None.
	void set_pitot_correction(const double pitot_correction)
	{
		m_correction = pitot_correction;
	}

	/// \brief Convert differential pressure to indicated airspeed, assumes incompressable flow 
	/// \param [in] indicated_airspeed     	Indicated airspeed in m/s.
	/// \return             		Returns differential pressure in Pa.
	double diff_press_from_indicated_airspeed(double const indicated_airspeed);

	/// \brief Convert differential pressure to indicated airspeed, assumes compressable flow, speed less than the speed of sound
	/// \param [in] indicated_airspeed     	Indicated airspeed in m/s.
	/// \return             		Returns differential pressure in Pa.
	double diff_press_from_indicated_airspeed_compressable_flow(const double indicated_airspeed);

	/// \brief Converts corrected airspeed to indicated airspeed using pitot calibration, does not account for flow loss
	/// \param [in] corrected_airspeed     	Corrected airspeed in m/s.
	/// \return             		Returns indicated airspeed in m/s.
	double indicated_airspeed_from_corrected_airspeed(const double corrected_airspeed);

	/// \brief Converts true airspeed to corrected airspeed
	/// \param [in] true_airspeed     	True airspeed in m/s.
	/// \param [in] static_pressure     	Static pressure in Pa.
	/// \param [in] temperature_celsius     Ambient temperature in degrees celsius.
	/// \return             		Returns corrected airspeed in m/s.
	double corrected_airspeed_from_true_airspeed(const double true_airspeed, const double static_pressure, const double temperature_celsius);

	/// \brief Converts true airspeed to differential pressure
	/// \param [in] true_airspeed     	True airspeed in m/s.
	/// \param [in] static_pressure     	Static pressure in Pa.
	/// \param [in] temperature_celsius     Ambient temperature in degrees celsius.
	/// \return             		Returns true airspeed in m/s.
	double diff_press_from_true_airspeed(const double true_airspeed, const double static_pressure, const double temperature_celsius);

	/// \brief computes air density
	/// \param [in] static_pressure     	Static pressure in Pa.
	/// \param [in] temperature_celsius     Ambient temperature in degrees celsius.
	/// \return             		Returns air density in kg/m^3.
	double get_air_density(const double static_pressure, const double temperature_celsius);


protected:

	double m_correction;

};

}