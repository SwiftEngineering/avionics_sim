/**
 * @brief       Lift_drag_model
 * @file        Lift_drag_model.hpp
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>, Evan Johnson <erjohnson227@gmail.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <vector>
#include <string>
#include "avionics_sim/Bilinear_interp.hpp"
#include "avionics_sim/Math_util.hpp"
#include <ignition/math.hh>

namespace avionics_sim
{
    
    class Lift_drag_model
    {
        public:
         
          Lift_drag_model(); ///< Default constructor

          // Destructor.  
          ///
          /// \brief      Destructor
          ///
          /// \details    Made virtual to allow for subclass destructor handling. 
          /// \param[in]  N/A
          /// \return     N/A
          virtual ~Lift_drag_model();
          
          // Function to set value of alpha.   
          ///
          /// \brief      Sets value of alpha. 
          ///
          /// \details    It is presumed that the value of _alpha is in degrees. Method will convert value to degrees otherwise.
          /// \param[in]  _alphaRad    value for alpha
          /// \return      N/A
          /// 
          void setAlpha(double _alpha, bool isInRadians=false);

          // Function to get value of alpha.   
          ///
          /// \brief      Returns value of alpha. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     Value for alpha in degrees.
          double getAlpha();

          // Function to set value of speed (v infinity).   
          ///
          /// \brief      Sets value of speed. 
          ///
          /// \details    N/A 
          /// \param[in]  _speed    value for speed
          /// \return      N/A 
          void setSpeed(double _speed);

          // Function to get value of speed.   
          ///
          /// \brief      Returns value of speed. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     Value for speed. 
          double getSpeed();

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
          double getAirDensity();

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
          double getArea();

	        // Function to set LUTs.   
          ///
          /// \brief      Sets value of CL, CD, and Alpha look up tables. 
          ///
          /// \details    It is presumed that the value of _alpha is in radians. Method will convert value to degrees otherwise.
          /// \param[in]  alphas       reference to vector containing alpha values.
          /// \param[in]  cls           reference to vector containing lift coefficients.
          /// \param[in]  cds           reference to vector containing drag coefficients.
          /// \return      N/A
          /// 
          void setLUTs(const std::vector<float>& alphas, const std::vector<float>& cls, const std::vector<float>& cds);

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
          /// \brief      Calculates dynamic pressure (q) 
          ///
          /// \details    Call this function once speed and rho have been set. 
          /// \param[in]  N/A
          /// \return     N/A
          double getDynamicPressure();

          // Function to get coefficient of lift from LUT.   
          ///
          /// \brief      Calculate/retrieve coefficient of lift from LUTs.
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     N/A.
          void lookupCL();

          // Function to get coefficient of drag from LUT.   
          ///
          /// \brief      Calculate/retrieve coefficient of drag from LUTs.
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     N/A.
          void lookupCD();

          // Function to calculate drag.   
          ///
          /// \brief      Returns value of drag. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     Value for drag.
          void calculateDrag();

          // Function to calculate lift.   
          ///
          /// \brief      Returns value of lift. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     Value for lift.
          void calculateLift();
          
          // Function to get the lift coefficient.   
          ///
          /// \brief      Returns value of lift coefficient. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     Value for lift coefficient.
          float getCL();

          // Function to get the drag coefficient.   
          ///
          /// \brief      Returns value of drag coefficient. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     Value for drag coefficient.
          float getCD();

          // Function to get lift.   
          ///
          /// \brief      Returns value of lift. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     Value for lift.
          double getLift();

          // Function to get drag.   
          ///
          /// \brief      Returns value of drag. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     Value for drag.
          double getDrag();

          // Function to get calculate cl, cd, lift, and drag.   
          ///
          /// \brief      Calculates cl, cd, lift, and drag. 
          ///
          /// \details    N/A 
          /// \param[in]  N/A
          /// \return     N/A.
          void calculateLiftDragModelValues();

          /// 
          /// /brief Calculates alpha from wing pose and world velocity. 
          /// 
          /// \param[in] wingPose  ignition Pose3d of wing coorinate system.
          /// \param[in] worldVel  ignition Vector3d of world linear velocity. 
          /// \param[out] alpha pointer to the resulting alpha calculated.
          /// 
          void calculateAlpha(ignition::math::Pose3d wingPose, ignition::math::Vector3d worldVel, double * const alpha = NULL, ignition::math::Vector3d * const vInf_p = NULL); 

          // Function to convert degrees to radians.   
          ///
          /// \brief      Returns value of angle in degrees. 
          ///
          /// \details    Function is public to ensure that any values returned are in radians (internal format is degrees, however).
          /// \param[in]  _angleRadians    value of angle in radians
          /// \return     Value for angle in degrees.
          double convertDegreesToRadians(double _angleDegrees);

          // Function to convert radians to degrees.   
          ///
          /// \brief      Returns value of angle in degrees. 
          ///
          /// \details    N/A 
          /// \param[in]  _angleRadians    value of angle in radians
          /// \return     Value for angle in degrees.
          double convertRadiansToDegrees(double _angleRadians);

          /// \brief minimum acceptable air density.
          /*
          Air density minimum and maximum values presume the following:
          30 degrees Celsius at Sea Level = 1.160 kg/m^3
          0 degrees Celsius at sea level = 1.29 kg/m^3
          3000 ft Density altitude = 0.9093 kg/m^3
          */
          constexpr static double MIN_AIR_DENSITY=0.9;

          /// \brief maximum acceptable air density.
          constexpr static double MAX_AIR_DENSITY=1.3;

          /// \brief minimum acceptable vInf (speed).
          constexpr static double MIN_VINF=0.0;

          /// \brief maximum acceptable vInf (speed).
          constexpr static double MAX_VINF=20.0;

          /// \brief minimum acceptable vInf (speed) for checking value of alpha.
          constexpr static double MIN_VINF_ALPHA=2.5;

          /// \brief minimum acceptable angle-of-attack (aka AoA, aka alpha) in degrees.
          /*
          AoA minimum and maximum values presme the following:
          Hover and Forward flight + 10 degrees buffer on either side.
          */
          constexpr static double MIN_AOA=-10.0;

          /// \brief maximum acceptable angle-of-attack (aka AoA, aka alpha) in degrees.
          constexpr static double MAX_AOA=110.0;

          /// \brief minimum acceptable coefficient of lift (CL)
          /*
          CL presumes limits of aircraft + margin
          */
          constexpr static float MIN_CL=-1.5;

          /// \brief maximum acceptable coefficient of lift (CL)
          constexpr static float MAX_CL=1.4;

          /// \brief minimum acceptable coefficient of drag (CD)
          /*
          CD presumes limits of aircraft + margin
          */
          constexpr static float MIN_CD=0.0;

          /// \brief maximum acceptable coefficient of drag (CD)
          constexpr static float MAX_CD=2.0;

          /// \brief minimum acceptable lift
          /*
          Lift corresponds to pushing roughly 1g.
          */
          constexpr static double MIN_LIFT=0.0;

          /// \brief maximum acceptable lift
          constexpr static double MAX_LIFT=200.0;

          /// \brief minimum acceptable drag
          /*
          Drag corresponds to pushing roughly 1g.
          */
          constexpr static double MIN_DRAG=0.0;

          /// \brief maximum acceptable drag
          constexpr static double MAX_DRAG=100.0;

          /// \brief minimum acceptable surface area
          /*
          Drag corresponds to pushing roughly 1g.
          */
          constexpr static double MIN_AREA=0.0;

          /// \brief maximum acceptable surface area
          constexpr static double MAX_AREA=1.5;

        private:

          // Function to clear the LUT, CL, and CD vectors.   
          ///
          /// \brief      Make empty the the LUT, CL, and CD vectors. 
          ///
          /// \details     N/A
          /// \param[in]   N/A
          /// \return      N/A
          /// 
          void emptyLUTAndCoefficientVectors();

          /// \Value of alpha
          double alpha;

          /// \Value of speed (v infinity)
          double vInf;

          /// \Value of rho
          double rho;

          /// \Value of area
          double area;

          /// Lift coefficient
          float cl;

          /// Drag coefficient
          float cd;

          ///Lift
          double lift;

          ///Drag
          double drag;

          /// LUTs for control surface.
	        std::vector<float> Aero_LUT_alpha;

	        std::vector<float> Aero_LUT_CL;

	        std::vector<float> Aero_LUT_CD;

          /// dynamic pressure
          double q;

          //Tolerance for floating point comparisons (double)
          constexpr static double tolerance=0.0001; //0.000000000001

          //Tolerance for floating point comparisons (float)
          constexpr static float floatTolerance=0.0001; //0.000000000001

          /// Bilinear interpolator instance
          avionics_sim::Bilinear_interp AeroInterp;

          //Math utility object (used for floating point comparison, display of digits to a certain precision)
          Math_util mu;

    };
}
