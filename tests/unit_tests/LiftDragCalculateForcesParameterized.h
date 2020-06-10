/**
 * @brief       LiftDrag::CalculateForcesParams Parameterized Unit Test Class
 * @file        LiftDragCalculateForcesParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "LiftDragCalculateForcesParamsStruct.h"

// LiftDragCalculateForcesParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts force calculation related parameters as input for test.
///
/// \details    N/A
///

class LiftDragCalculateForcesParameterized :
    public ::testing::TestWithParam<LiftDragCalculateForcesParams> {

		protected:

			///Default constructor
			LiftDragCalculateForcesParameterized() {};

			///Destructor
			virtual ~LiftDragCalculateForcesParameterized() {};

			// Function to prepare the objects for each test.   
			///
			/// \brief      N/A 
			///
			/// \details    N/A
			/// \param[in]  N/A
			/// \return     N/A
			///
			virtual void SetUp() {};

			// Function to release any resources allocated, as well as perform other cleanup tasks (report generation, etc.)   
			///
			/// \brief      N/A 
			///
			/// \details    N/A
			/// \param[in]  N/A
			/// \return     N/A
			///
			virtual void TearDown() {};
	};
