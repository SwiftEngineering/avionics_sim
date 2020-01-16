/**
 * @brief       LiftDrag::CalculateLateralForce Parameterized Unit Test Class
 * @file        LiftDragCalculateDragParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// LiftDragCalculateLateralForceParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

//Alpha and LUTs not currently used, but may be in future tests, so left alone.
typedef struct LiftDragCalculateLateralForceParams {
	double rho;
	double vInf;
	double angle;
	std::vector<double> LUT_alpha;
	std::vector<double> LUT_CL;
	std::vector<double> LUT_CD;
}LiftDragCalculateLateralForceParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragCalculateLateralForceParams> LiftDragCalculateLateralForceParameterCollections;

class LiftDragCalculateLateralForceParameterized :
    public ::testing::TestWithParam<LiftDragCalculateLateralForceParams> {

		protected:

			///Default constructor
			LiftDragCalculateLateralForceParameterized() {};

			///Destructor
			virtual ~LiftDragCalculateLateralForceParameterized() {};

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
