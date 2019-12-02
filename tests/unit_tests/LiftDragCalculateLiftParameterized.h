/**
 * @brief       LiftDrag::CalculateLift Parameterized Unit Test Class
 * @file        LiftDragCalculateLiftParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// LiftDragCalculateLiftParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragCalculateLiftParams {
	double rho;
	double vInf;
	double angle;
	std::vector<float> LUT_alpha;
	std::vector<float> LUT_CL;
	std::vector<float> LUT_CD;
}LiftDragCalculateLiftParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragCalculateLiftParams> LiftDragCalculateLiftParameterCollections;

class LiftDragCalculateLiftParameterized :
    public ::testing::TestWithParam<LiftDragCalculateLiftParams> {

		protected:

			///Default constructor
			LiftDragCalculateLiftParameterized() {};

			///Destructor
			virtual ~LiftDragCalculateLiftParameterized() {};

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