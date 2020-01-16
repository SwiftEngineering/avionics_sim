/**
 * @brief       MathUtil Linear Map Parameterized Unit Test Class
 * @file        MathUtilLinearMapParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// MathUtilLinearMapParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

typedef struct MathUtilParams {
	double x;
	double in_min;
	double in_max;
	double out_min;
	double out_max;
	double testValue;
}MathUtilLinearMapParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<MathUtilLinearMapParams> MathUtilLinearMapParameterCollections;

class MathUtilLinearMapParameterized :
    public ::testing::TestWithParam<MathUtilLinearMapParams> {

		protected:

			///Default constructor
			MathUtilLinearMapParameterized() {};

			///Destructor
			virtual ~MathUtilLinearMapParameterized() {};

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