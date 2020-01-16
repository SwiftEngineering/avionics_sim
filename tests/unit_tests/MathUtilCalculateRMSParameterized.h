/**
 * @brief       MathUtil Linear Map Parameterized Unit Test Class
 * @file        MathUtilCalculateRMSParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// MathUtilCalculateRMSParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

typedef struct MathUtilCalculateRMSParams {
	std::vector<double> samples;
	double count;
	double testValue;
}MathUtilCalculateRMSParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<MathUtilCalculateRMSParams> MathUtilCalculateRMSParameterCollections;

class MathUtilCalculateRMSParameterized :
    public ::testing::TestWithParam<MathUtilCalculateRMSParams> {

		protected:

			///Default constructor
			MathUtilCalculateRMSParameterized() {};

			///Destructor
			virtual ~MathUtilCalculateRMSParameterized() {};

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