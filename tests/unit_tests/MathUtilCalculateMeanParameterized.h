/**
 * @brief       MathUtil Calculate Mean Parameterized Unit Test Class
 * @file        MathUtilCalculateMeanParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// MathUtilCalculateMeanParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

typedef struct MathUtilCalculateMeanParams {
	std::vector<double> samples;
	double count;
	double testValue;
}MathUtilCalculateMeanParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<MathUtilCalculateMeanParams> MathUtilCalculateMeanParameterCollections;

class MathUtilCalculateMeanParameterized :
    public ::testing::TestWithParam<MathUtilCalculateMeanParams> {

		protected:

			///Default constructor
			MathUtilCalculateMeanParameterized() {};

			///Destructor
			virtual ~MathUtilCalculateMeanParameterized() {};

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