/**
 * @brief       MathUtil FP (Floating Point) Comparison Parameterized Unit Test Class
 * @file        MathUtilFPComparisonParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// MathUtilFPComparisonParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

const unsigned int EQUALS=1;
const unsigned int LESS_THAN=2;
const unsigned int GREATER_THAN=3;
const unsigned int LESS_THAN_EQ=4;
const unsigned int GREATER_THAN_EQ=5; 
typedef struct MathUtilFPComparisonParams {
	double lhs;
	double rhs;
	unsigned int operation;
	bool testValue;
}MathUtilFPComparisonParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<MathUtilFPComparisonParams> MathUtilFPComparisonParameterCollections;

class MathUtilFPComparisonParameterized :
    public ::testing::TestWithParam<MathUtilFPComparisonParams> {

		protected:

			///Default constructor
			MathUtilFPComparisonParameterized() {};

			///Destructor
			virtual ~MathUtilFPComparisonParameterized() {};

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