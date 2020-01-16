/**
 * @brief       Atan2 Parameterized Unit Test Class
 * @file        Atan2Parameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 * @detail     This test class was written to ensure that atan2 domain error is catchable on platform.
 * @   Copyright (c) 2019, Swift Engineering Inc.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// AtanParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

typedef struct Atan2Params {
	double y;
	double x;
}Atan2Params;

//Type definition for LiftDragParameterCollections.
typedef std::vector<Atan2Params> Atan2ParameterCollections;

class Atan2Parameterized :
    public ::testing::TestWithParam<Atan2Params> {

		protected:

			///Default constructor
			Atan2Parameterized() {};

			///Destructor
			virtual ~Atan2Parameterized() {};

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
