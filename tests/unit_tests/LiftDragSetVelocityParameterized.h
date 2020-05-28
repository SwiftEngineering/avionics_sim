/**
 * @brief       LiftDrag::SetVelocity Parameterized Unit Test Class
 * @file        LiftDragSetVelocityParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// LiftDragSetVelocityParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setVelocity related parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragSetVelocityParams {
	double value;
}LiftDragSetVelocityParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragSetVelocityParams> LiftDragSetVelocityParameterCollections;

class LiftDragSetVelocityParameterized :
    public ::testing::TestWithParam<LiftDragSetVelocityParams> {

		protected:

			///Default constructor
			LiftDragSetVelocityParameterized() {};

			///Destructor
			virtual ~LiftDragSetVelocityParameterized() {};

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