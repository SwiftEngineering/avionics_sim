/**
 * @brief       Motor Model Parameterized Set Exit Velocity Unit Test Class
 * @file        MotorModelSetExitVelocityParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// MotorModelSetExitVelocityParams class  
///
/// \brief      Subclass of testing::TestWithParam that accepts thrust as input for test.
///
/// \details    N/A
///

typedef struct MotorModelSetExitVelocityParams {
	double exitVelocity;

	//TODO: Once Motor model class is created, compare against its constants instead of passing them in from without (which is currently happening with these below)
	double minExitVelocity;
	double maxExitVelocity;
}MotorModelSetExitVelocityParams;

//Type definition for MotorParameterCollections.
typedef std::vector<MotorModelSetExitVelocityParams> MotorModelSetExitVelocityParameterCollections;

class MotorModelSetExitVelocityParameterized :
    public ::testing::TestWithParam<MotorModelSetExitVelocityParams> {

		protected:

			///Default constructor
			MotorModelSetExitVelocityParameterized() {};

			///Destructor
			virtual ~MotorModelSetExitVelocityParameterized() {};

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