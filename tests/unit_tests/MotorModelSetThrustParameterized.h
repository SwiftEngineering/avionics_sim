/**
 * @brief       Motor Model Parameterized Set Thrust Unit Test Class
 * @file        MotorModelSetThrustParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// MotorModelSetThrustParams class  
///
/// \brief      Subclass of testing::TestWithParam that accepts thrust as input for test.
///
/// \details    N/A
///

typedef struct MotorModelSetThrustParams {
	double thrust;

	//TODO: Once Motor model class is created, compare against its constants instead of passing them in from without (which is currently happening with these below)
	double minThrust;
	double maxThrust;
}MotorModelSetThrustParams;

//Type definition for MotorParameterCollections.
typedef std::vector<MotorModelSetThrustParams> MotorModelSetThrustParameterCollections;

class MotorModelSetThrustParameterized :
    public ::testing::TestWithParam<MotorModelSetThrustParams> {

		protected:

			///Default constructor
			MotorModelSetThrustParameterized() {};

			///Destructor
			virtual ~MotorModelSetThrustParameterized() {};

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