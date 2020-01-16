/**
 * @brief       LiftDrag::setLateralArea Parameterized Unit Test Class
 * @file        LiftDragSetLateralAreaParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// LiftDragSetLateralAreaParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragSetLateralAreaParams {
	double value;
}LiftDragSetLateralAreaParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragSetLateralAreaParams> LiftDragSetLateralAreaParameterCollections;

class LiftDragSetLateralAreaParameterized :
    public ::testing::TestWithParam<LiftDragSetLateralAreaParams> {

		protected:

			///Default constructor
			LiftDragSetLateralAreaParameterized() {};

			///Destructor
			virtual ~LiftDragSetLateralAreaParameterized() {};

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