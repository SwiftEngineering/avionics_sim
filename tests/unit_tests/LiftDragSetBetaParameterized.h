/**
 * @brief       LiftDrag Parameterized Set Beta Unit Test Class
 * @file        LiftDragSetBetaParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// LiftDragSetBetaParams class  
///
/// \brief      Subclass of testing::TestWithParam that accepts coordinate utils parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragSetBetaParams {
	double value;
	bool isRadians;
}LiftDragSetBetaParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragSetBetaParams> LiftDragSetBetaParameterCollections;

class LiftDragSetBetaParameterized :
    public ::testing::TestWithParam<LiftDragSetBetaParams> {

		protected:

			///Default constructor
			LiftDragSetBetaParameterized() {};

			///Destructor
			virtual ~LiftDragSetBetaParameterized() {};

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