/**
 * @brief       LiftDrag Parameterized Set Alpha Unit Test Class
 * @file        LiftDragSetAlphaParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// LiftDragSetAlphaParams class  
///
/// \brief      Subclass of testing::TestWithParam that accepts coordinate utils parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragSetAlphaParams {
	double value;
	bool isRadians;
	double vInf;
}LiftDragSetAlphaParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragSetAlphaParams> LiftDragSetAlphaParameterCollections;

class LiftDragSetAlphaParameterized :
    public ::testing::TestWithParam<LiftDragSetAlphaParams> {

		protected:

			///Default constructor
			LiftDragSetAlphaParameterized() {};

			///Destructor
			virtual ~LiftDragSetAlphaParameterized() {};

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