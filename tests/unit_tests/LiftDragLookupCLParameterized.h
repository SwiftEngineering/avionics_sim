/**
 * @brief       LiftDrag::LookupCL Parameterized Unit Test Class
 * @file        LiftDragLookupCLParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// LiftDragLookupCLParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragLookupCLParams {
	double value;
	std::vector<float> LUT_alpha;
	std::vector<float> LUT_CL;
	std::vector<float> LUT_CD;
}LiftDragLookupCLParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragLookupCLParams> LiftDragLookupCLParameterCollections;

class LiftDragLookupCLParameterized :
    public ::testing::TestWithParam<LiftDragLookupCLParams> {

		protected:

			///Default constructor
			LiftDragLookupCLParameterized() {};

			///Destructor
			virtual ~LiftDragLookupCLParameterized() {};

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