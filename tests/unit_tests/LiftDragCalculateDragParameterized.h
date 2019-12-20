/**
 * @brief       LiftDrag::CalculateDrag Parameterized Unit Test Class
 * @file        LiftDragCalculateDragParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>

// LiftDragCalculateDragParameterized class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setSpeed related parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragCalculateDragParams {
	double rho;
	double vInf;
	double angle;
	std::vector<double> LUT_alpha;
	std::vector<double> LUT_CL;
	std::vector<double> LUT_CD;
}LiftDragCalculateDragParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragCalculateDragParams> LiftDragCalculateDragParameterCollections;

class LiftDragCalculateDragParameterized :
    public ::testing::TestWithParam<LiftDragCalculateDragParams> {

		protected:

			///Default constructor
			LiftDragCalculateDragParameterized() {};

			///Destructor
			virtual ~LiftDragCalculateDragParameterized() {};

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
