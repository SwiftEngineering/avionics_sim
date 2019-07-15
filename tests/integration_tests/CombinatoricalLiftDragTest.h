/**
 * @brief       Combinatorical Lift Drag Test class
 * @file        CombinatoricalLiftDragTest.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <ignition/math.hh>
#include <stdio.h>
#include "avionics_sim/Lift_drag_model.hpp"

// LiftDragParams structure   
///
/// \brief      Holds lift drag plugin input data.
///
/// \details    Defined as a struct not class because it is designed to be just a data container.
///
typedef struct LiftDragParams {
	std::vector<std::string> worldLinearVelocities;
	std::vector<std::string> exitVelocities;
	std::vector<std::string> worldOrientations; 
	std::vector<std::string> controlSurfaceDeflections;
	std::vector<std::string> plantModels;
	float vInfTruths; 
	float alphaDegTruths;
	std::vector<std::string> liftDragForceVectorTruths;
}LiftDragParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragParams> LiftDragParameterCollections;

// CombinatoricalLiftDragTest class  
///
/// \brief      Subclass of testing::TestWithParam that accepts lift drag parameters as input for test.
///
/// \details    N/A
///
class CombinatoricalLiftDragTest :
    public ::testing::TestWithParam<LiftDragParams> {

		protected:

			///Default constructor
			CombinatoricalLiftDragTest() {};

			///Destructor
			virtual ~CombinatoricalLiftDragTest() {};

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
			
		private:

			// Function to generate HTML report upon completion of test.   
			///
			/// \brief      N/A 
			///
			/// \details    adapted from https://unmesh.me/2012/05/15/using-google-charts-api-for-test-results-dashboard/
			/// \param[in]  N/A
			/// \return     N/A
			///
  			void generateReportOverview();

	};
