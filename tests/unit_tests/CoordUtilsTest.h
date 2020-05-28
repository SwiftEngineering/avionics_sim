/**
 * @brief       CoordUtils Test Class
 * @file        CoordUtilsTest.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <ignition/math.hh>
#include <stdio.h>
#include "avionics_sim/Coordinate_Utils.hpp"

// CoordUtilsParams structure   
///
/// \brief      Holds coordinate utils plugin input data.
///
/// \details    Defined as a struct not class because it is designed to be just a data container.
///
typedef struct CoordUtilsParams {
	std::vector<std::string> worldLinearVelocities;
	std::vector<std::string> worldOrientations; 
	std::vector<std::string> globalVelocities;
	ignition::math::Vector3d vecFwd;
	ignition::math::Vector3d vecUpward;
}CoordUtilsParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<CoordUtilsParams> CoordUtilsDataCollections;

// CoordUtilsTest class  
///
/// \brief      Subclass of testing::TestWithParam that accepts coordinate utils parameters as input for test.
///
/// \details    N/A
///
class CoordUtilsTest :
    public ::testing::TestWithParam<CoordUtilsParams> {

		protected:

			///Default constructor
			CoordUtilsTest() {};

			///Destructor
			virtual ~CoordUtilsTest() {};

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