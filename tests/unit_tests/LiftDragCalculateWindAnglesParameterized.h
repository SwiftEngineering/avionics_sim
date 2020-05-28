/**
 * @brief       LiftDrag::CalculateWindAngles Parameterized Unit Test Class
 * @file        LiftDragCalculateLocalVelocitiesAndWindAnglesParamterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// LiftDragCalculateWindAnglesParams class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setVelocity related parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragCalculateWindAnglesParams {
	ignition::math::Vector3d inputWorldVel;
	ignition::math::Pose3d inputPose;
	ignition::math::Vector3d vFwd;
	ignition::math::Vector3d vUpwd;
	double correctAlpha;
	double correctBeta;
}LiftDragCalculateWindAnglesParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragCalculateWindAnglesParams> LiftDragCalculateWindAnglesParameterCollections;

class LiftDragCalculateWindAnglesParameterized :
    public ::testing::TestWithParam<LiftDragCalculateWindAnglesParams> {

		protected:

			///Default constructor
			LiftDragCalculateWindAnglesParameterized() {};

			///Destructor
			virtual ~LiftDragCalculateWindAnglesParameterized() {};

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
