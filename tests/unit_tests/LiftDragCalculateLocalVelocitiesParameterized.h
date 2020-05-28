/**
 * @brief       LiftDrag::CalculateLocalVelocities, LiftDrag::CalculateWindAngles Parameterized Unit Test Class
 * @file        LiftDragCalculateLocalVelocitiesParameterized.h
 * @author      Ihimu Ukpo <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <gtest/gtest.h>
#include <boost/array.hpp>
#include <stdio.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// LiftDragCalculateLocalVelocitiesAndWindAnglesParams class  
///
/// \brief      Subclass of testing::TestWithParam that accepts setVelocity related parameters as input for test.
///
/// \details    N/A
///

typedef struct LiftDragCalculateLocalVelocitiesParams {
	ignition::math::Vector3d inputWorldVel;
	ignition::math::Pose3d inputPose;
	double correctVinf;
	double correctVPlanar;
	double correctVLateral;
	ignition::math::Vector3d correctVBody;
	ignition::math::Vector3d vecFwd;
	ignition::math::Vector3d upwardVec;
}LiftDragCalculateLocalVelocitiesParams;

//Type definition for LiftDragParameterCollections.
typedef std::vector<LiftDragCalculateLocalVelocitiesParams> LiftDragCalculateLocalVelocitiesParameterCollections;

class LiftDragCalculateLocalVelocitiesParameterized :
    public ::testing::TestWithParam<LiftDragCalculateLocalVelocitiesParams> {

		protected:

			///Default constructor
			LiftDragCalculateLocalVelocitiesParameterized() {};

			///Destructor
			virtual ~LiftDragCalculateLocalVelocitiesParameterized() {};

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
