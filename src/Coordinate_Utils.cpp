/**
 * @brief       Coordinate_Utils
 * @file        Coordinate_Utils.cpp
 * @author      Evan Johnson <iukpo@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "avionics_sim/Coordinate_Utils.hpp"


namespace avionics_sim 
{
    int Coordinate_Utils::project_vector_global(ignition::math::Pose3d targetFrame, ignition::math::Vector3d vec, ignition::math::Vector3d * const res)
    {
        // Unit direction vectors in target frame. 
        ignition::math::Vector3d forward(1.0, 0.0, 0.0); 
        ignition::math::Vector3d upward(0.0, 0.0, 1.0); 

        // Rotate into target frame to get direction unit vectors. 
        ignition::math::Vector3d forwardI = targetFrame.Rot().RotateVector(forward); 
        ignition::math::Vector3d upwardI = targetFrame.Rot().RotateVector(upward); 

        // Cross product to get right direction vector. 
        ignition::math::Vector3d rightI = upwardI.Cross(forwardI).Normalize(); 

        // Project source vector into target frame. 
        *res = ignition::math::Vector3d(forwardI.Dot(vec), 
                                       rightI.Dot(vec),
                                       upwardI.Dot(vec)); 

        return 0; 
    }
}