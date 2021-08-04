/**
 * @brief       Coordinate_Utils
 * @file        Coordinate_Utils.cpp
 * @author      Evan Johnson <erjohnson227@gmail.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include "Coordinate_Utils.hpp"


namespace avionics_sim {
int Coordinate_Utils::project_vector_global(ignition::math::Pose3d targetFrame, ignition::math::Vector3d vec,
        ignition::math::Vector3d *const res) {
    // Unit direction vectors in target (world) frame.

    // Forward is positive on x-axis in world frame.
    ignition::math::Vector3d forward(1.0, 0.0, 0.0);

    // Upward is positive on z-axis in world frame.
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

ignition::math::Quaterniond Coordinate_Utils::QuatFromBasis(ignition::math::Vector3d forward,
        ignition::math::Vector3d up) {
    ignition::math::Vector3d  m2 = forward.Normalize();
    ignition::math::Vector3d  m1 = m2.Cross(up).Normalize();
    ignition::math::Vector3d  m0 = m1.Cross(m2).Normalize();


    ignition::math::Matrix4d rotAMatrix = ignition::math::Matrix4d(
            m0.X(), m0.Y(), m0.Z(), 0,
            m1.X(), m1.Y(), m1.Z(), 0,
            m2.X(), m2.Y(), m2.Z(), 0,
            0, 0, 0, 1);

    return rotAMatrix.Rotation();
}

}  // namespace avionics_sim
