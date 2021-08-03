/**
 * @brief       Coordinate_Utils
 * @file        Coordinate_Utils.hpp
 * @author      Evan Johnson <ejohnson@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#pragma once

#include <ignition/math.hh>
#include <math.h>

namespace avionics_sim {
class Coordinate_Utils {
  public:
    /**
     * @brief This function project a vector from one coordinate system into another.
     *
     * @param targetFrame a Pose3d containing the target frame to project the input vector onto. The
     * @param vec Vector3d to contain the vector to be projected into the target coordinate frame.
     * @param res Vector3d containing the projected vector in the target coordinate frame.
     * @return int reserved for future use. Always returns 0.
     */
    static int project_vector_global(ignition::math::Pose3d targetFrame, ignition::math::Vector3d vec,
                                     ignition::math::Vector3d *const res);

    static ignition::math::Quaterniond QuatFromBasis(ignition::math::Vector3d forward, ignition::math::Vector3d up);
};
}  // namespace avionics_sim
