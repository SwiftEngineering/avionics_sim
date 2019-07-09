/**
 * @brief       Coordinate_Utils
 * @file        Coordinate_Utils.hpp
 * @author      Evan Johnson <ejohnson@swiftengineering.com>
 * @copyright   Copyright (c) 2019, Swift Engineering Inc.
 * @license     Licensed under the MIT license. See LICENSE for details.
 */

#include <ignition/math.hh>

namespace avionics_sim
{
    class Coordinate_Utils {
      public: 
        static int project_vector_global(ignition::math::Pose3d targetFrame, ignition::math::Vector3d vec, ignition::math::Vector3d * const res);
    };
}
