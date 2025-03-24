// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MULTISENSORCALIBRATION_LIDARVEHICLEREGISTRATIONPARAMETERS_HPP
#define MULTISENSORCALIBRATION_LIDARVEHICLEREGISTRATIONPARAMETERS_HPP

// Std
#include <unordered_map>

// ROS
#include <rclcpp/rclcpp.hpp>

// multisensor_calibration
#include "DynamicParameter.hpp"

namespace multisensor_calibration
{

struct LidarVehicleRegistrationParameters
{
    //--- ENUM DECLARATION ---//

    enum EIcpMethod
    {
        ICP = 0,
        PlaneICP,
        GICP
    };

    // number of neighbors
    DynamicParameter<int>
      region_num_neighbors = DynamicParameter<int>(
        50,
        "Number of neighbors to select around seed point for estimating plane parameters.",
        10, 100);

    // local plane flag
    DynamicParameter<bool>
      region_use_local_plane = DynamicParameter<bool>(
        true,
        "Flag whether to estimate a local plane with a center point and a radius. Uncheck "
        "to estimate an infinate plane.");

    // radius of local plane
    DynamicParameter<double>
      local_plane_radius = DynamicParameter<double>(
        1.5,
        "Radius (in m) of local plane, if it is to be estimated.",
        0.5, 5.0);

    // local plane distance thresh
    DynamicParameter<double>
      local_plane_distance_thresh = DynamicParameter<double>(
        0.05,
        "Distance threshold (in m) from model for points to count as inliers during RANSAC.",
        0.01, 0.5);

    // target ICP variant
    DynamicParameter<int>
      registration_icp_variant = DynamicParameter<int>(
        2,
        "Select ICP variant to use for registration."
        "\n\t0 = ICP,"
        "\n\t1 = PlaneICP,"
        "\n\t2 = GICP",
        0, 2, 1);

    // target ICP maximum correspondence distance
    DynamicParameter<double>
      registration_icp_max_correspondence_distance = DynamicParameter<double>(
        0.1,
        "Maximum distance for ICP to search for point correspondences. "
        "Given as ratio with respect to shorter side of calibration target.",
        0.001, 10.0);

    // target ICP rotation tolerance
    DynamicParameter<double>
      registration_icp_rotation_tolerance = DynamicParameter<double>(
        0.5,
        "Rotation tolerance for convergence check. Given in degrees.",
        0.001, 10.0);

    // target ICP translation tolerance
    DynamicParameter<double>
      registration_icp_translation_tolerance = DynamicParameter<double>(
        0.001,
        "Translation tolerance for convergence check. Given in unit of the"
        "LiDAR point cloud, typically meters.",
        0.00001, 0.1);

    /**
     * @brief Declare parameters as dynamic to be adjusted during runtime.
     *
     * @param[in] ipNode Pointer to node for which the parameters are to be declared.
     */
    void declareDynamic(rclcpp::Node* ipNode) const
    {
        DECLARE_PARAMETER(region_num_neighbors, ipNode)
        DECLARE_PARAMETER(region_use_local_plane, ipNode)
        DECLARE_PARAMETER(local_plane_radius, ipNode)
        DECLARE_PARAMETER(local_plane_distance_thresh, ipNode)

        DECLARE_PARAMETER(registration_icp_variant, ipNode)
        DECLARE_PARAMETER(registration_icp_max_correspondence_distance, ipNode)
        DECLARE_PARAMETER(registration_icp_rotation_tolerance, ipNode)
        DECLARE_PARAMETER(registration_icp_translation_tolerance, ipNode)
    }

    /**
     * @brief Try to set available parameter from given rclcpp::Parameter.
     *
     * This will try through all parameters in list and try to set value.
     *
     * @return True, if successful, i.e., if a parameter from the list has been updated.
     * False otherwise.
     */
    bool tryToSetParameter(const rclcpp::Parameter& iParameter)
    {
        SET_PARAMETER_FROM_RCLCPP_PARAM(region_num_neighbors, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(region_use_local_plane, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(local_plane_radius, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(local_plane_distance_thresh, iParameter)

        SET_PARAMETER_FROM_RCLCPP_PARAM(registration_icp_variant, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(registration_icp_max_correspondence_distance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(registration_icp_rotation_tolerance, iParameter)
        SET_PARAMETER_FROM_RCLCPP_PARAM(registration_icp_translation_tolerance, iParameter)

        return false;
    }
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_LIDARVEHICLEREGISTRATIONPARAMETERS_HPP