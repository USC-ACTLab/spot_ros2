/**
 * @file point_cloud_client.hpp
 * @brief interfaces to SPOT point cloud client and helpers
 * @author Will Wu, ACT Lab @ Brown University
 */

#pragma once

#include <bosdyn/api/point_cloud.pb.h>
#include <spot_driver/types.hpp>
#include <tl_expected/expected.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

namespace spot_ros2 {

/**
 * @brief Defines an interface for a class to interact with Spot's point cloud client
 */
class PointCloudClientInterface {
  public:
    // client is move-only
    PointCloudClientInterface() = default;
    PointCloudClientInterface(PointCloudClientInterface&& other) = default;
    PointCloudClientInterface(const PointCloudClientInterface&) = delete;
    PointCloudClientInterface& operator=(PointCloudClientInterface&& other) = default;
    PointCloudClientInterface& operator=(const PointCloudClientInterface&) = delete;
    virtual ~PointCloudClientInterface() = default;
    virtual tl::expected<sensor_msgs::msg::PointCloud2, std::string> getPointCloud(::bosdyn::api::GetPointCloudRequest request) = 0;
};
} // namespace spot_ros2