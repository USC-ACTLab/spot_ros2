/**
 * @file default_point_cloud_client.hpp
 * @brief Implements PointCloudClientInterface to use the Spot C++ Client
 * @author Will Wu, ACT Lab @ Brown University
 */

#pragma once 

#include <bosdyn/client/point_cloud/point_cloud_client.h>
#include <bosdyn/client/sdk/client_sdk.h>
#include <spot_driver/api/time_sync_api.hpp>
#include <spot_driver/interfaces/point_cloud_client_interface.hpp>

#include <memory>

namespace spot_ros2 {

/**
 * @brief Implementation of PointCloudClientInterface with spot c++ sdk
 */
class DefaultPointCloudClient : public PointCloudClientInterface {
  public:
    DefaultPointCloudClient(::bosdyn::client::PointCloudClient *pc_client, 
                            std::shared_ptr<TimeSyncApi> time_sync_api,
                            std::string &robot_name);

    [[nodiscard]] tl::expected<::bosdyn::api::PointCloud, std::string> getPointCloud(::bosdyn::api::GetPointCloudRequest request)

  private:
    ::bosdyn::client::PointCloudClient* pc_client_;
    std::shared_ptr<TimeSyncApi> time_sync_api_;
    std::string robot_name_;
};
} // namespace spot_ros2