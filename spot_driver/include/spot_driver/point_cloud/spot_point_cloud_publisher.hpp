/**
 * @file spot_point_cloud_publisher.hpp
 * @brief defines internal publisher to connect with Spot and read/publish point cloud data
 * @author Will Wu, ACT Lab @ Brown University
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spot_driver/api/middleware_handle_base.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/point_cloud_client_interface.hpp>
#include <spot_driver/interfaces/tf_broadcaster_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>
#include <spot_driver/types.hpp>
#include <tl_expected/expected.hpp>

namespace spot_ros2::point_cloud {

class SpotPointCloudPublisher {
 public:
  /**
   * @brief A handle class around rclcpp::Node operations for SpotImagePublisher
   */
  class MiddlewareHandle : public MiddlewareHandleBase {
   public:
    virtual ~MiddlewareHandle() = default;

    virtual tl::expected<void, std::string> publishPointCloud(const sensor_msgs::msg::PointCloud2& msg) = 0;
  };

  /**
   * @brief Constructor for SpotPointCloudPublisher, which allows setting the specific implementation of the interface
   * classes which are used by the node.
   * @details This can be in production and to perform dependency injection in unit tests
   *
   * @param image_client_interface  A shared_ptr to an instance of a class that implements ImageClientInterface.
   * @param middleware_handle A unique_ptr to an instance of a class that implements
   * SpotPointCloudPublisher::MiddlewareHandle
   */
  SpotPointCloudPublisher(const std::shared_ptr<PointCloudClientInterface>& pc_client_interface,
                          std::unique_ptr<MiddlewareHandle> middleware_handle,
                          std::unique_ptr<ParameterInterfaceBase> parameters,
                          std::unique_ptr<LoggerInterfaceBase> logger,
                          std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                          std::unique_ptr<TimerInterfaceBase> timer);

 private:
  /**
   * @brief Callback function which is called through timer_interface_.
   * @details Requests point cloud from Spot, and then publishes the point cloud
   */
  void timerCallback();

  /**
   * @brief Create a point cloud request message for the velodyne LiDAR
   */
  void createVLPPointCloudRequest();

  /**
   * @brief Point cloud request message which is set when initialize() is called.
   */
  std::optional<::bosdyn::api::GetPointCloudRequest> point_cloud_request_msg_;

  // Interface classes to interact with Spot and the middleware.
  std::shared_ptr<PointCloudClientInterface> pc_client_interface_;
  std::unique_ptr<MiddlewareHandle> middleware_handle_;

  std::unique_ptr<ParameterInterfaceBase> parameters_;
  std::unique_ptr<LoggerInterfaceBase> logger_;
  std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster_;
  std::unique_ptr<TimerInterfaceBase> timer_;
};
}  // namespace spot_ros2::point_cloud
