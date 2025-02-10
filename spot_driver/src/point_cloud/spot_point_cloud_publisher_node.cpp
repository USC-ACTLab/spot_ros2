/**
 * @file spot_point_cloud_publisher_node.cpp
 * @brief implements point cloud publisher ros node
 * @author Will Wu, ACT Lab @ Brown University
 */

#include <spot_driver/point_cloud/spot_point_cloud_publisher_node.hpp>

#include <spot_driver/api/default_spot_api.hpp>
#include <spot_driver/point_cloud/point_cloud_middleware_handle.hpp>
#include <spot_driver/point_cloud/spot_point_cloud_publisher.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_node_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>

namespace {
constexpr auto KSDKClientName = "spot_lidar_publisher";
}

namespace spot_ros2::point_cloud {

SpotPointCloudPublisherNode::SpotPointCloudPublisherNode(std::unique_ptr<SpotApi> spot_api,
                                std::unique_ptr<SpotPointCloudPublisher::MiddlewareHandle> mw_handle,
                                std::unique_ptr<ParameterInterfaceBase> parameters,
                                std::unique_ptr<LoggerInterfaceBase> logger,
                                std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                                std::unique_ptr<TimerInterfaceBase> timer,
                                std::unique_ptr<NodeInterfaceBase> node_base_interface)
    : node_base_interface_{std::move(node_base_interface)} {
    initialize(std::move(spot_api), std::move(mw_handle), std::move(parameters), std::move(logger),
               std::move(tf_broadcaster), std::move(timer));
}

SpotPointCloudPublisherNode::SpotPointCloudPublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{}) {
    const auto node = std::make_shared<rclcpp::Node>("point_cloud_publisher", node_options);
    node_base_interface_ = std::make_unique<RclcppNodeInterface>(node->get_node_base_interface());

    auto mw_handle = std::make_unique<PointCloudMiddlewareHandle>(node);
    auto parameters = std::make_unique<RclcppParameterInterface>(node);
    auto logger = std::make_unique<RclcppLoggerInterface>(node->get_logger());
    auto tf_broadcaster = std::make_unique<RclcppTfBroadcasterInterface>(node);
    auto timer = std::make_unique<RclcppWallTimerInterface>(node);

    const auto timesync_timeout = parameters->getTimeSyncTimeout();
    auto spot_api = std::make_unique<DefaultSpotApi>(kSDKClientName, timesync_timeout, parameters->getCertificate());

    initialize(std::move(spot_api), std::move(mw_handle), std::move(parameters), std::move(logger),
               std::move(tf_broadcaster), std::move(timer));
}

std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> SpotPointCloudPublisherNode::get_node_base_interface() {
    return node_base_interface_->getNodeBaseInterface();
}

void SpotPointCloudPublisherNode::initialize(std::unique_ptr<SpotApi> spot_api,
                                             std::unique_ptr<SpotPointCloudPublisher::MiddlewareHandle> mw_handle,
                                             std::unique_ptr<ParameterInterfaceBase> parameters,
                                             std::unique_ptr<LoggerInterfaceBase> logger,
                                             std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                                             std::unique_ptr<TimerInterfaceBase> timer) {
    spot_api_ = std::move(spot_api);

    const auto hostname = parameters->getHostname();
    const auto port = parameters->getPort();
    const auto robot_name = parameters->getSpotName();
    const auto username = parameters->getUsername();
    const auto password = parameters->getPassword();

    // create and authenticate robot
    if (const auto create_robot_result = spot_api_->createRobot(robot_name, hostname, port); !create_robot_result) {
      const auto error_msg{std::string{"Failed to create interface to robot: "}.append(create_robot_result.error())};
      logger->logError(error_msg);
      throw std::runtime_error(error_msg);
    }

    if (const auto authentication_result = spot_api_->authenticate(username, password); !authentication_result) {
      const auto error_msg{std::string{"Failed to authenticate robot: "}.append(authentication_result.error())};
      logger->logError(error_msg);
      throw std::runtime_error(error_msg);
    }

    internal_ = std::make_unique<SpotPointCloudPublisher>(spot_api_->point_cloud_client_interface(),
                                                          std::move(mw_handle),
                                                          std::move(parameters),
                                                          std::move(logger),
                                                          std::move(tf_broadcaster),
                                                          std::move(timer));
}

} // namespace spot_ros2::point_cloud