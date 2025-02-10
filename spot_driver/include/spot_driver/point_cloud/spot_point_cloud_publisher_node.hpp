/**
 * @file spot_point_cloud_publisher_node.hpp
 * @brief defines point cloud publisher ros node
 * @author Will Wu, ACT Lab @ Brown University
 */

#pragma once

#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/node_options.hpp>
#include <spot_driver/api/spot_api.hpp>
#include <spot_driver/point_cloud/spot_point_cloud_publisher.hpp>
#include <spot_driver/interfaces/logger_interface_base.hpp>
#include <spot_driver/interfaces/node_interface_base.hpp>
#include <spot_driver/interfaces/parameter_interface_base.hpp>
#include <spot_driver/interfaces/tf_broadcaster_interface_base.hpp>
#include <spot_driver/interfaces/timer_interface_base.hpp>

namespace spot_ros2::point_cloud {
/**
 * @brief Wraps SpotPointCloudPublisher to allow using it like a rclcpp::Node.
 */
class SpotPointCloudPublisherNode {
public:
    SpotPointCloudPublisherNode(std::unique_ptr<SpotApi> spot_api,
                                std::unique_ptr<SpotPointCloudPublisher::MiddlewareHandle> mw_handle,
                                std::unique_ptr<ParameterInterfaceBase> parameters,
                                std::unique_ptr<LoggerInterfaceBase> logger,
                                std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                                std::unique_ptr<TimerInterfaceBase> timer,
                                std::unique_ptr<NodeInterfaceBase> node_base_interface);

    explicit SpotPointCloudPublisherNode(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

    /**
     * @brief Returns the NodeBaseInterface of this class's node.
     * @details This function exists to allow spinning the class's node as if it were derived from rclcpp::Node.
     * This allows loading this class as a component node in a composable node container.
     *
     * @return A shared_ptr to the NodeBaseInterface of the node stored as a private member of this class.
     */
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> get_node_base_interface();

private:
    void initialize(std::unique_ptr<SpotApi> spot_api,
                    std::unique_ptr<SpotPointCloudPublisher::MiddlewareHandle> mw_handle,
                    std::unique_ptr<ParameterInterfaceBase> parameters,
                    std::unique_ptr<LoggerInterfaceBase> logger,
                    std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                    std::unique_ptr<TimerInterfaceBase> timer);

    std::unique_ptr<NodeInterfaceBase> node_base_interface_;
    std::unique_ptr<SpotApi> spot_api_;
    std::unique_ptr<SpotPointCloudPublisher> internal_;
};

} // namespace spot_ros2::point_cloud