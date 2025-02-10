/**
 * @file point_cloud_middleware_handle.hpp
 * @brief defines point cloud middleware handle implementation
 * @author Will Wu, ACT Lab @ Brown University
 */

#pragma once

#include <memory>
#include <rclcpp/node.hpp>
#include <spot_driver/point_cloud/spot_point_cloud_publisher.hpp>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <tl_expected/expected.hpp>

namespace {
constexpr auto kPublisherHistoryDepth = 10;
}

namespace spot_ros2::point_cloud {

/**
 * Implementation of SpotPointCloudPublisher::MiddlewareHandle
 */
class PointCloudMiddlewareHandle: public SpotPointCloudPublisher::MiddlewareHandle {

public:
    /**
     * @brief Constructor for PointCloudMiddlewareHandle
     *
     * @param node  A shared_ptr to an instance of a rclcpp::Node
     */
    explicit PointCloudMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node);

    /**
     * @brief Constructor for PointCloudMiddlewareHandle which creates an instance of an rclcpp::node
     *
     * @param node_options Options for rclcpp::node initialization
     */
    explicit PointCloudMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{});

    // publishes aquired message through point cloud publishers
    tl::expected<void, std::string> publishPointCloud(const sensor_msgs::msg::PointCloud2& msg) override;

private:
    // Shared instance of an rclcpp node to create publishers 
    std::shared_ptr<rclcpp::Node> node_;

    // Point cloud publisher
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> point_cloud_publisher_;
};
} // namespace spot_ros2::point_cloud