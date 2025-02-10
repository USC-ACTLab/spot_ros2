/**
 * @file spot_point_cloud_middleware_handle.cpp
 * @brief implements spot point cloud middleware handle
 * @author Will Wu, ACT Lab @ Brown University
 */

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spot_driver/point_cloud/point_cloud_middleware_handle.hpp>

namespace spot_ros2::point_cloud {

PointCloudMiddlewareHandle::PointCloudMiddlewareHandle(const std::shared_ptr<rclcpp::Node>& node)
    : node_(node) {
    point_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lidar/points", makePublisherQoS(kPublisherHistoryDepth));
}

PointCloudMiddlewareHandle::PointCloudMiddlewareHandle(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions{})
    : PointCloudMiddlewareHandle(std::make_shared<rclcpp::Node>("point_cloud_publisher", node_options)) {}

tl::expected<void, std::string> PointCloudMiddlewareHandle::publishPointCloud(const sensor_msgs::msg::PointCloud2& msg) {
    if (point_cloud_publisher_ == nullptr) {
        return tl::make_unexpected("Failed to create point cloud publisher");
    }
    point_cloud_publisher->publish(msg);
}
} // namespace spot_ros2::point_cloud