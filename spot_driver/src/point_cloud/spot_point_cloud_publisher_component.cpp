/**
 * @file spot_point_cloud_publisher_component.cpp
 * @brief Register point cloud publisher as a ROS node
 * @author Will Wu, ACT Lab @ Brown University
 */

#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <spot_driver/point_cloud/spot_point_cloud_publisher_node.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(spot_ros2::point_cloud::SpotPointCloudPublisherNode)