/**
 * @file spot_point_cloud_publisher_node_main.cpp
 * @brief spins point cloud publisher ros node
 * @author Will Wu, ACT Lab @ Brown University
 */

#include <rclcpp/executors.hpp>
#include <rclcpp/utilities.hpp>
#include <spot_driver/point_cloud/spot_point_cloud_publisher_node.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<spot_ros2::point_cloud::SpotPointCloudPublisherNode>();

  // Spins the node with the default single-threaded executor.
  rclcpp::spin(node->get_node_base_interface());

  return 0;
}