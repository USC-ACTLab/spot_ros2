/**
 * @file default_point_cloud_client.cpp
 * @brief Use SPOT SDK to receive and parse point cloud messages
 * @author Will Wu, ACT Lab @ Brown University
 */

#include <spot_driver/api/default_point_cloud_client.hpp>

#include <google/protobuf/duration.pb.h>
#include <bosdyn/api/point_cloud.pb.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spot_driver/api/default_time_sync_api.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/time.hpp>
#include <spot_driver/types.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace {
std_msgs::msg::Header createPointCloudHeader(const bosdyn::api::PointCloudSource,
                                             const std::string robot_name,
                                             const google::protobuf::Duration& clock_skew) {

}
} // namespace

namespace spot_ros2 {

} // namespace spot_ros2