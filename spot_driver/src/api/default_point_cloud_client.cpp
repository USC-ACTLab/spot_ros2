/**
 * @file default_point_cloud_client.cpp
 * @brief Use SPOT SDK to receive and parse point cloud messages
 * @author Will Wu, ACT Lab @ Brown University
 */

#include <spot_driver/api/default_point_cloud_client.hpp>

#include <bosdyn/api/point_cloud.pb.h>
#include <google/protobuf/duration.pb.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <spot_driver/api/default_time_sync_api.hpp>
#include <spot_driver/conversions/geometry.hpp>
#include <spot_driver/conversions/time.hpp>
#include <spot_driver/types.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tl_expected/expected.hpp>

namespace {
std_msgs::msg::Header createPointCloudHeader(const ::bosdyn::api::PointCloudSource point_cloud_source,
                                             const std::string robot_name,
                                             const google::protobuf::Duration& clock_skew) {
  std_msgs::msg::Header header;
  header.frame_id = "velodyne";
  //   header.frame_id = (robot_name.empty() ? "" : robot_name + "/") + point_cloud_source.frame_name_sensor();
  header.stamp = spot_ros2::robotTimeToLocalTime(point_cloud_source.acquisition_time(), clock_skew);
  return header;
}

tl::expected<sensor_msgs::msg::PointCloud2, std::string> toPointCloud2Msg(
    const ::bosdyn::api::PointCloud& point_cloud, const std::string& robot_name,
    const google::protobuf::Duration& clock_skew) {
  sensor_msgs::msg::PointCloud2 message_out;
  message_out.header = createPointCloudHeader(point_cloud.source(), robot_name, clock_skew);
  message_out.width = point_cloud.num_points();
  message_out.height = 1;
  message_out.is_dense = true;
  sensor_msgs::PointCloud2Modifier pc_modifier(message_out);
  switch (point_cloud.encoding()) {
    case ::bosdyn::api::PointCloud_Encoding_ENCODING_XYZ_32F:
      message_out.is_bigendian = false;
      pc_modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                       sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                       sensor_msgs::msg::PointField::FLOAT32);
      message_out.data = std::vector<uint8_t>(point_cloud.data().begin(), point_cloud.data().end());
      break;
    default:
      return tl::make_unexpected("Unsupported point cloud encoding format");
  }
  return message_out;
}
}  // namespace

namespace spot_ros2 {
DefaultPointCloudClient::DefaultPointCloudClient(::bosdyn::client::PointCloudClient* pc_client,
                                                 std::shared_ptr<TimeSyncApi> time_sync_api, std::string& robot_name)
    : pc_client_(pc_client), time_sync_api_(time_sync_api), robot_name_(robot_name) {}

tl::expected<sensor_msgs::msg::PointCloud2, std::string> DefaultPointCloudClient::getPointCloud(
    ::bosdyn::api::GetPointCloudRequest request) {
  std::shared_future<::bosdyn::client::GetPointCloudResultType> get_point_cloud_result_future =
      pc_client_->GetPointCloudAsync(request);
  ::bosdyn::client::GetPointCloudResultType get_point_cloud_result = get_point_cloud_result_future.get();
  if (!get_point_cloud_result.status) {
    return tl::make_unexpected("Failed to get point cloud from LiDAR" + get_point_cloud_result.status.DebugString());
  }

  const auto clock_skew_result = time_sync_api_->getClockSkew();
  if (!clock_skew_result) {
    return tl::make_unexpected("Failed to get latest clock skew: " + clock_skew_result.error());
  }

  for (const auto& point_cloud_response : get_point_cloud_result.response.point_cloud_responses()) {
    if (point_cloud_response.status() != ::bosdyn::api::PointCloudResponse_Status_STATUS_OK) {
      return tl::make_unexpected("Responded point cloud data not valid");
    }

    const auto& source_label = point_cloud_response.point_cloud().source().name();
    if (source_label.compare(eap_point_cloud_source) == 0) {
      const auto out = toPointCloud2Msg(point_cloud_response.point_cloud(), robot_name_, clock_skew_result.value());
      if (!out) {
        return tl::make_unexpected("Failed to convert SDK point cloud to ROS Pointcloud message " + out.error());
      }
      return out.value();
    }
  }

  return tl::make_unexpected("Cannot find point cloud from EAP Lidar");
}
}  // namespace spot_ros2
