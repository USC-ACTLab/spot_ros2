/**
 * @file spot_point_cloud_publisher.cpp
 * @brief implements internal publisher to connect with Spot and read/publish point cloud data
 * @author Will Wu, ACT Lab @ Brown University
 */
#include <chrono>

#include <bosdyn/api/point_cloud.pb.h>
#include <spot_driver/interfaces/rclcpp_logger_interface.hpp>
#include <spot_driver/interfaces/rclcpp_parameter_interface.hpp>
#include <spot_driver/interfaces/rclcpp_tf_broadcaster_interface.hpp>
#include <spot_driver/interfaces/rclcpp_wall_timer_interface.hpp>
#include <spot_driver/point_cloud/spot_point_cloud_publisher.hpp>
#include <spot_driver/point_cloud/point_cloud_middleware_handle.hpp>
#include <spot_driver/types.hpp>

namespace {
    constexpr auto kPointCloudPublisherPeriod_HZ = std::chrono::duration<double>(1.0 / 15.0);
}

namespace spot_ros2::point_cloud {

SpotPointCloudPublisher::SpotPointCloudPublisher(const std::shared_ptr<PointCloudClientInterface>& pc_client_interface,
                                                 std::unique_ptr<MiddlewareHandle> middleware_handle,
                                                 std::unique_ptr<ParameterInterfaceBase> parameters,
                                                 std::unique_ptr<LoggerInterfaceBase> logger,
                                                 std::unique_ptr<TfBroadcasterInterfaceBase> tf_broadcaster,
                                                 std::unique_ptr<TimerInterfaceBase> timer)
    : pc_client_interface_(pc_client_interface),
      middleware_handle_(middleware_handle),
      parameters_(parameters),
      logger_(logger),
      tf_broadcaster_(tf_broadcaster),
      timer_(timer) {
    createVLPPointCloudRequest();
    timer_->setTimer(kPointCloudPublisherPeriod_HZ, [this]() {timerCallback()});
}

void SpotPointCloudPublisher::timerCallback() {
    if (!point_cloud_request_msg_) {
        logger_->logError("No point cloud request message generated.");
        return;
    }

    const auto point_cloud_result = pc_client_->getPointCloud(point_cloud_request_msg_.value());
    if (!point_cloud_result.has_value()) {
        logger_logError(std::string{"Failed to get point cloud from robot: "}.append(image_result.error()));
        return;
    }

    middleware_handle_->publishPointCloud(point_cloud_result.value());
}

void SpotPointCloudPublisher::createVLPPointCloudRequest() {
    ::bosdyn::api::GetPointCloudRequest point_cloud_request_msg;
    ::bosdyn::api::PointCloudRequest *pc_request = point_cloud_request_msg.add_point_cloud_requests();
    pc_request->set_point_cloud_source_name(eap_point_cloud_source);
    point_cloud_request_msg_ = point_cloud_request_msg;
}

} // namespace spot_ros2::point_cloud