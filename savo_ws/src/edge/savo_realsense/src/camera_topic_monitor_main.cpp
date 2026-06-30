#include <memory>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "savo_realsense/camera_monitor_common.hpp"

namespace
{

constexpr const char * COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw";
constexpr const char * COLOR_INFO_TOPIC = "/camera/camera/color/camera_info";
constexpr const char * DEPTH_IMAGE_TOPIC = "/camera/camera/depth/image_rect_raw";
constexpr const char * DEPTH_INFO_TOPIC = "/camera/camera/depth/camera_info";
constexpr const char * POINTCLOUD_TOPIC = "/camera/camera/depth/color/points";
constexpr const char * DIAGNOSTICS_TOPIC = "/diagnostics";

class CameraTopicMonitorNode : public rclcpp::Node
{
public:
  CameraTopicMonitorNode()
  : Node("camera_topic_monitor_node")
  {
    publish_hz_ = declare_parameter<double>("publish_hz", 2.0);
    params_.stale_timeout_s = declare_parameter<double>("stale_timeout_s", 0.75);
    params_.expected_color_hz = declare_parameter<double>("expected_color_hz", 20.0);
    params_.expected_depth_hz = declare_parameter<double>("expected_depth_hz", 20.0);
    params_.expected_camera_info_hz = declare_parameter<double>("expected_camera_info_hz", 20.0);
    params_.expected_pointcloud_hz = declare_parameter<double>("expected_pointcloud_hz", 8.0);
    require_pointcloud_ = declare_parameter<bool>("require_pointcloud", false);

    if (publish_hz_ <= 0.0) {
      publish_hz_ = 2.0;
    }

    auto sensor_qos = rclcpp::SensorDataQoS();
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      COLOR_IMAGE_TOPIC, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr) { color_tracker_.tick(now()); });

    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      COLOR_INFO_TOPIC, reliable_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) { color_info_tracker_.tick(now()); });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      DEPTH_IMAGE_TOPIC, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr) { depth_tracker_.tick(now()); });

    depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      DEPTH_INFO_TOPIC, reliable_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr) { depth_info_tracker_.tick(now()); });

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      POINTCLOUD_TOPIC, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr) { pointcloud_tracker_.tick(now()); });

    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(DIAGNOSTICS_TOPIC, reliable_qos);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() { publish_diagnostics(); });
  }

private:
  void publish_diagnostics()
  {
    const auto now_time = now();

    const auto color = savo_realsense::build_stream_status(
      COLOR_IMAGE_TOPIC, color_tracker_, now_time,
      params_.expected_color_hz, params_.stale_timeout_s);
    const auto color_info = savo_realsense::build_stream_status(
      COLOR_INFO_TOPIC, color_info_tracker_, now_time,
      params_.expected_camera_info_hz, params_.stale_timeout_s);
    const auto depth = savo_realsense::build_stream_status(
      DEPTH_IMAGE_TOPIC, depth_tracker_, now_time,
      params_.expected_depth_hz, params_.stale_timeout_s);
    const auto depth_info = savo_realsense::build_stream_status(
      DEPTH_INFO_TOPIC, depth_info_tracker_, now_time,
      params_.expected_camera_info_hz, params_.stale_timeout_s);
    const auto pointcloud = savo_realsense::build_stream_status(
      POINTCLOUD_TOPIC, pointcloud_tracker_, now_time,
      params_.expected_pointcloud_hz, params_.stale_timeout_s);

    std::vector<diagnostic_msgs::msg::DiagnosticStatus> diagnostics;
    diagnostics.push_back(savo_realsense::make_stream_diagnostic("RealSense color image", color));
    diagnostics.push_back(
      savo_realsense::make_stream_diagnostic("RealSense color camera info", color_info));
    diagnostics.push_back(savo_realsense::make_stream_diagnostic("RealSense depth image", depth));
    diagnostics.push_back(
      savo_realsense::make_stream_diagnostic("RealSense depth camera info", depth_info));

    if (require_pointcloud_ || pointcloud.seen) {
      diagnostics.push_back(
        savo_realsense::make_stream_diagnostic("RealSense pointcloud", pointcloud));
    }

    diagnostics_pub_->publish(savo_realsense::make_diagnostic_array(diagnostics, now_time));
  }

  double publish_hz_{2.0};
  bool require_pointcloud_{false};
  savo_realsense::StreamMonitorParams params_;

  savo_realsense::RateTracker color_tracker_;
  savo_realsense::RateTracker color_info_tracker_;
  savo_realsense::RateTracker depth_tracker_;
  savo_realsense::RateTracker depth_info_tracker_;
  savo_realsense::RateTracker pointcloud_tracker_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraTopicMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
