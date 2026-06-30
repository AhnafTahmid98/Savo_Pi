#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_realsense/camera_monitor_common.hpp"

namespace
{

constexpr const char * COLOR_IMAGE_TOPIC = "/camera/camera/color/image_raw";
constexpr const char * COLOR_INFO_TOPIC = "/camera/camera/color/camera_info";
constexpr const char * DEPTH_IMAGE_TOPIC = "/camera/camera/depth/image_rect_raw";
constexpr const char * DEPTH_INFO_TOPIC = "/camera/camera/depth/camera_info";
constexpr const char * POINTCLOUD_TOPIC = "/camera/camera/depth/color/points";
constexpr const char * STATUS_TOPIC = "/realsense/status";
constexpr const char * DIAGNOSTICS_TOPIC = "/diagnostics";

class CameraHealthNode : public rclcpp::Node
{
public:
  CameraHealthNode()
  : Node("camera_health_node")
  {
    status_hz_ = declare_parameter<double>("status_hz", 2.0);
    params_.stale_timeout_s = declare_parameter<double>("stale_timeout_s", 0.75);
    params_.expected_color_hz = declare_parameter<double>("expected_color_hz", 20.0);
    params_.expected_depth_hz = declare_parameter<double>("expected_depth_hz", 20.0);
    params_.expected_camera_info_hz = declare_parameter<double>("expected_camera_info_hz", 20.0);
    params_.expected_pointcloud_hz = declare_parameter<double>("expected_pointcloud_hz", 8.0);
    require_pointcloud_ = declare_parameter<bool>("require_pointcloud", false);

    if (status_hz_ <= 0.0) {
      status_hz_ = 2.0;
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

    status_pub_ = create_publisher<std_msgs::msg::String>(STATUS_TOPIC, reliable_qos);
    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(DIAGNOSTICS_TOPIC, reliable_qos);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_hz_),
      [this]() { publish_status(); });
  }

private:
  void publish_status()
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

    const bool pointcloud_ok = require_pointcloud_ ? pointcloud.ok() : true;
    const bool ok = color.ok() && color_info.ok() && depth.ok() && depth_info.ok() && pointcloud_ok;

    std_msgs::msg::String message;
    message.data = make_status_json(ok, color, color_info, depth, depth_info, pointcloud);
    status_pub_->publish(message);

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

  std::string make_status_json(
    bool ok,
    const savo_realsense::StreamStatus & color,
    const savo_realsense::StreamStatus & color_info,
    const savo_realsense::StreamStatus & depth,
    const savo_realsense::StreamStatus & depth_info,
    const savo_realsense::StreamStatus & pointcloud) const
  {
    std::ostringstream stream;
    stream
      << "{"
      << "\"ok\":" << json_bool(ok)
      << ",\"message\":\"" << status_message(ok, color, color_info, depth, depth_info, pointcloud)
      << "\""
      << ",\"color_ok\":" << json_bool(color.ok())
      << ",\"color_info_ok\":" << json_bool(color_info.ok())
      << ",\"depth_ok\":" << json_bool(depth.ok())
      << ",\"depth_info_ok\":" << json_bool(depth_info.ok())
      << ",\"pointcloud_ok\":" << json_bool(require_pointcloud_ ? pointcloud.ok() : true)
      << ",\"require_pointcloud\":" << json_bool(require_pointcloud_)
      << "}";

    return stream.str();
  }

  std::string status_message(
    bool ok,
    const savo_realsense::StreamStatus & color,
    const savo_realsense::StreamStatus & color_info,
    const savo_realsense::StreamStatus & depth,
    const savo_realsense::StreamStatus & depth_info,
    const savo_realsense::StreamStatus & pointcloud) const
  {
    if (ok) {
      return "RealSense streams OK";
    }

    std::vector<std::string> bad;
    if (!color.ok()) {
      bad.push_back("color");
    }
    if (!color_info.ok()) {
      bad.push_back("color_info");
    }
    if (!depth.ok()) {
      bad.push_back("depth");
    }
    if (!depth_info.ok()) {
      bad.push_back("depth_info");
    }
    if (require_pointcloud_ && !pointcloud.ok()) {
      bad.push_back("pointcloud");
    }

    if (bad.empty()) {
      return "No RealSense streams detected";
    }

    std::ostringstream stream;
    stream << "Unhealthy streams: ";
    for (std::size_t i = 0; i < bad.size(); ++i) {
      if (i > 0) {
        stream << ", ";
      }
      stream << bad[i];
    }
    return stream.str();
  }

  static const char * json_bool(bool value)
  {
    return value ? "true" : "false";
  }

  double status_hz_{2.0};
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

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraHealthNode>());
  rclcpp::shutdown();
  return 0;
}
