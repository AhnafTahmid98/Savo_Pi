#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "savo_head/core/camera_health.hpp"
#include "savo_head/core/head_types.hpp"

namespace savo_head
{

namespace
{

std::chrono::nanoseconds seconds_to_period(double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(std::max(0.001, seconds)));
}

diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  bool value)
{
  return make_key_value(
    key,
    std::string(value ? "true" : "false"));
}

diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  double value)
{
  std::ostringstream stream;
  stream << value;
  return make_key_value(key, stream.str());
}

diagnostic_msgs::msg::KeyValue make_key_value(
  const std::string & key,
  std::uint64_t value)
{
  return make_key_value(key, std::to_string(value));
}

std::uint8_t diagnostic_level(CameraHealthLevel level)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  switch (level) {
    case CameraHealthLevel::kOk:
      return DiagnosticStatus::OK;
    case CameraHealthLevel::kWarn:
      return DiagnosticStatus::WARN;
    case CameraHealthLevel::kError:
      return DiagnosticStatus::ERROR;
  }

  return DiagnosticStatus::ERROR;
}

std::int64_t stamp_nanoseconds(
  const builtin_interfaces::msg::Time & stamp)
{
  return rclcpp::Time(stamp).nanoseconds();
}

bool camera_info_is_calibrated(
  const sensor_msgs::msg::CameraInfo & info)
{
  const auto fx = info.k[0];
  const auto fy = info.k[4];
  return std::isfinite(fx) && std::isfinite(fy) && fx > 0.0 && fy > 0.0;
}

}  // namespace

class HeadCameraStatusNode : public rclcpp::Node
{
public:
  HeadCameraStatusNode()
  : Node("head_camera_status_node")
  {
    declare_parameters();
    load_configuration();
    start_time_s_ = now_s();

    /*
     * ROS 2 gscam constructs CameraInfo for every pulled frame and publishes
     * it with the Image under the same header. Monitoring CameraInfo preserves
     * frame cadence/freshness evidence without receiving pixel buffers.
     */
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_,
      rclcpp::SensorDataQoS().keep_last(5),
      std::bind(
        &HeadCameraStatusNode::on_camera_info,
        this,
        std::placeholders::_1));

    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10).best_effort());

    diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());

    health_service_ = create_service<std_srvs::srv::Trigger>(
      health_check_service_,
      std::bind(
        &HeadCameraStatusNode::on_health_check,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    status_timer_ = create_wall_timer(
      seconds_to_period(1.0 / status_publish_hz_),
      std::bind(&HeadCameraStatusNode::publish_status, this));

    RCLCPP_INFO(
      get_logger(),
      "camera status node started: image_graph=%s metadata=%s status=%s",
      image_topic_.c_str(),
      camera_info_topic_.c_str(),
      status_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("image_topic", kTopicCameraImageRaw);
    declare_parameter<std::string>("camera_info_topic", kTopicCameraInfo);
    declare_parameter<std::string>("status_topic", kTopicCameraStatus);
    declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    declare_parameter<std::string>(
      "health_check_service", "/savo_head/camera/health_check");
    declare_parameter<std::int64_t>("expected_width", 640);
    declare_parameter<std::int64_t>("expected_height", 480);
    declare_parameter<std::string>("expected_frame_id", kFrameCameraOptical);
    declare_parameter<std::string>("expected_encoding", "rgb8");
    declare_parameter<double>("startup_grace_s", 3.0);
    declare_parameter<double>("metadata_stale_timeout_s", 2.0);
    declare_parameter<double>("min_frame_rate_hz", 10.0);
    declare_parameter<std::int64_t>("min_frame_samples", 5);
    declare_parameter<double>("frame_rate_ema_alpha", 0.20);
    declare_parameter<bool>("require_calibration_for_pose", true);
    declare_parameter<bool>("strict_frame_id", true);
    declare_parameter<bool>("strict_resolution", true);
    declare_parameter<double>("status_publish_hz", 2.0);
  }

  void load_configuration()
  {
    image_topic_ = get_parameter("image_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();
    health_check_service_ = get_parameter("health_check_service").as_string();

    if (
      image_topic_.empty() || camera_info_topic_.empty() ||
      status_topic_.empty() || diagnostics_topic_.empty() ||
      health_check_service_.empty())
    {
      throw std::runtime_error("camera topic and service names must not be empty");
    }

    const auto expected_width = get_parameter("expected_width").as_int();
    const auto expected_height = get_parameter("expected_height").as_int();
    const auto min_frame_samples = get_parameter("min_frame_samples").as_int();

    if (
      expected_width <= 0 ||
      expected_width > static_cast<std::int64_t>(
        std::numeric_limits<std::uint32_t>::max()))
    {
      throw std::runtime_error("expected_width is out of range");
    }

    if (
      expected_height <= 0 ||
      expected_height > static_cast<std::int64_t>(
        std::numeric_limits<std::uint32_t>::max()))
    {
      throw std::runtime_error("expected_height is out of range");
    }

    if (min_frame_samples <= 0) {
      throw std::runtime_error("min_frame_samples must be positive");
    }

    config_.expected_width = static_cast<std::uint32_t>(expected_width);
    config_.expected_height = static_cast<std::uint32_t>(expected_height);
    config_.expected_frame_id = get_parameter("expected_frame_id").as_string();
    config_.expected_encoding = get_parameter("expected_encoding").as_string();
    config_.startup_grace_s = get_parameter("startup_grace_s").as_double();
    config_.metadata_stale_timeout_s =
      get_parameter("metadata_stale_timeout_s").as_double();
    config_.min_frame_rate_hz = get_parameter("min_frame_rate_hz").as_double();
    config_.min_frame_samples = static_cast<std::uint64_t>(min_frame_samples);
    config_.require_calibration_for_pose =
      get_parameter("require_calibration_for_pose").as_bool();
    config_.strict_frame_id = get_parameter("strict_frame_id").as_bool();
    config_.strict_resolution = get_parameter("strict_resolution").as_bool();
    frame_rate_ema_alpha_ = get_parameter("frame_rate_ema_alpha").as_double();
    status_publish_hz_ = get_parameter("status_publish_hz").as_double();

    if (
      !std::isfinite(frame_rate_ema_alpha_) ||
      frame_rate_ema_alpha_ <= 0.0 || frame_rate_ema_alpha_ > 1.0)
    {
      throw std::runtime_error("frame_rate_ema_alpha must be in the range (0.0, 1.0]");
    }

    if (!std::isfinite(status_publish_hz_) || status_publish_hz_ <= 0.0) {
      throw std::runtime_error("status_publish_hz must be positive");
    }

    const auto errors = config_.validation_errors();
    if (!errors.empty()) {
      std::ostringstream stream;
      stream << "invalid camera health configuration:";
      for (const auto & error : errors) {
        stream << " " << error << ";";
      }
      throw std::runtime_error(stream.str());
    }
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    const auto receipt_time_s = now_s();
    const auto message_stamp_ns = stamp_nanoseconds(msg->header.stamp);

    if (
      message_stamp_ns <= 0 ||
      (last_metadata_stamp_ns_.has_value() &&
      message_stamp_ns <= last_metadata_stamp_ns_.value()))
    {
      metadata_timestamp_monotonic_ = false;
    }
    last_metadata_stamp_ns_ = message_stamp_ns;

    if (stream_metadata_seen_) {
      const auto delta_s = receipt_time_s - metadata_receipt_time_s_;
      if (std::isfinite(delta_s) && delta_s > 0.0) {
        const auto instantaneous_hz = 1.0 / delta_s;
        if (!frame_rate_initialized_ || frame_rate_hz_ <= 0.0) {
          frame_rate_hz_ = instantaneous_hz;
          frame_rate_initialized_ = true;
        } else {
          frame_rate_hz_ =
            frame_rate_ema_alpha_ * instantaneous_hz +
            (1.0 - frame_rate_ema_alpha_) * frame_rate_hz_;
        }
      }
    }

    stream_metadata_seen_ = true;
    metadata_receipt_time_s_ = receipt_time_s;
    metadata_width_ = msg->width;
    metadata_height_ = msg->height;
    metadata_frame_id_ = msg->header.frame_id;
    camera_calibrated_ = camera_info_is_calibrated(*msg);
    ++frames_received_;
  }

  CameraHealthSnapshot current_snapshot() const
  {
    CameraHealthSnapshot snapshot;
    snapshot.start_time_s = start_time_s_;
    snapshot.now_s = now_s();
    snapshot.image_publisher_present = count_publishers(image_topic_) > 0U;
    snapshot.stream_metadata_seen = stream_metadata_seen_;
    snapshot.metadata_receipt_time_s = metadata_receipt_time_s_;
    snapshot.metadata_timestamp_monotonic = metadata_timestamp_monotonic_;
    snapshot.metadata_width = metadata_width_;
    snapshot.metadata_height = metadata_height_;
    snapshot.metadata_frame_id = metadata_frame_id_;
    snapshot.camera_calibrated = camera_calibrated_;
    snapshot.frames_received = frames_received_;
    snapshot.frame_rate_hz = frame_rate_hz_;
    return snapshot;
  }

  void on_health_check(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    const auto snapshot = current_snapshot();
    const auto result = evaluate_camera_health(config_, snapshot);
    response->success = result.stream_healthy;
    response->message = camera_health_status_text(config_, result, snapshot);
  }

  void publish_status()
  {
    const auto snapshot = current_snapshot();
    const auto result = evaluate_camera_health(config_, snapshot);

    std_msgs::msg::String status_message;
    status_message.data = camera_health_status_text(config_, result, snapshot);
    status_pub_->publish(status_message);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "savo_head.camera";
    status.hardware_id = "pi_camera_v2_noir";
    status.level = diagnostic_level(result.level);
    status.message = result.reason;

    const auto metadata_age_s = snapshot.stream_metadata_seen ?
      std::max(0.0, snapshot.now_s - snapshot.metadata_receipt_time_s) : 0.0;

    status.values = {
      make_key_value("status", result.status),
      make_key_value("reason", result.reason),
      make_key_value("stream_healthy", result.stream_healthy),
      make_key_value("ready_for_pose_estimation", result.ready_for_pose_estimation),
      make_key_value("image_publisher_present", snapshot.image_publisher_present),
      make_key_value("stream_metadata_seen", snapshot.stream_metadata_seen),
      make_key_value("camera_info_seen", snapshot.stream_metadata_seen),
      make_key_value("camera_calibrated", snapshot.camera_calibrated),
      make_key_value(
        "metadata_timestamp_monotonic", snapshot.metadata_timestamp_monotonic),
      make_key_value(
        "camera_info_timestamp_monotonic", snapshot.metadata_timestamp_monotonic),
      make_key_value("metadata_age_s", metadata_age_s),
      make_key_value("camera_info_age_s", metadata_age_s),
      make_key_value("frame_rate_hz", snapshot.frame_rate_hz),
      make_key_value("frames_received", snapshot.frames_received),
      make_key_value("metadata_frame_id", snapshot.metadata_frame_id),
      make_key_value("camera_info_frame_id", snapshot.metadata_frame_id),
      make_key_value(
        "metadata_resolution",
        std::to_string(snapshot.metadata_width) + "x" +
        std::to_string(snapshot.metadata_height)),
      make_key_value(
        "camera_info_resolution",
        std::to_string(snapshot.metadata_width) + "x" +
        std::to_string(snapshot.metadata_height)),
      make_key_value("configured_encoding", config_.expected_encoding),
      make_key_value("encoding_verification", std::string("static_config"))
    };

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status = {status};
    diagnostics_pub_->publish(array);
  }

  double now_s() const
  {
    return now().seconds();
  }

  CameraHealthConfig config_{};

  std::string image_topic_{};
  std::string camera_info_topic_{};
  std::string status_topic_{};
  std::string diagnostics_topic_{};
  std::string health_check_service_{};

  double frame_rate_ema_alpha_{0.20};
  double status_publish_hz_{2.0};
  double start_time_s_{0.0};
  double metadata_receipt_time_s_{0.0};
  double frame_rate_hz_{0.0};

  bool stream_metadata_seen_{false};
  bool metadata_timestamp_monotonic_{true};
  bool camera_calibrated_{false};
  bool frame_rate_initialized_{false};

  std::uint32_t metadata_width_{0U};
  std::uint32_t metadata_height_{0U};
  std::string metadata_frame_id_{};
  std::uint64_t frames_received_{0U};
  std::optional<std::int64_t> last_metadata_stamp_ns_{};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
  diagnostics_pub_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr health_service_{};
  rclcpp::TimerBase::SharedPtr status_timer_{};
};

}  // namespace savo_head

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<savo_head::HeadCameraStatusNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "head_camera_status_node failed: " << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
