// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_localization/localization_health_core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace savo_localization
{
namespace
{

constexpr std::size_t kRateWindowMinimum{3U};
constexpr std::size_t kMaxReasonCount{16U};
constexpr double kPi{3.14159265358979323846};

bool finite(const double value)
{
  return std::isfinite(value);
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!finite(quaternion.x) || !finite(quaternion.y) ||
    !finite(quaternion.z) || !finite(quaternion.w))
  {
    return false;
  }

  const double norm_sq =
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;

  return norm_sq > 1.0e-8 && finite(norm_sq);
}

template<typename ContainerT>
bool all_finite(const ContainerT & values)
{
  return std::all_of(
    values.begin(), values.end(),
    [](const auto value) {return finite(static_cast<double>(value));});
}

bool valid_imu_message(const sensor_msgs::msg::Imu & message)
{
  return finite_quaternion(message.orientation) &&
         finite(message.angular_velocity.x) &&
         finite(message.angular_velocity.y) &&
         finite(message.angular_velocity.z) &&
         finite(message.linear_acceleration.x) &&
         finite(message.linear_acceleration.y) &&
         finite(message.linear_acceleration.z) &&
         all_finite(message.orientation_covariance) &&
         all_finite(message.angular_velocity_covariance) &&
         all_finite(message.linear_acceleration_covariance);
}

bool valid_odom_message(const nav_msgs::msg::Odometry & message)
{
  return finite(message.pose.pose.position.x) &&
         finite(message.pose.pose.position.y) &&
         finite(message.pose.pose.position.z) &&
         finite_quaternion(message.pose.pose.orientation) &&
         finite(message.twist.twist.linear.x) &&
         finite(message.twist.twist.linear.y) &&
         finite(message.twist.twist.linear.z) &&
         finite(message.twist.twist.angular.x) &&
         finite(message.twist.twist.angular.y) &&
         finite(message.twist.twist.angular.z) &&
         all_finite(message.pose.covariance) &&
         all_finite(message.twist.covariance);
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double shortest_angular_distance(const double from, const double to)
{
  return std::remainder(to - from, 2.0 * kPi);
}

std::string escape_json(const std::string & input)
{
  std::ostringstream output;
  for (const unsigned char character : input) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;
      case '"':
        output << "\\\"";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20U) {
          output << "\\u"
                 << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character)
                 << std::dec << std::setfill(' ');
        } else {
          output << static_cast<char>(character);
        }
        break;
    }
  }
  return output.str();
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

std::chrono::milliseconds period_from_hz(const double hz)
{
  if (!finite(hz) || hz <= 0.0) {
    throw std::invalid_argument("timer frequency must be finite and > 0");
  }

  const auto milliseconds = static_cast<std::int64_t>(std::llround(1000.0 / hz));
  return std::chrono::milliseconds(std::max<std::int64_t>(1, milliseconds));
}

diagnostic_msgs::msg::KeyValue key_value(
  std::string key,
  std::string value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = std::move(key);
  item.value = std::move(value);
  return item;
}

int diagnostic_level(const LocalizationHealthState state)
{
  switch (state) {
    case LocalizationHealthState::kOk:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case LocalizationHealthState::kInitializing:
    case LocalizationHealthState::kDegraded:
    case LocalizationHealthState::kStale:
    case LocalizationHealthState::kUnknown:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case LocalizationHealthState::kError:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  return diagnostic_msgs::msg::DiagnosticStatus::STALE;
}

std::string join_reasons(const std::vector<std::string> & reasons)
{
  if (reasons.empty()) {
    return "none";
  }

  std::ostringstream output;
  const std::size_t count = std::min(reasons.size(), kMaxReasonCount);
  for (std::size_t index = 0; index < count; ++index) {
    if (index > 0U) {
      output << ',';
    }
    output << reasons[index];
  }
  return output.str();
}

bool contains_case_sensitive(
  const std::string & value,
  const std::string & token)
{
  return value.find(token) != std::string::npos;
}

int extract_json_integer(
  const std::string & payload,
  const std::string & key,
  const int fallback)
{
  const std::regex expression(
    "\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+)");
  std::smatch match;
  if (!std::regex_search(payload, match, expression) || match.size() < 2U) {
    return fallback;
  }

  try {
    return std::stoi(match[1].str());
  } catch (const std::exception &) {
    return fallback;
  }
}

struct RuntimeTracker
{
  bool received{false};
  bool data_valid{true};
  bool frame_valid{true};
  bool diagnostic_warning{false};
  bool diagnostic_error{false};
  std::string detail{};

  rclcpp::Time last_receive{};
  std::int64_t last_header_stamp_ns{-1};
  double timestamp_fault_until_s{0.0};
  std::deque<double> arrival_times_s{};

  void record(
    const rclcpp::Time & receive_time,
    const builtin_interfaces::msg::Time & header_stamp,
    const bool valid_data,
    const bool valid_frame,
    std::string observation_detail,
    const double timestamp_fault_hold_s,
    const std::size_t rate_window_size)
  {
    received = true;
    data_valid = valid_data;
    frame_valid = valid_frame;
    detail = std::move(observation_detail);
    last_receive = receive_time;

    const std::int64_t header_ns = rclcpp::Time(header_stamp).nanoseconds();
    if (header_ns > 0) {
      if (last_header_stamp_ns > 0 && header_ns < last_header_stamp_ns) {
        timestamp_fault_until_s = receive_time.seconds() + timestamp_fault_hold_s;
      }
      last_header_stamp_ns = header_ns;
    }

    arrival_times_s.push_back(receive_time.seconds());
    while (arrival_times_s.size() > rate_window_size) {
      arrival_times_s.pop_front();
    }
  }

  [[nodiscard]] double age_s(const rclcpp::Time & current_time) const
  {
    if (!received) {
      return -1.0;
    }
    return std::max(0.0, (current_time - last_receive).seconds());
  }

  [[nodiscard]] double rate_hz() const
  {
    if (arrival_times_s.size() < kRateWindowMinimum) {
      return 0.0;
    }

    const double duration_s = arrival_times_s.back() - arrival_times_s.front();
    if (!finite(duration_s) || duration_s <= 0.0) {
      return 0.0;
    }

    return static_cast<double>(arrival_times_s.size() - 1U) / duration_s;
  }

  [[nodiscard]] bool timestamp_valid(const rclcpp::Time & current_time) const
  {
    return current_time.seconds() >= timestamp_fault_until_s;
  }
};

struct TransformObservationResult
{
  bool available{false};
  bool fresh{false};
  double age_s{-1.0};
  std::string detail{};
};

}  // namespace

class LocalizationHealthNode final : public rclcpp::Node
{
public:
  LocalizationHealthNode()
  : Node("localization_health_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    start_time_(now())
  {
    declare_and_load_parameters();
    create_publishers();
    create_subscriptions();

    health_timer_ = create_wall_timer(
      period_from_hz(publish_rate_hz_),
      [this]() {evaluate_and_publish();});

    heartbeat_timer_ = create_wall_timer(
      period_from_hz(heartbeat_rate_hz_),
      [this]() {publish_heartbeat();});

    evaluate_and_publish();

    RCLCPP_INFO(
      get_logger(),
      "C++ localization health node ready | base=%s | imu=%s | use_vo=%s | vo_required=%s",
      base_frame_id_.c_str(),
      imu_frame_id_.c_str(),
      bool_text(use_vo_),
      bool_text(vo_required_));
  }

private:
  void declare_and_load_parameters()
  {
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 1.0);
    startup_grace_s_ = declare_parameter<double>("startup_grace_s", 3.0);
    timestamp_fault_hold_s_ = declare_parameter<double>("timestamp_fault_hold_s", 2.0);

    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    imu_state_topic_ = declare_parameter<std::string>(
      "imu_state_topic", "/savo_localization/imu_state");
    wheel_odom_topic_ = declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom");
    wheel_odom_state_topic_ = declare_parameter<std::string>(
      "wheel_odom_state_topic", "/savo_localization/wheel_odom_state");
    filtered_odom_topic_ = declare_parameter<std::string>(
      "filtered_odom_topic", "/odometry/filtered");
    vo_odom_topic_ = declare_parameter<std::string>("vo_odom_topic", "/vo/odom");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "/diagnostics");

    health_topic_ = declare_parameter<std::string>(
      "health_topic", "/savo_localization/health");
    state_summary_topic_ = declare_parameter<std::string>(
      "state_summary_topic", "/savo_localization/state_summary");
    heartbeat_topic_ = declare_parameter<std::string>(
      "heartbeat_topic", "/savo_localization/heartbeat");

    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");

    use_imu_ = declare_parameter<bool>("use_imu", true);
    use_wheel_odom_ = declare_parameter<bool>("use_wheel_odom", true);
    use_ekf_ = declare_parameter<bool>("use_ekf", true);
    use_vo_ = declare_parameter<bool>("use_vo", false);
    vo_required_ = declare_parameter<bool>("vo_required", false);

    require_odom_to_base_tf_ = declare_parameter<bool>("require_odom_to_base_tf", true);
    require_base_to_imu_tf_ = declare_parameter<bool>("require_base_to_imu_tf", true);

    expected_imu_rate_hz_ = declare_parameter<double>("expected_imu_rate_hz", 25.0);
    expected_wheel_odom_rate_hz_ = declare_parameter<double>(
      "expected_wheel_odom_rate_hz", 30.0);
    expected_ekf_rate_hz_ = declare_parameter<double>("expected_ekf_rate_hz", 30.0);
    expected_vo_rate_hz_ = declare_parameter<double>("expected_vo_rate_hz", 15.0);
    rate_tolerance_ratio_ = declare_parameter<double>("rate_tolerance_ratio", 0.50);
    const int rate_window_size = declare_parameter<int>("rate_window_size", 30);
    rate_window_size_ = static_cast<std::size_t>(std::max(3, rate_window_size));

    max_imu_age_s_ = declare_parameter<double>("max_imu_age_s", 0.5);
    max_wheel_odom_age_s_ = declare_parameter<double>("max_wheel_odom_age_s", 0.5);
    max_filtered_odom_age_s_ = declare_parameter<double>("max_filtered_odom_age_s", 0.5);
    max_vo_odom_age_s_ = declare_parameter<double>("max_vo_odom_age_s", 0.5);
    max_tf_age_s_ = declare_parameter<double>("max_tf_age_s", 0.5);

    max_odom_linear_speed_mps_ = declare_parameter<double>(
      "max_odom_linear_speed_mps", 1.5);
    max_odom_angular_speed_rad_s_ = declare_parameter<double>(
      "max_odom_angular_speed_rad_s", 4.0);
    max_pose_jump_m_ = declare_parameter<double>("max_pose_jump_m", 0.50);
    max_yaw_jump_rad_ = declare_parameter<double>("max_yaw_jump_rad", 1.00);
    max_encoder_illegal_transitions_ = declare_parameter<int>(
      "max_encoder_illegal_transitions", 20);

    log_warnings_ = declare_parameter<bool>("log_warnings", true);
    log_errors_ = declare_parameter<bool>("log_errors", true);

    validate_parameters();
  }

  void validate_parameters() const
  {
    const std::array<double, 17> positive_values{
      publish_rate_hz_, heartbeat_rate_hz_, startup_grace_s_,
      timestamp_fault_hold_s_, expected_imu_rate_hz_,
      expected_wheel_odom_rate_hz_, expected_ekf_rate_hz_,
      expected_vo_rate_hz_, rate_tolerance_ratio_, max_imu_age_s_,
      max_wheel_odom_age_s_, max_filtered_odom_age_s_, max_vo_odom_age_s_,
      max_tf_age_s_, max_odom_linear_speed_mps_,
      max_odom_angular_speed_rad_s_, max_pose_jump_m_};

    for (const double value : positive_values) {
      if (!finite(value) || value <= 0.0) {
        throw std::invalid_argument("localization health numeric parameters must be finite and > 0");
      }
    }

    if (!finite(max_yaw_jump_rad_) || max_yaw_jump_rad_ <= 0.0) {
      throw std::invalid_argument("max_yaw_jump_rad must be finite and > 0");
    }
    if (rate_tolerance_ratio_ > 1.0) {
      throw std::invalid_argument("rate_tolerance_ratio must be <= 1.0");
    }
    if (max_encoder_illegal_transitions_ < 0) {
      throw std::invalid_argument("max_encoder_illegal_transitions must be >= 0");
    }

    const std::array<std::string, 13> names{
      imu_topic_, imu_state_topic_, wheel_odom_topic_, wheel_odom_state_topic_,
      filtered_odom_topic_, vo_odom_topic_, diagnostics_topic_, health_topic_,
      state_summary_topic_, heartbeat_topic_, odom_frame_id_, base_frame_id_,
      imu_frame_id_};
    for (const auto & name : names) {
      if (name.empty()) {
        throw std::invalid_argument("localization health topic and frame names cannot be empty");
      }
    }
    if (odom_frame_id_ == base_frame_id_) {
      throw std::invalid_argument("odom_frame_id and base_frame_id cannot be identical");
    }
  }

  void create_publishers()
  {
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    health_publisher_ = create_publisher<std_msgs::msg::String>(health_topic_, latched_qos);
    summary_publisher_ = create_publisher<std_msgs::msg::String>(
      state_summary_topic_, latched_qos);
    heartbeat_publisher_ = create_publisher<std_msgs::msg::String>(
      heartbeat_topic_, latched_qos);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());
  }

  void create_subscriptions()
  {
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {on_imu(*message);});

    wheel_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_wheel_odom(*message);});

    filtered_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      filtered_odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_filtered_odom(*message);});

    vo_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      vo_odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_vo_odom(*message);});

    imu_state_subscription_ = create_subscription<std_msgs::msg::String>(
      imu_state_topic_, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_imu_state(message->data);});

    wheel_state_subscription_ = create_subscription<std_msgs::msg::String>(
      wheel_odom_state_topic_, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {on_wheel_state(message->data);});

    diagnostics_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        on_diagnostics(*message);
      });
  }

  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    const auto receive_time = now();
    const bool frame_valid = message.header.frame_id == imu_frame_id_;
    imu_tracker_.record(
      receive_time,
      message.header.stamp,
      valid_imu_message(message),
      frame_valid,
      frame_valid ? std::string{} :
      "expected=" + imu_frame_id_ + ",actual=" + message.header.frame_id,
      timestamp_fault_hold_s_,
      rate_window_size_);
  }

  bool odom_speed_valid(const nav_msgs::msg::Odometry & message) const
  {
    const double linear_speed = std::hypot(
      message.twist.twist.linear.x,
      message.twist.twist.linear.y);
    return linear_speed <= max_odom_linear_speed_mps_ &&
           std::abs(message.twist.twist.angular.z) <= max_odom_angular_speed_rad_s_;
  }

  void record_odom(
    RuntimeTracker & tracker,
    const nav_msgs::msg::Odometry & message,
    const std::string & expected_parent,
    const std::string & expected_child)
  {
    const bool frame_valid =
      message.header.frame_id == expected_parent &&
      message.child_frame_id == expected_child;
    const bool data_valid = valid_odom_message(message) && odom_speed_valid(message);

    std::string detail;
    if (!frame_valid) {
      detail = "expected=" + expected_parent + "->" + expected_child +
        ",actual=" + message.header.frame_id + "->" + message.child_frame_id;
    } else if (!data_valid) {
      detail = "nonfinite_or_out_of_bounds_odometry";
    }

    tracker.record(
      now(), message.header.stamp, data_valid, frame_valid, detail,
      timestamp_fault_hold_s_, rate_window_size_);
  }

  void on_wheel_odom(const nav_msgs::msg::Odometry & message)
  {
    record_odom(wheel_tracker_, message, odom_frame_id_, base_frame_id_);
  }

  void on_filtered_odom(const nav_msgs::msg::Odometry & message)
  {
    const auto receive_time = now();
    record_odom(filtered_tracker_, message, odom_frame_id_, base_frame_id_);

    const double x = message.pose.pose.position.x;
    const double y = message.pose.pose.position.y;
    const double yaw = yaw_from_quaternion(message.pose.pose.orientation);

    if (filtered_pose_received_ && valid_odom_message(message)) {
      const double dt_s = std::max(0.0, (receive_time - previous_filtered_receive_).seconds());
      if (dt_s > 0.0 && dt_s <= 1.0) {
        const double distance = std::hypot(x - previous_filtered_x_, y - previous_filtered_y_);
        if (distance > max_pose_jump_m_) {
          pose_jump_fault_until_s_ = receive_time.seconds() + timestamp_fault_hold_s_;
        }

        const double yaw_change = std::abs(
          shortest_angular_distance(previous_filtered_yaw_, yaw));
        if (yaw_change > max_yaw_jump_rad_) {
          yaw_jump_fault_until_s_ = receive_time.seconds() + timestamp_fault_hold_s_;
        }
      }
    }

    if (finite(x) && finite(y) && finite(yaw)) {
      filtered_pose_received_ = true;
      previous_filtered_receive_ = receive_time;
      previous_filtered_x_ = x;
      previous_filtered_y_ = y;
      previous_filtered_yaw_ = yaw;
    }
  }

  void on_vo_odom(const nav_msgs::msg::Odometry & message)
  {
    // /vo/odom is a pose measurement of the same planar base_footprint used by
    // robot_localization. VO does not own or publish the odom TF.
    record_odom(vo_tracker_, message, odom_frame_id_, base_frame_id_);
  }

  void on_imu_state(const std::string & payload)
  {
    if (contains_case_sensitive(payload, "unexpected BNO055 chip id") ||
      contains_case_sensitive(payload, "BNO055 system error") ||
      contains_case_sensitive(payload, "BNO055 calibration restore failed"))
    {
      imu_tracker_.diagnostic_error = true;
      imu_tracker_.detail = "imu_state_reports_error";
    } else if (contains_case_sensitive(payload, "IMU usable, calibration restore failed")) {
      imu_tracker_.diagnostic_error = false;
      imu_tracker_.diagnostic_warning = true;
      imu_tracker_.detail = "imu_calibration_restore_failed_optional";
    } else if (contains_case_sensitive(payload, "calibration not fully ready")) {
      imu_tracker_.diagnostic_error = false;
      imu_tracker_.diagnostic_warning = true;
      imu_tracker_.detail = "imu_calibration_not_motion_ready";
    } else if (contains_case_sensitive(payload, "IMU healthy")) {
      imu_tracker_.diagnostic_error = false;
      imu_tracker_.diagnostic_warning = false;
    }
  }

  void on_wheel_state(const std::string & payload)
  {
    if (!contains_case_sensitive(payload, "\"status\":\"OK\"")) {
      wheel_tracker_.diagnostic_error = true;
      wheel_tracker_.detail = "wheel_state_not_ok";
      return;
    }

    wheel_tracker_.diagnostic_error = false;
    const int illegal_transitions = extract_json_integer(
      payload, "total_illegal_transitions", -1);
    if (illegal_transitions > max_encoder_illegal_transitions_) {
      wheel_tracker_.diagnostic_warning = true;
      wheel_tracker_.detail = "encoder_illegal_transition_limit_exceeded";
    } else {
      wheel_tracker_.diagnostic_warning = false;
    }
  }

  void on_diagnostics(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    for (const auto & status : message.status) {
      if (status.name != "savo_localization/imu_node") {
        continue;
      }

      imu_tracker_.diagnostic_error =
        status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      imu_tracker_.diagnostic_warning =
        status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN;
      if (imu_tracker_.diagnostic_error || imu_tracker_.diagnostic_warning) {
        imu_tracker_.detail = status.message;
      }
    }
  }

  [[nodiscard]] SourceHealthObservation make_source_observation(
    const std::string & name,
    const RuntimeTracker & tracker,
    const bool enabled,
    const bool required,
    const double max_age_s,
    const double expected_rate_hz,
    const rclcpp::Time & current_time) const
  {
    SourceHealthObservation observation;
    observation.name = name;
    observation.enabled = enabled;
    observation.required = required;
    observation.received = tracker.received;
    observation.age_s = tracker.age_s(current_time);
    observation.fresh = tracker.received && observation.age_s <= max_age_s;
    observation.data_valid = tracker.data_valid;
    observation.frame_valid = tracker.frame_valid;
    observation.timestamp_valid = tracker.timestamp_valid(current_time);
    observation.rate_hz = tracker.rate_hz();
    observation.rate_valid =
      tracker.arrival_times_s.size() < kRateWindowMinimum ||
      observation.rate_hz >= expected_rate_hz * rate_tolerance_ratio_;
    observation.diagnostic_warning = tracker.diagnostic_warning;
    observation.diagnostic_error = tracker.diagnostic_error;
    observation.detail = tracker.detail;
    return observation;
  }

  [[nodiscard]] TransformObservationResult observe_transform(
    const std::string & target_frame,
    const std::string & source_frame,
    const bool require_fresh_stamp,
    const rclcpp::Time & current_time)
  {
    TransformObservationResult result;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
      result.available = true;

      const rclcpp::Time transform_time(transform.header.stamp);
      if (!require_fresh_stamp || transform_time.nanoseconds() == 0) {
        result.fresh = true;
        result.age_s = 0.0;
      } else {
        result.age_s = std::max(0.0, (current_time - transform_time).seconds());
        result.fresh = result.age_s <= max_tf_age_s_;
        if (!result.fresh) {
          result.detail = "age_s=" + std::to_string(result.age_s);
        }
      }
    } catch (const tf2::TransformException & exception) {
      result.detail = exception.what();
    }
    return result;
  }

  [[nodiscard]] LocalizationHealthInputs build_inputs(const rclcpp::Time & current_time)
  {
    LocalizationHealthInputs inputs;
    inputs.startup_age_s = std::max(0.0, (current_time - start_time_).seconds());
    inputs.startup_grace_s = startup_grace_s_;

    inputs.imu = make_source_observation(
      "imu", imu_tracker_, use_imu_, use_imu_, max_imu_age_s_,
      expected_imu_rate_hz_, current_time);
    inputs.wheel_odom = make_source_observation(
      "wheel_odom", wheel_tracker_, use_wheel_odom_, use_wheel_odom_,
      max_wheel_odom_age_s_, expected_wheel_odom_rate_hz_, current_time);
    inputs.filtered_odom = make_source_observation(
      "filtered_odom", filtered_tracker_, use_ekf_, use_ekf_,
      max_filtered_odom_age_s_, expected_ekf_rate_hz_, current_time);
    inputs.vo_odom = make_source_observation(
      "vo_odom", vo_tracker_, use_vo_, vo_required_, max_vo_odom_age_s_,
      expected_vo_rate_hz_, current_time);

    const auto odom_to_base = observe_transform(
      odom_frame_id_, base_frame_id_, true, current_time);
    inputs.odom_to_base.name = "odom_to_base_footprint_tf";
    inputs.odom_to_base.required = require_odom_to_base_tf_;
    inputs.odom_to_base.available = odom_to_base.available;
    inputs.odom_to_base.fresh = odom_to_base.fresh;
    inputs.odom_to_base.age_s = odom_to_base.age_s;
    inputs.odom_to_base.detail = odom_to_base.detail;

    const auto base_to_imu = observe_transform(
      base_frame_id_, imu_frame_id_, false, current_time);
    inputs.base_to_imu.name = "base_footprint_to_imu_tf";
    inputs.base_to_imu.required = require_base_to_imu_tf_ && use_imu_;
    inputs.base_to_imu.available = base_to_imu.available;
    inputs.base_to_imu.fresh = base_to_imu.fresh;
    inputs.base_to_imu.age_s = base_to_imu.age_s;
    inputs.base_to_imu.detail = base_to_imu.detail;

    inputs.filtered_pose_jump_detected =
      current_time.seconds() < pose_jump_fault_until_s_;
    inputs.filtered_yaw_jump_detected =
      current_time.seconds() < yaw_jump_fault_until_s_;

    return inputs;
  }

  static void append_source_json(
    std::ostringstream & output,
    const SourceHealthObservation & source)
  {
    output << "{"
           << "\"enabled\":" << bool_text(source.enabled) << ','
           << "\"required\":" << bool_text(source.required) << ','
           << "\"received\":" << bool_text(source.received) << ','
           << "\"fresh\":" << bool_text(source.fresh) << ','
           << "\"data_valid\":" << bool_text(source.data_valid) << ','
           << "\"frame_valid\":" << bool_text(source.frame_valid) << ','
           << "\"timestamp_valid\":" << bool_text(source.timestamp_valid) << ','
           << "\"rate_valid\":" << bool_text(source.rate_valid) << ','
           << "\"diagnostic_warning\":" << bool_text(source.diagnostic_warning) << ','
           << "\"diagnostic_error\":" << bool_text(source.diagnostic_error) << ','
           << "\"age_s\":" << source.age_s << ','
           << "\"rate_hz\":" << source.rate_hz << ','
           << "\"detail\":\"" << escape_json(source.detail) << "\""
           << '}';
  }

  static void append_transform_json(
    std::ostringstream & output,
    const TransformHealthObservation & transform)
  {
    output << "{"
           << "\"required\":" << bool_text(transform.required) << ','
           << "\"available\":" << bool_text(transform.available) << ','
           << "\"fresh\":" << bool_text(transform.fresh) << ','
           << "\"age_s\":" << transform.age_s << ','
           << "\"detail\":\"" << escape_json(transform.detail) << "\""
           << '}';
  }

  [[nodiscard]] std::string health_json(
    const LocalizationHealthResult & result,
    const LocalizationHealthInputs & inputs,
    const rclcpp::Time & current_time) const
  {
    std::ostringstream output;
    output << std::fixed << std::setprecision(6);
    output << '{'
           << "\"schema_version\":1,"
           << "\"node\":\"localization_health_node\","
           << "\"state\":\"" << LocalizationHealthCore::ToString(result.state) << "\","
           << "\"ready\":" << bool_text(result.ready) << ','
           << "\"degraded\":" << bool_text(result.degraded) << ','
           << "\"reason_code\":\"" << escape_json(result.reason_code) << "\","
           << "\"stamp_s\":" << current_time.seconds() << ','
           << "\"startup_age_s\":" << inputs.startup_age_s << ','
           << "\"frames\":{"
           << "\"odom\":\"" << escape_json(odom_frame_id_) << "\","
           << "\"base\":\"" << escape_json(base_frame_id_) << "\","
           << "\"imu\":\"" << escape_json(imu_frame_id_) << "\"},"
           << "\"reasons\":[";

    for (std::size_t index = 0; index < result.reasons.size(); ++index) {
      if (index > 0U) {
        output << ',';
      }
      output << '"' << escape_json(result.reasons[index]) << '"';
    }

    output << "],\"components\":{\"imu\":";
    append_source_json(output, inputs.imu);
    output << ",\"wheel_odom\":";
    append_source_json(output, inputs.wheel_odom);
    output << ",\"filtered_odom\":";
    append_source_json(output, inputs.filtered_odom);
    output << ",\"vo_odom\":";
    append_source_json(output, inputs.vo_odom);
    output << "},\"transforms\":{\"odom_to_base_footprint\":";
    append_transform_json(output, inputs.odom_to_base);
    output << ",\"base_footprint_to_imu\":";
    append_transform_json(output, inputs.base_to_imu);
    output << "}}";
    return output.str();
  }

  [[nodiscard]] std::string summary_json(
    const LocalizationHealthResult & result,
    const rclcpp::Time & current_time) const
  {
    std::ostringstream output;
    output << std::fixed << std::setprecision(6)
           << '{'
           << "\"schema_version\":1,"
           << "\"state\":\"" << LocalizationHealthCore::ToString(result.state) << "\","
           << "\"ready\":" << bool_text(result.ready) << ','
           << "\"degraded\":" << bool_text(result.degraded) << ','
           << "\"reason_code\":\"" << escape_json(result.reason_code) << "\","
           << "\"stamp_s\":" << current_time.seconds()
           << '}';
    return output.str();
  }

  diagnostic_msgs::msg::DiagnosticArray make_diagnostics(
    const LocalizationHealthResult & result,
    const LocalizationHealthInputs & inputs,
    const rclcpp::Time & current_time) const
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = current_time;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "savo_localization/localization_health";
    status.hardware_id = "robot_savo_localization";
    status.level = diagnostic_level(result.state);
    status.message = result.reason_code;
    status.values.push_back(key_value(
      "state", std::string(LocalizationHealthCore::ToString(result.state))));
    status.values.push_back(key_value("ready", bool_text(result.ready)));
    status.values.push_back(key_value("degraded", bool_text(result.degraded)));
    status.values.push_back(key_value("reasons", join_reasons(result.reasons)));
    status.values.push_back(key_value("base_frame", base_frame_id_));
    status.values.push_back(key_value(
      "filtered_odom_age_s", std::to_string(inputs.filtered_odom.age_s)));
    status.values.push_back(key_value(
      "odom_to_base_tf", bool_text(inputs.odom_to_base.available && inputs.odom_to_base.fresh)));
    status.values.push_back(key_value(
      "base_to_imu_tf", bool_text(inputs.base_to_imu.available && inputs.base_to_imu.fresh)));
    array.status.push_back(std::move(status));
    return array;
  }

  void evaluate_and_publish()
  {
    const auto current_time = now();
    const auto inputs = build_inputs(current_time);
    const auto result = core_.Evaluate(inputs);

    std_msgs::msg::String health_message;
    health_message.data = health_json(result, inputs, current_time);
    health_publisher_->publish(health_message);

    std_msgs::msg::String summary_message;
    summary_message.data = summary_json(result, current_time);
    summary_publisher_->publish(summary_message);

    diagnostics_publisher_->publish(make_diagnostics(result, inputs, current_time));

    if (result.state != last_state_ || result.reason_code != last_reason_code_) {
      if (result.state == LocalizationHealthState::kError && log_errors_) {
        RCLCPP_ERROR(
          get_logger(), "localization health=%s reason=%s details=%s",
          std::string(LocalizationHealthCore::ToString(result.state)).c_str(),
          result.reason_code.c_str(), join_reasons(result.reasons).c_str());
      } else if (!result.ready && log_warnings_) {
        RCLCPP_WARN(
          get_logger(), "localization health=%s reason=%s details=%s",
          std::string(LocalizationHealthCore::ToString(result.state)).c_str(),
          result.reason_code.c_str(), join_reasons(result.reasons).c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "localization health=%s reason=%s",
          std::string(LocalizationHealthCore::ToString(result.state)).c_str(),
          result.reason_code.c_str());
      }
    }

    last_state_ = result.state;
    last_reason_code_ = result.reason_code;
    last_result_ = result;
  }

  void publish_heartbeat()
  {
    std_msgs::msg::String message;
    std::ostringstream output;
    output << std::fixed << std::setprecision(6)
           << '{'
           << "\"schema_version\":1,"
           << "\"node\":\"localization_health_node\","
           << "\"alive\":true,"
           << "\"state\":\"" << LocalizationHealthCore::ToString(last_result_.state) << "\","
           << "\"ready\":" << bool_text(last_result_.ready) << ','
           << "\"stamp_s\":" << now().seconds()
           << '}';
    message.data = output.str();
    heartbeat_publisher_->publish(message);
  }

  LocalizationHealthCore core_{};

  double publish_rate_hz_{2.0};
  double heartbeat_rate_hz_{1.0};
  double startup_grace_s_{3.0};
  double timestamp_fault_hold_s_{2.0};

  std::string imu_topic_{"/imu/data"};
  std::string imu_state_topic_{"/savo_localization/imu_state"};
  std::string wheel_odom_topic_{"/wheel/odom"};
  std::string wheel_odom_state_topic_{"/savo_localization/wheel_odom_state"};
  std::string filtered_odom_topic_{"/odometry/filtered"};
  std::string vo_odom_topic_{"/vo/odom"};
  std::string diagnostics_topic_{"/diagnostics"};
  std::string health_topic_{"/savo_localization/health"};
  std::string state_summary_topic_{"/savo_localization/state_summary"};
  std::string heartbeat_topic_{"/savo_localization/heartbeat"};

  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_footprint"};
  std::string imu_frame_id_{"imu_link"};

  bool use_imu_{true};
  bool use_wheel_odom_{true};
  bool use_ekf_{true};
  bool use_vo_{false};
  bool vo_required_{false};
  bool require_odom_to_base_tf_{true};
  bool require_base_to_imu_tf_{true};

  double expected_imu_rate_hz_{25.0};
  double expected_wheel_odom_rate_hz_{30.0};
  double expected_ekf_rate_hz_{30.0};
  double expected_vo_rate_hz_{15.0};
  double rate_tolerance_ratio_{0.50};
  std::size_t rate_window_size_{30U};

  double max_imu_age_s_{0.5};
  double max_wheel_odom_age_s_{0.5};
  double max_filtered_odom_age_s_{0.5};
  double max_vo_odom_age_s_{0.5};
  double max_tf_age_s_{0.5};
  double max_odom_linear_speed_mps_{1.5};
  double max_odom_angular_speed_rad_s_{4.0};
  double max_pose_jump_m_{0.50};
  double max_yaw_jump_rad_{1.00};
  int max_encoder_illegal_transitions_{20};
  bool log_warnings_{true};
  bool log_errors_{true};

  RuntimeTracker imu_tracker_{};
  RuntimeTracker wheel_tracker_{};
  RuntimeTracker filtered_tracker_{};
  RuntimeTracker vo_tracker_{};

  bool filtered_pose_received_{false};
  rclcpp::Time previous_filtered_receive_{};
  double previous_filtered_x_{0.0};
  double previous_filtered_y_{0.0};
  double previous_filtered_yaw_{0.0};
  double pose_jump_fault_until_s_{0.0};
  double yaw_jump_fault_until_s_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Time start_time_;

  LocalizationHealthState last_state_{LocalizationHealthState::kUnknown};
  std::string last_reason_code_{"not_evaluated"};
  LocalizationHealthResult last_result_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr summary_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wheel_state_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_subscription_;

  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace savo_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<savo_localization::LocalizationHealthNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("localization_health_node"),
      "Fatal localization health error: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
