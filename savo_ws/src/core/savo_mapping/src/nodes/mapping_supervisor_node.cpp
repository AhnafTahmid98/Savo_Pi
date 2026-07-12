#include "savo_mapping/exploration_mode.hpp"
#include "savo_mapping/mapping_mode.hpp"
#include "savo_mapping/mapping_status.hpp"
#include "savo_mapping/map_quality_placeholder.hpp"
#include "savo_mapping/parameter_utils.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/session_state.hpp"
#include "savo_mapping/supervisor_runtime.hpp"
#include "savo_mapping/topic_names.hpp"
#include "savo_mapping/version.hpp"
#include "savo_mapping/workflow_phase.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_mapping
{

namespace
{

using SteadyClock = std::chrono::steady_clock;
using ReceiptTime = SteadyClock::time_point;

std::string require_non_empty(
  const std::string & parameter_name,
  std::string value)
{
  if (value.empty()) {
    throw std::invalid_argument(
            "parameter '" + parameter_name + "' must not be empty");
  }

  return value;
}

std::chrono::nanoseconds rate_to_period(double rate_hz)
{
  const auto duration = std::chrono::duration<double>(1.0 / rate_hz);

  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration);
}

double receipt_age_s(
  const std::optional<ReceiptTime> & receipt,
  const ReceiptTime & now)
{
  if (!receipt.has_value()) {
    return std::numeric_limits<double>::infinity();
  }

  return std::chrono::duration<double>(now - receipt.value()).count();
}

bool is_zero_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

}  // namespace

class MappingSupervisorNode final : public rclcpp::Node
{
public:
  MappingSupervisorNode()
  : Node("mapping_supervisor_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    configure_initial_status();
    create_publishers();
    create_subscriptions();
    create_timers();

    evaluate_runtime_state();
    publish_status();

    RCLCPP_INFO(
      get_logger(),
      "savo_mapping supervisor started: version=%s mode=%s odom_topic=%s",
      std::string{version::get_version()}.c_str(),
      std::string{to_string(status_.mode)}.c_str(),
      odom_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "monitoring scan=%s map=%s frames=%s->%s->%s->%s",
      scan_topic_.c_str(),
      map_topic_.c_str(),
      map_frame_.c_str(),
      odom_frame_.c_str(),
      base_frame_.c_str(),
      lidar_frame_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "freshness timeouts: scan=%.2fs map=%.2fs odom=%.2fs tf=%.2fs",
      scan_timeout_s_,
      map_timeout_s_,
      odom_timeout_s_,
      tf_stale_timeout_s_);

    RCLCPP_INFO(
      get_logger(),
      "monitor-only ownership lock: direct_motor=false "
      "internal_calls=false navigation_handoff=false");
  }

private:
  using StringPublisher = rclcpp::Publisher<std_msgs::msg::String>;

  void declare_parameters()
  {
    publish_rate_hz_ = params::require_positive_parameter(
      "mapping.publish_rate_hz",
      params::declare_or_get<double>(
        *this, "mapping.publish_rate_hz", 5.0));

    heartbeat_rate_hz_ = params::require_positive_parameter(
      "mapping.heartbeat_rate_hz",
      params::declare_or_get<double>(
        *this, "mapping.heartbeat_rate_hz", 1.0));

    const auto mode_text = params::declare_or_get<std::string>(
      *this, "mapping.default_mode", std::string{"monitor_only"});

    const auto parsed_mode = mapping_mode_from_string(mode_text);

    if (!parsed_mode.has_value()) {
      throw std::invalid_argument(
              "invalid mapping.default_mode: " + mode_text);
    }

    configured_mode_ = parsed_mode.value();

    const auto exploration_text = params::declare_or_get<std::string>(
      *this,
      "mapping.default_exploration_mode",
      std::string{"idle"});

    const auto parsed_exploration =
      exploration_mode_from_string(exploration_text);

    if (!parsed_exploration.has_value()) {
      throw std::invalid_argument(
              "invalid mapping.default_exploration_mode: " +
              exploration_text);
    }

    configured_exploration_mode_ = parsed_exploration.value();

    use_filtered_odom_ = params::declare_or_get<bool>(
      *this, "mapping.use_filtered_odom", true);

    require_scan_ = params::declare_or_get<bool>(
      *this, "mapping.require_scan", true);

    require_tf_ = params::declare_or_get<bool>(
      *this, "mapping.require_tf", true);

    require_odom_ = params::declare_or_get<bool>(
      *this, "mapping.require_odom", true);

    require_map_ = params::declare_or_get<bool>(
      *this, "mapping.require_map", true);

    allow_direct_motor_control_ =
      params::declare_or_get<bool>(
      *this,
      "mapping.allow_direct_motor_control",
      false);

    allow_internal_package_calls_ =
      params::declare_or_get<bool>(
      *this,
      "mapping.allow_internal_package_calls",
      false);

    navigation_handoff_enabled_ =
      params::declare_or_get<bool>(
      *this,
      "navigation_handoff.enabled",
      false);

    realsense_monitoring_enabled_ =
      params::declare_or_get<bool>(
      *this,
      "optional_features.realsense_monitoring_enabled",
      false);

    voxel_obstacle_monitoring_enabled_ =
      params::declare_or_get<bool>(
      *this,
      "optional_features.voxel_obstacle_monitoring_enabled",
      false);

    semantic_mapping_enabled_ =
      params::declare_or_get<bool>(
      *this,
      "optional_features.semantic_mapping_enabled",
      false);

    const MonitorOnlyPolicy monitor_only_policy{
      allow_direct_motor_control_,
      allow_internal_package_calls_,
      navigation_handoff_enabled_,
      realsense_monitoring_enabled_,
      voxel_obstacle_monitoring_enabled_,
      semantic_mapping_enabled_
    };

    const std::string policy_error =
      validate_monitor_only_policy(monitor_only_policy);

    if (!policy_error.empty()) {
      throw std::invalid_argument(
              "invalid monitor-only policy: " + policy_error);
    }

    scan_timeout_s_ = params::require_positive_parameter(
      "input_timeouts.scan_s",
      params::declare_or_get<double>(
        *this, "input_timeouts.scan_s", 1.0));

    map_timeout_s_ = params::require_positive_parameter(
      "input_timeouts.map_s",
      params::declare_or_get<double>(
        *this, "input_timeouts.map_s", 5.0));

    odom_timeout_s_ = params::require_positive_parameter(
      "input_timeouts.odom_s",
      params::declare_or_get<double>(
        *this, "input_timeouts.odom_s", 1.0));

    tf_stale_timeout_s_ = params::require_positive_parameter(
      "frame_policy.tf_stale_timeout_s",
      params::declare_or_get<double>(
        *this, "frame_policy.tf_stale_timeout_s", 1.0));

    free_threshold_ =
      params::require_parameter_in_closed_range(
      "map_output.free_threshold",
      params::declare_or_get<double>(
        *this, "map_output.free_threshold", 0.25),
      0.0,
      1.0);

    occupied_threshold_ =
      params::require_parameter_in_closed_range(
      "map_output.occupied_threshold",
      params::declare_or_get<double>(
        *this, "map_output.occupied_threshold", 0.65),
      0.0,
      1.0);

    if (free_threshold_ >= occupied_threshold_) {
      throw std::invalid_argument(
              "map_output.free_threshold must be lower than "
              "map_output.occupied_threshold");
    }

    require_map_to_odom_ = params::declare_or_get<bool>(
      *this, "frame_policy.require_map_to_odom", true);

    require_odom_to_base_ = params::declare_or_get<bool>(
      *this, "frame_policy.require_odom_to_base", true);

    require_base_to_lidar_ = params::declare_or_get<bool>(
      *this, "frame_policy.require_base_to_lidar", true);

    require_fresh_map_to_odom_ =
      params::declare_or_get<bool>(
      *this,
      "frame_policy.require_fresh_map_to_odom",
      false);

    require_fresh_odom_to_base_ =
      params::declare_or_get<bool>(
      *this,
      "frame_policy.require_fresh_odom_to_base",
      true);

    require_fresh_base_to_lidar_ =
      params::declare_or_get<bool>(
      *this,
      "frame_policy.require_fresh_base_to_lidar",
      false);

    scan_topic_ = require_non_empty(
      "topics.inputs.scan",
      params::declare_or_get<std::string>(
        *this, "topics.inputs.scan", std::string{topics::SCAN}));

    map_topic_ = require_non_empty(
      "topics.inputs.map",
      params::declare_or_get<std::string>(
        *this, "topics.inputs.map", std::string{topics::MAP}));

    const auto raw_odom_topic = require_non_empty(
      "topics.inputs.odom",
      params::declare_or_get<std::string>(
        *this, "topics.inputs.odom", std::string{topics::ODOM}));

    const auto filtered_odom_topic = require_non_empty(
      "topics.inputs.odom_filtered",
      params::declare_or_get<std::string>(
        *this,
        "topics.inputs.odom_filtered",
        std::string{topics::ODOM_FILTERED}));

    odom_topic_ =
      use_filtered_odom_ ? filtered_odom_topic : raw_odom_topic;

    status_topic_ = require_non_empty(
      "topics.outputs.status",
      params::declare_or_get<std::string>(
        *this, "topics.outputs.status", std::string{topics::STATUS}));

    readiness_topic_ = require_non_empty(
      "topics.outputs.readiness",
      params::declare_or_get<std::string>(
        *this,
        "topics.outputs.readiness",
        std::string{topics::READINESS}));

    mode_topic_ = require_non_empty(
      "topics.outputs.mode",
      params::declare_or_get<std::string>(
        *this, "topics.outputs.mode", std::string{topics::MODE}));

    workflow_phase_topic_ = require_non_empty(
      "topics.outputs.workflow_phase",
      params::declare_or_get<std::string>(
        *this,
        "topics.outputs.workflow_phase",
        std::string{topics::WORKFLOW_PHASE}));

    session_state_topic_ = require_non_empty(
      "topics.outputs.session_state",
      params::declare_or_get<std::string>(
        *this,
        "topics.outputs.session_state",
        std::string{topics::SESSION_STATE}));

    dashboard_topic_ = require_non_empty(
      "topics.outputs.dashboard",
      params::declare_or_get<std::string>(
        *this,
        "topics.outputs.dashboard",
        std::string{topics::DASHBOARD}));

    map_quality_topic_ = require_non_empty(
      "topics.outputs.map_quality",
      params::declare_or_get<std::string>(
        *this,
        "topics.outputs.map_quality",
        std::string{topics::MAP_QUALITY}));

    map_frame_ = require_non_empty(
      "frames.map",
      params::declare_or_get<std::string>(
        *this, "frames.map", std::string{"map"}));

    odom_frame_ = require_non_empty(
      "frames.odom",
      params::declare_or_get<std::string>(
        *this, "frames.odom", std::string{"odom"}));

    base_frame_ = require_non_empty(
      "frames.base",
      params::declare_or_get<std::string>(
        *this, "frames.base", std::string{"base_link"}));

    lidar_frame_ = require_non_empty(
      "frames.lidar",
      params::declare_or_get<std::string>(
        *this, "frames.lidar", std::string{"laser_frame"}));
  }

  void configure_initial_status()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);

    status_ = make_default_status();
    status_.mode = configured_mode_;
    status_.exploration_mode = configured_exploration_mode_;
    status_.workflow_phase = WorkflowPhase::Idle;
    status_.session_state = SessionState::Idle;
    status_.healthy = true;
    status_.ready = false;
    status_.slam_active = false;
    status_.message = "not_ready: startup";
  }

  void create_publishers()
  {
    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      status_topic_, qos::status_qos());

    readiness_publisher_ =
      create_publisher<std_msgs::msg::String>(
      readiness_topic_, qos::state_qos());

    mode_publisher_ =
      create_publisher<std_msgs::msg::String>(
      mode_topic_, qos::state_qos());

    workflow_phase_publisher_ =
      create_publisher<std_msgs::msg::String>(
      workflow_phase_topic_, qos::state_qos());

    session_state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      session_state_topic_, qos::state_qos());

    dashboard_publisher_ =
      create_publisher<std_msgs::msg::String>(
      dashboard_topic_, qos::status_qos());

    map_quality_publisher_ =
      create_publisher<std_msgs::msg::String>(
      map_quality_topic_, qos::state_qos());
  }

  void create_subscriptions()
  {
    using std::placeholders::_1;

    scan_subscription_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      qos::scan_qos(),
      std::bind(&MappingSupervisorNode::on_scan, this, _1));

    map_subscription_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      qos::map_qos(),
      std::bind(&MappingSupervisorNode::on_map, this, _1));

    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile(),
      std::bind(&MappingSupervisorNode::on_odom, this, _1));
  }

  void create_timers()
  {
    status_timer_ = create_wall_timer(
      rate_to_period(publish_rate_hz_),
      std::bind(&MappingSupervisorNode::on_status_timer, this));

    heartbeat_timer_ = create_wall_timer(
      rate_to_period(heartbeat_rate_hz_),
      std::bind(&MappingSupervisorNode::update_heartbeat, this));
  }

  void on_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);

    last_scan_receipt_ = SteadyClock::now();

    if (!message->header.frame_id.empty()) {
      active_scan_frame_ = message->header.frame_id;
    }
  }

  void on_map(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);

    last_map_receipt_ = SteadyClock::now();
    ++map_update_count_;

    const int free_threshold_percent =
      static_cast<int>(std::lround(free_threshold_ * 100.0));

    const int occupied_threshold_percent =
      static_cast<int>(std::lround(occupied_threshold_ * 100.0));

    map_quality_ = evaluate_map_quality_placeholder(
      message->info.resolution,
      message->info.width,
      message->info.height,
      message->data,
      map_update_count_,
      free_threshold_percent,
      occupied_threshold_percent);
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);

    last_odom_receipt_ = SteadyClock::now();
  }

  void on_status_timer()
  {
    evaluate_runtime_state();
    publish_status();
  }

  void update_heartbeat()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);

    ++status_.heartbeat_seq;
  }

  bool transform_stamp_is_fresh(
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    if (is_zero_stamp(transform.header.stamp)) {
      // Static transforms are timeless in the TF contract.
      return true;
    }

    const rclcpp::Time transform_time(
      transform.header.stamp,
      get_clock()->get_clock_type());

    const rclcpp::Time current_time = get_clock()->now();

    if (transform_time > current_time) {
      return false;
    }

    return (current_time - transform_time).seconds() <=
           tf_stale_timeout_s_;
  }

  bool check_transform(
    const std::string & target_frame,
    const std::string & source_frame,
    bool require_fresh_stamp,
    std::string & failure_reason)
  {
    if (target_frame == source_frame) {
      return true;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame,
        source_frame,
        tf2::TimePointZero);

      if (require_fresh_stamp &&
          !transform_stamp_is_fresh(transform))
      {
        failure_reason =
          "stale_transform:" + target_frame + "<-" + source_frame;
        return false;
      }

      return true;
    } catch (const tf2::TransformException & exception) {
      failure_reason =
        "missing_transform:" + target_frame + "<-" + source_frame;

      RCLCPP_DEBUG(
        get_logger(),
        "TF lookup failed for %s <- %s: %s",
        target_frame.c_str(),
        source_frame.c_str(),
        exception.what());

      return false;
    }
  }

  bool evaluate_tf(
    const std::string & scan_frame,
    std::string & failure_reason)
  {
    if (!require_tf_) {
      return true;
    }

    if (require_map_to_odom_ &&
        !check_transform(
          map_frame_,
          odom_frame_,
          require_fresh_map_to_odom_,
          failure_reason))
    {
      return false;
    }

    if (require_odom_to_base_ &&
        !check_transform(
          odom_frame_,
          base_frame_,
          require_fresh_odom_to_base_,
          failure_reason))
    {
      return false;
    }

    if (require_base_to_lidar_ &&
        !check_transform(
          base_frame_,
          scan_frame,
          require_fresh_base_to_lidar_,
          failure_reason))
    {
      return false;
    }

    return true;
  }

  void evaluate_runtime_state()
  {
    const ReceiptTime now = SteadyClock::now();

    std::optional<ReceiptTime> scan_receipt;
    std::optional<ReceiptTime> map_receipt;
    std::optional<ReceiptTime> odom_receipt;
    std::string scan_frame;
    bool map_structure_valid = false;
    double quality_score = 0.0;

    {
      std::lock_guard<std::mutex> lock(status_mutex_);

      scan_receipt = last_scan_receipt_;
      map_receipt = last_map_receipt_;
      odom_receipt = last_odom_receipt_;

      scan_frame =
        active_scan_frame_.empty() ?
        lidar_frame_ :
        active_scan_frame_;

      map_structure_valid = map_quality_.structurally_valid;
      quality_score = map_quality_.quality_score;
    }

    const bool scan_fresh = is_fresh_age(
      receipt_age_s(scan_receipt, now),
      scan_timeout_s_);

    const bool map_fresh = is_fresh_age(
      receipt_age_s(map_receipt, now),
      map_timeout_s_);

    const bool odom_fresh = is_fresh_age(
      receipt_age_s(odom_receipt, now),
      odom_timeout_s_);

    std::string tf_failure_reason;
    const bool tf_ok =
      evaluate_tf(scan_frame, tf_failure_reason);

    const SupervisorRequirements requirements{
      require_scan_,
      require_map_,
      require_odom_,
      require_tf_
    };

    const SupervisorInputs inputs{
      scan_fresh,
      map_fresh,
      odom_fresh,
      tf_ok,
      map_structure_valid
    };

    const SupervisorDecision decision =
      evaluate_supervisor_runtime(
      requirements,
      inputs,
      tf_failure_reason);

    {
      std::lock_guard<std::mutex> lock(status_mutex_);

      status_.healthy = true;
      status_.scan_received = scan_fresh;
      status_.map_received = map_fresh;
      status_.odom_ok = odom_fresh;
      status_.tf_ok = tf_ok;
      status_.slam_active = decision.slam_active;
      status_.ready = decision.ready;
      status_.quality_score = quality_score;
      status_.message = decision.reason;
    }

    log_readiness_transition(decision.reason, decision.ready);
  }

  void log_readiness_transition(
    const std::string & readiness_reason,
    bool ready)
  {
    if (readiness_reason == last_readiness_reason_) {
      return;
    }

    last_readiness_reason_ = readiness_reason;

    if (ready) {
      RCLCPP_INFO(
        get_logger(),
        "mapping readiness changed: %s",
        readiness_reason.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "mapping readiness changed: %s",
        readiness_reason.c_str());
    }
  }

  void publish_status()
  {
    MappingStatus snapshot;
    MapQualityPlaceholder quality_snapshot;

    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      snapshot = status_;
      quality_snapshot = map_quality_;
    }

    publish_text(status_publisher_, make_status_json(snapshot));
    publish_text(readiness_publisher_, snapshot.message);
    publish_text(mode_publisher_, std::string{to_string(snapshot.mode)});
    publish_text(
      workflow_phase_publisher_,
      std::string{to_string(snapshot.workflow_phase)});
    publish_text(
      session_state_publisher_,
      std::string{to_string(snapshot.session_state)});
    publish_text(
      dashboard_publisher_,
      make_dashboard_text(snapshot));

    publish_text(
      map_quality_publisher_,
      make_map_quality_json(quality_snapshot));
  }

  static void publish_text(
    const StringPublisher::SharedPtr & publisher,
    const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    publisher->publish(message);
  }

  static std::string make_dashboard_text(
    const MappingStatus & status)
  {
    std::ostringstream out;

    out
      << "mode=" << to_string(status.mode)
      << " exploration=" << to_string(status.exploration_mode)
      << " phase=" << to_string(status.workflow_phase)
      << " session=" << to_string(status.session_state)
      << " ready=" << (status.ready ? "true" : "false")
      << " scan=" << (status.scan_received ? "true" : "false")
      << " map=" << (status.map_received ? "true" : "false")
      << " odom=" << (status.odom_ok ? "true" : "false")
      << " tf=" << (status.tf_ok ? "true" : "false")
      << " slam=" << (status.slam_active ? "true" : "false")
      << " quality_score=" << status.quality_score
      << " heartbeat=" << status.heartbeat_seq
      << " reason=\"" << status.message << "\"";

    return out.str();
  }

  MappingMode configured_mode_{MappingMode::MonitorOnly};
  ExplorationMode configured_exploration_mode_{ExplorationMode::Idle};

  double publish_rate_hz_{5.0};
  double heartbeat_rate_hz_{1.0};

  double scan_timeout_s_{1.0};
  double map_timeout_s_{5.0};
  double odom_timeout_s_{1.0};
  double tf_stale_timeout_s_{1.0};
  double free_threshold_{0.25};
  double occupied_threshold_{0.65};

  bool use_filtered_odom_{true};
  bool require_scan_{true};
  bool require_tf_{true};
  bool require_odom_{true};
  bool require_map_{true};

  bool require_map_to_odom_{true};
  bool require_odom_to_base_{true};
  bool require_base_to_lidar_{true};

  bool require_fresh_map_to_odom_{false};
  bool require_fresh_odom_to_base_{true};
  bool require_fresh_base_to_lidar_{false};

  bool allow_direct_motor_control_{false};
  bool allow_internal_package_calls_{false};
  bool navigation_handoff_enabled_{false};

  bool realsense_monitoring_enabled_{false};
  bool voxel_obstacle_monitoring_enabled_{false};
  bool semantic_mapping_enabled_{false};

  std::string scan_topic_;
  std::string map_topic_;
  std::string odom_topic_;

  std::string status_topic_;
  std::string readiness_topic_;
  std::string mode_topic_;
  std::string workflow_phase_topic_;
  std::string session_state_topic_;
  std::string dashboard_topic_;
  std::string map_quality_topic_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string active_scan_frame_;

  std::mutex status_mutex_;
  MappingStatus status_;
  MapQualityPlaceholder map_quality_;
  std::uint64_t map_update_count_{0};

  std::optional<ReceiptTime> last_scan_receipt_;
  std::optional<ReceiptTime> last_map_receipt_;
  std::optional<ReceiptTime> last_odom_receipt_;

  std::string last_readiness_reason_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  StringPublisher::SharedPtr status_publisher_;
  StringPublisher::SharedPtr readiness_publisher_;
  StringPublisher::SharedPtr mode_publisher_;
  StringPublisher::SharedPtr workflow_phase_publisher_;
  StringPublisher::SharedPtr session_state_publisher_;
  StringPublisher::SharedPtr dashboard_publisher_;
  StringPublisher::SharedPtr map_quality_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
    scan_subscription_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_subscription_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<savo_mapping::MappingSupervisorNode>());
  } catch (const std::exception & exception) {
    std::cerr
      << "mapping_supervisor_node failed: "
      << exception.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
