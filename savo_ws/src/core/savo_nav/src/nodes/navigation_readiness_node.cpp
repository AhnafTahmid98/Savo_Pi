// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "savo_nav/action_names.hpp"
#include "savo_nav/frame_names.hpp"
#include "savo_nav/localization_monitor.hpp"
#include "savo_nav/nav2_health_monitor.hpp"
#include "savo_nav/navigation_readiness.hpp"
#include "savo_nav/safety_state_monitor.hpp"
#include "savo_nav/sensor_freshness_monitor.hpp"
#include "savo_nav/topic_names.hpp"

namespace
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;

double MonotonicSeconds()
{
  const auto now =
    std::chrono::steady_clock::now().time_since_epoch();

  return std::chrono::duration<double>(now).count();
}

std::string NormalizeFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }

  return frame;
}

std::string NormalizeMode(std::string mode)
{
  mode.erase(
    mode.begin(),
    std::find_if(
      mode.begin(),
      mode.end(),
      [](const unsigned char character)
      {
        return std::isspace(character) == 0;
      }));

  mode.erase(
    std::find_if(
      mode.rbegin(),
      mode.rend(),
      [](const unsigned char character)
      {
        return std::isspace(character) == 0;
      }).base(),
    mode.end());

  std::transform(
    mode.begin(),
    mode.end(),
    mode.begin(),
    [](const unsigned char character)
    {
      return static_cast<char>(
        std::toupper(character));
    });

  return mode;
}

std::string Join(
  const std::vector<std::string> & values)
{
  std::ostringstream stream;

  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index > 0) {
      stream << ',';
    }

    stream << values[index];
  }

  return stream.str();
}

class NavigationReadinessNode final : public rclcpp::Node
{
public:
  NavigationReadinessNode()
  : Node("navigation_readiness_node")
  {
    const int publish_period_ms =
      declare_parameter<int>(
      "publish_period_ms",
      200);

    if (publish_period_ms <= 0) {
      throw std::invalid_argument(
              "publish_period_ms must be greater than zero");
    }

    policy_.require_pointcloud =
      declare_parameter<bool>(
      "require_pointcloud",
      true);

    policy_.require_control_mode =
      declare_parameter<bool>(
      "require_control_mode",
      true);

    policy_.require_safety_state =
      declare_parameter<bool>(
      "require_safety_state",
      true);

    policy_.require_global_costmap =
      declare_parameter<bool>(
      "require_global_costmap",
      true);

    policy_.require_local_costmap =
      declare_parameter<bool>(
      "require_local_costmap",
      true);

    const double tf_timeout_seconds =
      declare_parameter<double>(
      "tf_timeout_seconds",
      3.0);

    const double odometry_timeout_seconds =
      declare_parameter<double>(
      "odometry_timeout_seconds",
      1.0);

    const double lidar_timeout_seconds =
      declare_parameter<double>(
      "lidar_timeout_seconds",
      1.0);

    const double pointcloud_timeout_seconds =
      declare_parameter<double>(
      "pointcloud_timeout_seconds",
      1.0);

    const double costmap_timeout_seconds =
      declare_parameter<double>(
      "costmap_timeout_seconds",
      3.0);

    const double control_timeout_seconds =
      declare_parameter<double>(
      "control_timeout_seconds",
      2.0);

    const double safety_timeout_seconds =
      declare_parameter<double>(
      "safety_timeout_seconds",
      1.0);

    accepted_control_modes_ =
      declare_parameter<std::vector<std::string>>(
      "accepted_control_modes",
      std::vector<std::string>{"NAV"});

    for (auto & mode : accepted_control_modes_) {
      mode = NormalizeMode(mode);
    }

    map_to_odom_monitor_ =
      std::make_unique<savo_nav::SensorFreshnessMonitor>(
      tf_timeout_seconds);

    odom_to_base_monitor_ =
      std::make_unique<savo_nav::SensorFreshnessMonitor>(
      tf_timeout_seconds);

    lidar_monitor_ =
      std::make_unique<savo_nav::SensorFreshnessMonitor>(
      lidar_timeout_seconds);

    pointcloud_monitor_ =
      std::make_unique<savo_nav::SensorFreshnessMonitor>(
      pointcloud_timeout_seconds);

    control_monitor_ =
      std::make_unique<savo_nav::SensorFreshnessMonitor>(
      control_timeout_seconds);

    localization_monitor_ =
      std::make_unique<savo_nav::LocalizationMonitor>(
      odometry_timeout_seconds);

    safety_monitor_ =
      std::make_unique<savo_nav::SafetyStateMonitor>(
      safety_timeout_seconds);

    nav2_monitor_ =
      std::make_unique<savo_nav::Nav2HealthMonitor>(
      costmap_timeout_seconds);

    const std::string map_topic =
      declare_parameter<std::string>(
      "map_topic",
      std::string(savo_nav::topics::kMap));

    const std::string tf_topic =
      declare_parameter<std::string>(
      "tf_topic",
      std::string(savo_nav::topics::kTf));

    const std::string tf_static_topic =
      declare_parameter<std::string>(
      "tf_static_topic",
      std::string(savo_nav::topics::kTfStatic));

    const std::string odometry_topic =
      declare_parameter<std::string>(
      "odometry_topic",
      std::string(
        savo_nav::topics::kFilteredOdometry));

    const std::string scan_topic =
      declare_parameter<std::string>(
      "scan_topic",
      std::string(savo_nav::topics::kLaserScan));

    const std::string pointcloud_topic =
      declare_parameter<std::string>(
      "pointcloud_topic",
      std::string(
        savo_nav::topics::kFilteredObstaclePoints));

    const std::string global_costmap_topic =
      declare_parameter<std::string>(
      "global_costmap_topic",
      std::string(
        savo_nav::topics::kGlobalCostmap));

    const std::string local_costmap_topic =
      declare_parameter<std::string>(
      "local_costmap_topic",
      std::string(
        savo_nav::topics::kLocalCostmap));

    const std::string control_mode_topic =
      declare_parameter<std::string>(
      "control_mode_topic",
      std::string(
        savo_nav::topics::kControlModeState));

    const std::string safety_stop_topic =
      declare_parameter<std::string>(
      "safety_stop_topic",
      std::string(savo_nav::topics::kSafetyStop));

    const std::string safety_slowdown_topic =
      declare_parameter<std::string>(
      "safety_slowdown_topic",
      std::string(
        savo_nav::topics::kSafetySlowdownFactor));

    const std::string nav2_action_name =
      declare_parameter<std::string>(
      "nav2_action_name",
      std::string(
        savo_nav::actions::kNav2NavigateToPose));

    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    state_qos.reliable();
    state_qos.transient_local();

    auto heartbeat_qos =
      rclcpp::QoS(rclcpp::KeepLast(10));
    heartbeat_qos.reliable();

    readiness_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(savo_nav::topics::kReadiness),
      state_qos);

    reason_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(
        savo_nav::topics::kReadinessReason),
      state_qos);

    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      std::string(savo_nav::topics::kStatus),
      state_qos);

    heartbeat_publisher_ =
      create_publisher<std_msgs::msg::UInt64>(
      std::string(savo_nav::topics::kHeartbeat),
      heartbeat_qos);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    map_subscription_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic,
      map_qos,
      [this](
        const nav_msgs::msg::OccupancyGrid::SharedPtr message)
      {
        map_available_ =
        message->info.width > 0 &&
        message->info.height > 0 &&
        NormalizeFrame(message->header.frame_id) ==
        savo_nav::frames::kMap;
      });

    tf_subscription_ =
      create_subscription<tf2_msgs::msg::TFMessage>(
      tf_topic,
      rclcpp::SensorDataQoS(),
      [this](
        const tf2_msgs::msg::TFMessage::SharedPtr message)
      {
        OnTf(message);
      });

    auto tf_static_qos =
      rclcpp::QoS(rclcpp::KeepLast(100));

    tf_static_qos.reliable();
    tf_static_qos.transient_local();

    tf_static_subscription_ =
      create_subscription<tf2_msgs::msg::TFMessage>(
      tf_static_topic,
      tf_static_qos,
      [this](
        const tf2_msgs::msg::TFMessage::SharedPtr message)
      {
        OnTf(message);
      });

    odometry_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic,
      rclcpp::SensorDataQoS(),
      [this](
        const nav_msgs::msg::Odometry::SharedPtr message)
      {
        const std::string parent =
        NormalizeFrame(message->header.frame_id);

        const std::string child =
        NormalizeFrame(message->child_frame_id);

        if (
          parent == savo_nav::frames::kOdom &&
          child == savo_nav::frames::kBaseFootprint &&
          std::isfinite(
            message->pose.pose.position.x) &&
          std::isfinite(
            message->pose.pose.position.y))
        {
          localization_monitor_->MarkOdometryReceived(
            MonotonicSeconds());
        }
      });

    scan_subscription_ =
      create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      rclcpp::SensorDataQoS(),
      [this](
        const sensor_msgs::msg::LaserScan::SharedPtr message)
      {
        if (
          !message->ranges.empty() &&
          std::isfinite(message->angle_increment) &&
          message->angle_increment > 0.0)
        {
          lidar_monitor_->MarkReceived(
            MonotonicSeconds());
        }
      });

    pointcloud_subscription_ =
      create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic,
      rclcpp::SensorDataQoS(),
      [this](
        const sensor_msgs::msg::PointCloud2::SharedPtr message)
      {
        if (
          message->width > 0 &&
          message->height > 0 &&
          !message->data.empty())
        {
          pointcloud_monitor_->MarkReceived(
            MonotonicSeconds());
        }
      });

    global_costmap_subscription_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
      global_costmap_topic,
      map_qos,
      [this](
        const nav_msgs::msg::OccupancyGrid::SharedPtr message)
      {
        if (
          message->info.width > 0 &&
          message->info.height > 0 &&
          NormalizeFrame(message->header.frame_id) ==
          savo_nav::frames::kMap)
        {
          nav2_monitor_->MarkGlobalCostmapReceived(
            MonotonicSeconds());
        }
      });

    local_costmap_subscription_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic,
      map_qos,
      [this](
        const nav_msgs::msg::OccupancyGrid::SharedPtr message)
      {
        if (
          message->info.width > 0 &&
          message->info.height > 0 &&
          NormalizeFrame(message->header.frame_id) ==
          savo_nav::frames::kOdom)
        {
          nav2_monitor_->MarkLocalCostmapReceived(
            MonotonicSeconds());
        }
      });

    control_mode_subscription_ =
      create_subscription<std_msgs::msg::String>(
      control_mode_topic,
      rclcpp::QoS(10).reliable(),
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        control_mode_ = NormalizeMode(message->data);

        control_allows_navigation_ =
        std::find(
          accepted_control_modes_.begin(),
          accepted_control_modes_.end(),
          control_mode_) !=
        accepted_control_modes_.end();

        control_monitor_->MarkReceived(
          MonotonicSeconds());
      });

    safety_stop_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic,
      rclcpp::QoS(10).reliable(),
      [this](
        const std_msgs::msg::Bool::SharedPtr message)
      {
        safety_monitor_->UpdateStop(
          message->data,
          MonotonicSeconds());
      });

    safety_slowdown_subscription_ =
      create_subscription<std_msgs::msg::Float32>(
      safety_slowdown_topic,
      rclcpp::QoS(10).reliable(),
      [this](
        const std_msgs::msg::Float32::SharedPtr message)
      {
        safety_monitor_->UpdateSlowdown(
          static_cast<double>(message->data),
          MonotonicSeconds());
      });

    nav2_client_ =
      rclcpp_action::create_client<NavigateToPose>(
      this,
      nav2_action_name);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      [this]()
      {
        EvaluateAndPublish();
      });

    EvaluateAndPublish();

    RCLCPP_INFO(
      get_logger(),
      "Navigation readiness node started: "
      "subscriber-only external package monitoring enabled");
  }

private:
  void OnTf(
    const tf2_msgs::msg::TFMessage::SharedPtr & message)
  {
    const double now = MonotonicSeconds();

    for (const auto & transform : message->transforms) {
      const std::string parent =
        NormalizeFrame(transform.header.frame_id);

      const std::string child =
        NormalizeFrame(transform.child_frame_id);

      if (
        parent == savo_nav::frames::kMap &&
        child == savo_nav::frames::kOdom)
      {
        map_to_odom_monitor_->MarkReceived(now);
      }

      if (
        parent == savo_nav::frames::kOdom &&
        child == savo_nav::frames::kBaseFootprint)
      {
        odom_to_base_monitor_->MarkReceived(now);
      }

      if (
        parent == savo_nav::frames::kBaseFootprint &&
        child == savo_nav::frames::kBaseLink)
      {
        base_footprint_to_base_link_available_ = true;
      }
    }
  }

  void EvaluateAndPublish()
  {
    const double now = MonotonicSeconds();

    nav2_monitor_->UpdateActionServerAvailable(
      nav2_client_->wait_for_action_server(
        std::chrono::seconds(0)));

    const auto localization =
      localization_monitor_->GetSnapshot(now);

    const auto safety =
      safety_monitor_->GetSnapshot(now);

    const auto nav2 =
      nav2_monitor_->GetSnapshot(now);

    savo_nav::NavigationDependencySnapshot snapshot;

    snapshot.map_available = map_available_;

    snapshot.map_to_odom_fresh =
      map_to_odom_monitor_->IsFresh(now);

    snapshot.odom_to_base_fresh =
      odom_to_base_monitor_->IsFresh(now);

    snapshot.base_footprint_to_base_link_available =
      base_footprint_to_base_link_available_;

    snapshot.localization_fresh =
      localization.odometry_fresh;

    snapshot.lidar_fresh =
      lidar_monitor_->IsFresh(now);

    snapshot.pointcloud_fresh =
      pointcloud_monitor_->IsFresh(now);

    snapshot.nav2_action_server_available =
      nav2.action_server_available;

    snapshot.global_costmap_fresh =
      nav2.global_costmap_fresh;

    snapshot.local_costmap_fresh =
      nav2.local_costmap_fresh;

    snapshot.control_state_fresh =
      control_monitor_->IsFresh(now);

    snapshot.control_allows_navigation =
      control_allows_navigation_;

    snapshot.safety_state_fresh =
      safety.state_fresh;

    snapshot.safety_stop_active =
      safety.stop_active;

    snapshot.slowdown_valid =
      safety.slowdown_valid;

    snapshot.slowdown_factor =
      safety.slowdown_factor;

    const auto evaluated =
      savo_nav::NavigationReadiness::Evaluate(
      snapshot,
      policy_);

    const bool changed =
      evaluated.state != readiness_.GetResult().state ||
      evaluated.reason != readiness_.GetResult().reason;

    const bool updated = readiness_.Update(
      evaluated.state,
      evaluated.goal_acceptance_allowed,
      evaluated.reason,
      evaluated.failed_dependencies);

    if (!updated) {
      RCLCPP_ERROR(
        get_logger(),
        "Readiness evaluator produced an invalid result");

      return;
    }

    Publish();

    if (changed) {
      RCLCPP_INFO(
        get_logger(),
        "Readiness changed: state=%s reason=%s "
        "goal_acceptance_allowed=%s",
        std::string(
          savo_nav::NavigationReadiness::ToString(
            readiness_.GetResult().state)).c_str(),
        readiness_.GetResult().reason.c_str(),
        readiness_.GetResult().goal_acceptance_allowed ?
        "true" : "false");
    }
  }

  void Publish()
  {
    const auto & result = readiness_.GetResult();

    std_msgs::msg::String readiness_message;

    readiness_message.data = std::string(
      savo_nav::NavigationReadiness::ToString(
        result.state));

    readiness_publisher_->publish(readiness_message);

    std_msgs::msg::String reason_message;
    reason_message.data = result.reason;

    reason_publisher_->publish(reason_message);

    std_msgs::msg::String status_message;

    status_message.data =
      "state=" + readiness_message.data +
      ";goal_acceptance_allowed=" +
      (
      result.goal_acceptance_allowed ?
      std::string("true") :
      std::string("false")
      ) +
      ";reason=" + result.reason +
      ";failed_dependencies=" +
      Join(result.failed_dependencies);

    status_publisher_->publish(status_message);

    std_msgs::msg::UInt64 heartbeat_message;
    heartbeat_message.data = ++heartbeat_count_;

    heartbeat_publisher_->publish(heartbeat_message);
  }

  savo_nav::NavigationReadiness readiness_{};
  savo_nav::NavigationReadinessPolicy policy_{};

  std::unique_ptr<savo_nav::SensorFreshnessMonitor>
  map_to_odom_monitor_;

  std::unique_ptr<savo_nav::SensorFreshnessMonitor>
  odom_to_base_monitor_;

  std::unique_ptr<savo_nav::SensorFreshnessMonitor>
  lidar_monitor_;

  std::unique_ptr<savo_nav::SensorFreshnessMonitor>
  pointcloud_monitor_;

  std::unique_ptr<savo_nav::SensorFreshnessMonitor>
  control_monitor_;

  std::unique_ptr<savo_nav::LocalizationMonitor>
  localization_monitor_;

  std::unique_ptr<savo_nav::SafetyStateMonitor>
  safety_monitor_;

  std::unique_ptr<savo_nav::Nav2HealthMonitor>
  nav2_monitor_;

  std::vector<std::string> accepted_control_modes_;

  bool map_available_{false};

  bool base_footprint_to_base_link_available_{false};

  bool control_allows_navigation_{false};

  std::string control_mode_{};

  std::uint64_t heartbeat_count_{0};

  rclcpp_action::Client<NavigateToPose>::SharedPtr
    nav2_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    readiness_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    reason_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr
    heartbeat_publisher_;

  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr
    map_subscription_;

  rclcpp::Subscription<
    tf2_msgs::msg::TFMessage>::SharedPtr
    tf_subscription_;

  rclcpp::Subscription<
    tf2_msgs::msg::TFMessage>::SharedPtr
    tf_static_subscription_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    odometry_subscription_;

  rclcpp::Subscription<
    sensor_msgs::msg::LaserScan>::SharedPtr
    scan_subscription_;

  rclcpp::Subscription<
    sensor_msgs::msg::PointCloud2>::SharedPtr
    pointcloud_subscription_;

  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr
    global_costmap_subscription_;

  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid>::SharedPtr
    local_costmap_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    control_mode_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    safety_stop_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Float32>::SharedPtr
    safety_slowdown_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    rclcpp::spin(
      std::make_shared<NavigationReadinessNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "navigation_readiness_node"),
      "Fatal error: %s",
      exception.what());

    exit_code = 1;
  }

  rclcpp::shutdown();

  return exit_code;
}
