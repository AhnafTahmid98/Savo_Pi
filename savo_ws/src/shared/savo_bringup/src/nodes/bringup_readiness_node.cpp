// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_bringup/bringup_contract.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using std_msgs::msg::Bool;
using std_msgs::msg::String;
using std_msgs::msg::UInt64;

constexpr char kNodeName[] = "bringup_readiness_node";

std::string BoolText(const bool value)
{
  return value ? "true" : "false";
}

bool ContainsReadyToken(const std::string & value)
{
  std::string normalized(value);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return normalized == "ready" || normalized.rfind("ok:", 0U) == 0U ||
         normalized.find("ready=true") != std::string::npos ||
         normalized.find("\"ready\":true") != std::string::npos ||
         normalized.find("\"ok\":true") != std::string::npos ||
         normalized.find("\"healthy\":true") != std::string::npos ||
         normalized.find("\"state\":\"ready\"") != std::string::npos ||
         normalized.find("synchronized=true") != std::string::npos ||
         normalized.find("level=ok") != std::string::npos ||
         normalized.find("healthy") != std::string::npos;
}

bool ContainsFailureToken(const std::string & value)
{
  std::string normalized(value);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  return normalized == "blocked" || normalized == "fault" || normalized == "error" ||
         normalized.rfind("error:", 0U) == 0U ||
         normalized.rfind("stale:", 0U) == 0U ||
         normalized.find("ready=false") != std::string::npos ||
         normalized.find("\"ready\":false") != std::string::npos ||
         normalized.find("\"ok\":false") != std::string::npos ||
         normalized.find("\"healthy\":false") != std::string::npos ||
         normalized.find("level=error") != std::string::npos ||
         normalized.find("state=blocked") != std::string::npos ||
         normalized.find("state=critical") != std::string::npos ||
         normalized.find("state=error") != std::string::npos ||
         normalized.find("state=fault") != std::string::npos ||
         normalized.find("state=stale") != std::string::npos ||
         normalized.find("state=unknown") != std::string::npos;
}

class BringupReadinessNode final : public rclcpp::Node
{
public:
  BringupReadinessNode()
  : rclcpp::Node(kNodeName), started_at_(now())
  {
    LoadParameters();
    ConfigurePublishers();
    ConfigureSubscriptions();

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {EvaluateAndPublish();});

    EvaluateAndPublish();
    RCLCPP_INFO(
      get_logger(),
      "Robot Savo bringup readiness started: role=%s mode=%s profile=%s",
      std::string(savo_bringup::ToString(role_)).c_str(),
      std::string(savo_bringup::ToString(mode_)).c_str(),
      std::string(savo_bringup::ToString(profile_)).c_str());
  }

private:
  struct Observation
  {
    bool seen{false};
    bool ready{false};
    bool failed{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string detail{"not_observed"};
  };

  void LoadParameters()
  {
    const auto role_text = declare_parameter<std::string>("host_role", "core");
    const auto mode_text = declare_parameter<std::string>("robot_mode", "safe_idle");
    const auto profile_text = declare_parameter<std::string>("bringup_profile", "lidar_only");
    const auto role = savo_bringup::ParseHostRole(role_text);
    const auto mode = savo_bringup::ParseRobotMode(mode_text);
    const auto profile = savo_bringup::ParseBringupProfile(profile_text);
    if (!role || !mode || !profile) {
      throw std::invalid_argument("invalid host_role, robot_mode, or bringup_profile");
    }
    role_ = *role;
    mode_ = *mode;
    profile_ = *profile;

    const bool d435_validated = declare_parameter<bool>("d435_voxel_validated", false);
    require_locked_geometry_ = declare_parameter<bool>("require_locked_geometry", true);
    allow_provisional_geometry_ = declare_parameter<bool>("allow_provisional_geometry", false);
    const auto validation = savo_bringup::ValidateCombination(
      role_, mode_, profile_, d435_validated, require_locked_geometry_,
      allow_provisional_geometry_);
    if (!validation.empty()) {
      throw std::invalid_argument(validation);
    }

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    startup_timeout_s_ = declare_parameter<double>("startup_timeout_s", 45.0);
    freshness_timeout_s_ = declare_parameter<double>("freshness_timeout_s", 3.0);
    if (publish_rate_hz_ <= 0.0 || startup_timeout_s_ <= 0.0 || freshness_timeout_s_ <= 0.0) {
      throw std::invalid_argument("readiness timing parameters must be positive");
    }

    require_geometry_ = declare_parameter<bool>("require_geometry",
        role_ != savo_bringup::HostRole::kEdge);
    geometry_policy_validated_ = declare_parameter<bool>(
      "geometry_policy_validated", require_locked_geometry_ || allow_provisional_geometry_);
    require_base_ = declare_parameter<bool>("require_base", false);
    require_control_ = declare_parameter<bool>("require_control", false);
    require_safety_ = declare_parameter<bool>("require_safety", false);
    require_lidar_ = declare_parameter<bool>("require_lidar", false);
    require_perception_ = declare_parameter<bool>("require_perception", false);
    require_localization_ = declare_parameter<bool>("require_localization", false);
    require_power_ = declare_parameter<bool>("require_power", false);
    require_supervisor_ = declare_parameter<bool>("require_supervisor", false);
    require_supervisor_authority_ = declare_parameter<bool>("require_supervisor_authority", false);
    require_mapping_ = declare_parameter<bool>("require_mapping", false);
    require_navigation_ = declare_parameter<bool>("require_navigation", false);
    require_active_release_ = declare_parameter<bool>("require_active_release", false);
    active_release_verified_ = declare_parameter<bool>("active_release_verified", false);
    require_map_context_ = declare_parameter<bool>("require_map_context", false);
    require_goal_admission_ = declare_parameter<bool>("require_goal_admission", false);
    require_bridge_ = declare_parameter<bool>("require_bridge", false);
    require_realsense_ = declare_parameter<bool>("require_realsense", false);
    require_vo_ = declare_parameter<bool>("require_vo", false);
    require_speech_ = declare_parameter<bool>("require_speech", false);
    require_ui_ = declare_parameter<bool>("require_ui", false);
    require_obstacle_cloud_ = declare_parameter<bool>("require_obstacle_cloud", false);

    const std::string default_namespace =
      "/savo_bringup/" + std::string(savo_bringup::ToString(role_));
    output_namespace_ = declare_parameter<std::string>("output_namespace", default_namespace);
    while (output_namespace_.size() > 1U && output_namespace_.back() == '/') {
      output_namespace_.pop_back();
    }
    if (output_namespace_.empty() || output_namespace_.front() != '/') {
      throw std::invalid_argument("output_namespace must be an absolute ROS namespace");
    }
  }

  void ConfigurePublishers()
  {
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    state_publisher_ = create_publisher<String>(output_namespace_ + "/state", state_qos);
    ready_publisher_ = create_publisher<Bool>(output_namespace_ + "/ready", state_qos);
    heartbeat_publisher_ = create_publisher<UInt64>(output_namespace_ + "/heartbeat", stream_qos);
    diagnostics_publisher_ = create_publisher<DiagnosticArray>(
      output_namespace_ + "/diagnostics", stream_qos);
  }

  void SubscribeString(
    const std::string & key, const std::string & parameter, const std::string & topic,
    const bool availability_only = false)
  {
    const auto resolved_topic = declare_parameter<std::string>(parameter, topic);
    string_subscriptions_.push_back(create_subscription<String>(
      resolved_topic, rclcpp::QoS(10).reliable(),
        [this, key, availability_only](const String::SharedPtr message) {
          const bool failed = ContainsFailureToken(message->data);
          Mark(key, availability_only ? !failed : ContainsReadyToken(message->data), failed,
          message->data);
      }));
  }

  void SubscribeBool(
    const std::string & key, const std::string & parameter, const std::string & topic,
    const bool invert = false)
  {
    const auto resolved_topic = declare_parameter<std::string>(parameter, topic);
    bool_subscriptions_.push_back(create_subscription<Bool>(
      resolved_topic, rclcpp::QoS(10).reliable(),
        [this, key, invert](const Bool::SharedPtr message) {
          const bool ready = invert ? !message->data : message->data;
          Mark(key, ready, false, BoolText(message->data));
      }));
  }

  void SubscribeCounter(
    const std::string & key, const std::string & parameter, const std::string & topic)
  {
    const auto resolved_topic = declare_parameter<std::string>(parameter, topic);
    counter_subscriptions_.push_back(create_subscription<UInt64>(
      resolved_topic, rclcpp::QoS(10).reliable(),
        [this, key](const UInt64::SharedPtr message) {
          Mark(key, true, false, std::to_string(message->data));
      }));
  }

  void ConfigureSubscriptions()
  {
    if (require_base_) {
      SubscribeString("base", "base_state_topic", "/savo_base/base_state", true);
    }
    if (require_control_) {
      SubscribeString("control", "control_state_topic", "/savo_control/mode_state", true);
    }
    if (require_safety_) {
      SubscribeString("safety_state", "safety_state_topic", "/savo_perception/safety_state", true);
      SubscribeBool("safety_clear", "safety_stop_topic", "/safety/stop", true);
    }
    if (require_lidar_) {
      SubscribeString("lidar_heartbeat", "lidar_heartbeat_topic", "/savo_lidar/heartbeat", true);
    }
    if (require_perception_) {
      SubscribeString("perception_heartbeat", "perception_heartbeat_topic",
          "/savo_perception/heartbeat", true);
    }
    if (require_localization_) {
      SubscribeString("localization", "localization_health_topic", "/savo_localization/health");
      SubscribeString("localization_heartbeat", "localization_heartbeat_topic",
          "/savo_localization/heartbeat", true);
    }
    if (require_power_) {
      const std::string topic = role_ == savo_bringup::HostRole::kEdge ?
        "/savo_power/edge/ups" : "/savo_power/health";
      SubscribeString("power", "power_status_topic", topic, true);
    }
    if (require_supervisor_) {
      SubscribeString("supervisor_heartbeat", "supervisor_heartbeat_topic",
          "/savo_supervisor/heartbeat", true);
      if (require_supervisor_authority_) {
        SubscribeBool("supervisor_authority", "supervisor_ready_topic",
            "/savo_supervisor/system_ready");
      }
    }
    if (require_mapping_) {
      SubscribeString("mapping", "mapping_readiness_topic", "/savo_mapping/readiness");
    }
    if (require_navigation_) {
      SubscribeString("navigation", "navigation_readiness_topic", "/savo_nav/readiness");
      SubscribeCounter("navigation_heartbeat", "navigation_heartbeat_topic", "/savo_nav/heartbeat");
    }
    if (require_map_context_) {
      SubscribeString("map_context", "map_context_status_topic", "/savo_nav/map_context/status");
      SubscribeCounter("map_context_heartbeat", "map_context_heartbeat_topic",
          "/savo_nav/map_context/heartbeat");
    }
    if (require_goal_admission_) {
      SubscribeString("goal_admission", "goal_admission_state_topic",
          "/savo_nav/goal_admission/state", true);
    }
    if (require_bridge_) {
      SubscribeBool("bridge", "bridge_readiness_topic", "/savo_bridge/readiness");
      SubscribeCounter("bridge_heartbeat", "bridge_heartbeat_topic", "/savo_bridge/heartbeat");
    }
    if (require_realsense_) {
      SubscribeString("realsense", "realsense_status_topic", "/realsense/status");
    }
    if (require_vo_) {
      SubscribeString("vo", "vo_health_topic", "/vo/health");
    }
    if (require_speech_) {
      SubscribeString("speech", "speech_readiness_topic", "/savo_speech/readiness");
      SubscribeCounter("speech_heartbeat", "speech_heartbeat_topic", "/savo_speech/heartbeat");
    }
    if (require_ui_) {
      SubscribeString("ui", "ui_status_topic", "/savo_ui/status_text", true);
    }
    if (require_obstacle_cloud_) {
      SubscribeBool("obstacle_cloud", "obstacle_cloud_health_topic",
          "/savo_perception/obstacle_cloud/health");
      SubscribeString(
        "obstacle_cloud_heartbeat", "obstacle_cloud_heartbeat_topic",
        "/savo_perception/obstacle_cloud/heartbeat", true);
    }
  }

  void Mark(const std::string & key, const bool ready, const bool failed, std::string detail)
  {
    auto & observation = observations_[key];
    observation.seen = true;
    observation.ready = ready;
    observation.failed = failed;
    observation.stamp = now();
    observation.detail = std::move(detail);
  }

  bool Fresh(const Observation & observation) const
  {
    return observation.seen && (now() - observation.stamp).seconds() <= freshness_timeout_s_;
  }

  void AddStatic(
    std::vector<savo_bringup::DependencyStatus> & dependencies,
    const std::string & name, const bool required, const bool ready,
    const savo_bringup::ReadinessState waiting_state) const
  {
    if (!required) {
      return;
    }
    dependencies.push_back({
        name, waiting_state, required, true, true, ready, false,
        ready ? "validated" : "not_validated"});
  }

  void AddDynamic(
    std::vector<savo_bringup::DependencyStatus> & dependencies,
    const std::string & name, const bool required,
    const savo_bringup::ReadinessState waiting_state,
    const std::string & heartbeat = "") const
  {
    const auto found = observations_.find(name);
    const Observation empty;
    const Observation & status = found == observations_.end() ? empty : found->second;
    bool observed = status.seen;
    bool fresh = Fresh(status);
    std::string detail = status.detail;
    if (!heartbeat.empty()) {
      const auto heartbeat_found = observations_.find(heartbeat);
      const Observation & heartbeat_status =
        heartbeat_found == observations_.end() ? empty : heartbeat_found->second;
      observed = observed && heartbeat_status.seen;
      fresh = Fresh(heartbeat_status);
      if (!heartbeat_status.seen) {
        detail = "heartbeat_not_observed";
      } else if (!fresh) {
        detail = "heartbeat_stale";
      }
    }
    dependencies.push_back({
        name, waiting_state, required, observed, fresh, status.ready,
        status.failed, detail});
  }

  static std::string Join(const std::vector<std::string> & values)
  {
    std::ostringstream stream;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0U) {
        stream << ',';
      }
      stream << values[index];
    }
    return stream.str();
  }

  void EvaluateAndPublish()
  {
    using savo_bringup::ReadinessState;
    std::vector<savo_bringup::DependencyStatus> dependencies;
    AddStatic(dependencies, "geometry", require_geometry_, geometry_policy_validated_,
      ReadinessState::kValidatingGeometry);
    AddDynamic(dependencies, "base", require_base_, ReadinessState::kWaitingForDependencies);
    AddDynamic(dependencies, "control", require_control_, ReadinessState::kWaitingForSafety);
    AddDynamic(dependencies, "safety_state", require_safety_, ReadinessState::kWaitingForSafety);
    AddDynamic(dependencies, "safety_clear", require_safety_, ReadinessState::kWaitingForSafety);
    AddDynamic(dependencies, "lidar_heartbeat", require_lidar_,
        ReadinessState::kWaitingForDependencies);
    AddDynamic(dependencies, "perception_heartbeat", require_perception_,
        ReadinessState::kWaitingForSafety);
    AddDynamic(dependencies, "localization", require_localization_,
      ReadinessState::kWaitingForLocalization, "localization_heartbeat");
    AddDynamic(dependencies, "power", require_power_, ReadinessState::kWaitingForDependencies);
    AddDynamic(dependencies, "supervisor_heartbeat", require_supervisor_,
        ReadinessState::kWaitingForSafety);
    AddDynamic(dependencies, "supervisor_authority", require_supervisor_authority_,
        ReadinessState::kWaitingForSafety);
    AddDynamic(dependencies, "mapping", require_mapping_, ReadinessState::kWaitingForNavigation);
    AddStatic(dependencies, "active_release", require_active_release_, active_release_verified_,
      ReadinessState::kWaitingForMapContext);
    AddDynamic(dependencies, "map_context", require_map_context_,
      ReadinessState::kWaitingForMapContext, "map_context_heartbeat");
    AddDynamic(dependencies, "navigation", require_navigation_,
      ReadinessState::kWaitingForNavigation, "navigation_heartbeat");
    AddDynamic(dependencies, "goal_admission", require_goal_admission_,
        ReadinessState::kWaitingForNavigation);
    AddDynamic(dependencies, "bridge", require_bridge_, ReadinessState::kWaitingForDependencies,
        "bridge_heartbeat");
    AddDynamic(dependencies, "realsense", require_realsense_,
        ReadinessState::kWaitingForDependencies);
    AddDynamic(dependencies, "vo", require_vo_, ReadinessState::kWaitingForDependencies);
    AddDynamic(dependencies, "speech", require_speech_, ReadinessState::kWaitingForDependencies,
        "speech_heartbeat");
    AddDynamic(dependencies, "ui", require_ui_, ReadinessState::kWaitingForDependencies);
    AddDynamic(dependencies, "obstacle_cloud", require_obstacle_cloud_,
      ReadinessState::kWaitingForNavigation, "obstacle_cloud_heartbeat");

    const bool startup_expired = (now() - started_at_).seconds() > startup_timeout_s_;
    const auto decision = savo_bringup::EvaluateReadiness(
      dependencies, true, startup_expired);
    const std::string state(savo_bringup::ToString(decision.state));
    std::string reason;
    if (!decision.failed.empty()) {
      reason = "failed=" + Join(decision.failed);
    } else if (startup_expired && !decision.missing.empty()) {
      reason = "startup_timeout_missing=" + Join(decision.missing);
    } else if (!decision.missing.empty()) {
      reason = "waiting_for=" + Join(decision.missing);
    } else if (!decision.degraded.empty()) {
      reason = "optional_degraded=" + Join(decision.degraded);
    } else {
      reason = "all_required_bringup_dependencies_ready";
    }

    String state_message;
    state_message.data =
      "state=" + state + ";ready=" + BoolText(decision.ready) +
      ";role=" + std::string(savo_bringup::ToString(role_)) +
      ";mode=" + std::string(savo_bringup::ToString(mode_)) +
      ";profile=" + std::string(savo_bringup::ToString(profile_)) +
      ";reason=" + reason;
    state_publisher_->publish(state_message);

    Bool ready_message;
    ready_message.data = decision.ready;
    ready_publisher_->publish(ready_message);
    UInt64 heartbeat_message;
    heartbeat_message.data = ++heartbeat_count_;
    heartbeat_publisher_->publish(heartbeat_message);
    PublishDiagnostics(state, reason, decision);

    if (state != last_state_ || reason != last_reason_) {
      RCLCPP_INFO(
        get_logger(), "Bringup readiness changed: state=%s reason=%s",
        state.c_str(), reason.c_str());
      last_state_ = state;
      last_reason_ = reason;
    }
  }

  void PublishDiagnostics(
    const std::string & state, const std::string & reason,
    const savo_bringup::ReadinessDecision & decision)
  {
    DiagnosticArray array;
    array.header.stamp = now();
    DiagnosticStatus status;
    status.name = "Robot Savo bringup readiness";
    status.hardware_id = std::string(savo_bringup::ToString(role_));
    status.level = decision.state == savo_bringup::ReadinessState::kBlocked ?
      DiagnosticStatus::ERROR :
      (decision.state ==
      savo_bringup::ReadinessState::kReady ? DiagnosticStatus::OK : DiagnosticStatus::WARN);
    status.message = reason;
    const std::vector<std::pair<std::string, std::string>> values{
      {"state", state},
      {"ready", BoolText(decision.ready)},
      {"robot_mode", std::string(savo_bringup::ToString(mode_))},
      {"bringup_profile", std::string(savo_bringup::ToString(profile_))},
      {"missing", Join(decision.missing)},
      {"failed", Join(decision.failed)},
      {"degraded", Join(decision.degraded)},
    };
    for (const auto & [key, value] : values) {
      KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    }
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  savo_bringup::HostRole role_{savo_bringup::HostRole::kCore};
  savo_bringup::RobotMode mode_{savo_bringup::RobotMode::kSafeIdle};
  savo_bringup::BringupProfile profile_{savo_bringup::BringupProfile::kLidarOnly};
  bool require_locked_geometry_{true};
  bool allow_provisional_geometry_{false};
  bool require_geometry_{false};
  bool geometry_policy_validated_{false};
  bool require_base_{false};
  bool require_control_{false};
  bool require_safety_{false};
  bool require_lidar_{false};
  bool require_perception_{false};
  bool require_localization_{false};
  bool require_power_{false};
  bool require_supervisor_{false};
  bool require_supervisor_authority_{false};
  bool require_mapping_{false};
  bool require_navigation_{false};
  bool require_active_release_{false};
  bool active_release_verified_{false};
  bool require_map_context_{false};
  bool require_goal_admission_{false};
  bool require_bridge_{false};
  bool require_realsense_{false};
  bool require_vo_{false};
  bool require_speech_{false};
  bool require_ui_{false};
  bool require_obstacle_cloud_{false};
  double publish_rate_hz_{2.0};
  double startup_timeout_s_{45.0};
  double freshness_timeout_s_{3.0};
  rclcpp::Time started_at_;
  std::uint64_t heartbeat_count_{0U};
  std::string output_namespace_;
  std::map<std::string, Observation> observations_;
  std::string last_state_;
  std::string last_reason_;

  rclcpp::Publisher<String>::SharedPtr state_publisher_;
  rclcpp::Publisher<Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<UInt64>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::vector<rclcpp::Subscription<String>::SharedPtr> string_subscriptions_;
  std::vector<rclcpp::Subscription<Bool>::SharedPtr> bool_subscriptions_;
  std::vector<rclcpp::Subscription<UInt64>::SharedPtr> counter_subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<BringupReadinessNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger(kNodeName), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
