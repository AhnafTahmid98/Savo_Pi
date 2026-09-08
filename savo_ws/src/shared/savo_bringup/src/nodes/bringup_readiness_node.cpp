// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_bringup/bringup_contract.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/rclcpp.hpp"
#include "savo_msgs/msg/semantic_interruption_status.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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

std::string JsonEscape(const std::string & value)
{
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

std::optional<savo_bringup::QualityLevel> ExtractQuality(const std::string & payload)
{
  std::string compact(payload);
  compact.erase(
    std::remove_if(compact.begin(), compact.end(),
    [](const unsigned char character) {return std::isspace(character) != 0;}),
    compact.end());
  for (const auto quality : {
      savo_bringup::QualityLevel::kBelowMinimum,
      savo_bringup::QualityLevel::kMinimum,
      savo_bringup::QualityLevel::kGood,
      savo_bringup::QualityLevel::kExcellent})
  {
    const std::string value(savo_bringup::ToString(quality));
    if (compact.find("\"quality\":\"" + value + "\"") != std::string::npos ||
      compact.find("\"rate_quality\":\"" + value + "\"") != std::string::npos)
    {
      return quality;
    }
  }
  return std::nullopt;
}

bool ContainsReadyToken(const std::string & value)
{
  std::string normalized(value);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  normalized.erase(
    std::remove_if(normalized.begin(), normalized.end(),
    [](const unsigned char character) {return std::isspace(character) != 0;}),
    normalized.end());
  return normalized == "ready" || normalized.rfind("ok:", 0U) == 0U ||
         normalized.find("ready=true") != std::string::npos ||
         normalized.find("\"ready\":true") != std::string::npos ||
         normalized.find("\"ok\":true") != std::string::npos ||
         normalized.find("\"overall_ok\":true") != std::string::npos ||
         normalized.find("\"healthy\":true") != std::string::npos ||
         normalized.find("\"state\":\"ready\"") != std::string::npos ||
         normalized.find("status=ok") != std::string::npos ||
         normalized.find("synchronized=true") != std::string::npos ||
         normalized.find("level=ok") != std::string::npos ||
         normalized.find("healthy") != std::string::npos;
}

bool ContainsFailureToken(const std::string & value)
{
  std::string normalized(value);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
    [](const unsigned char character) {return static_cast<char>(std::tolower(character));});
  normalized.erase(
    std::remove_if(normalized.begin(), normalized.end(),
    [](const unsigned char character) {return std::isspace(character) != 0;}),
    normalized.end());
  return normalized == "blocked" || normalized == "fault" || normalized == "error" ||
         normalized.rfind("error:", 0U) == 0U ||
         normalized.rfind("stale:", 0U) == 0U ||
         normalized.find("ready=false") != std::string::npos ||
         normalized.find("\"ready\":false") != std::string::npos ||
         normalized.find("\"ok\":false") != std::string::npos ||
         normalized.find("\"overall_ok\":false") != std::string::npos ||
         normalized.find("\"healthy\":false") != std::string::npos ||
         normalized.find("level=error") != std::string::npos ||
         normalized.find("status=error") != std::string::npos ||
         normalized.find("status=stale") != std::string::npos ||
         normalized.find("status=fault") != std::string::npos ||
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
  : rclcpp::Node(kNodeName), started_at_(now()),
    stage_entered_at_(std::chrono::steady_clock::now())
  {
    LoadParameters();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    ConfigurePublishers();
    ConfigureSubscriptions();

    const double evaluation_rate = startup_enabled_ ? startup_evaluation_rate_hz_ :
      publish_rate_hz_;
    const auto period = std::chrono::duration<double>(1.0 / evaluation_rate);
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
    std::optional<savo_bringup::QualityLevel> quality;
  };

  struct Stage
  {
    Stage(
      std::string stage_name,
      savo_bringup::StartupStageTiming stage_timing,
      std::vector<std::string> stage_dependencies)
    : name(std::move(stage_name)),
      dependencies(std::move(stage_dependencies)),
      tracker(stage_timing)
    {
    }

    std::string name;
    std::vector<std::string> dependencies;
    savo_bringup::StartupStageTracker tracker;
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
    infrastructure_parent_frame_ = declare_parameter<std::string>(
      "infrastructure.parent_frame", "base_footprint");
    infrastructure_child_frame_ = declare_parameter<std::string>(
      "infrastructure.child_frame", "base_link");
    if (infrastructure_parent_frame_.empty() || infrastructure_child_frame_.empty()) {
      throw std::invalid_argument("infrastructure TF frames must be non-empty");
    }
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
    require_head_ = declare_parameter<bool>("require_head", false);
    require_locations_ = declare_parameter<bool>("require_locations", false);
    require_semantic_ = declare_parameter<bool>("require_semantic", false);

    startup_enabled_ = declare_parameter<bool>("startup.enabled", false);
    startup_schema_version_ = declare_parameter<int>("startup.schema_version", 1);
    startup_evaluation_rate_hz_ = declare_parameter<double>(
      "startup.evaluation_rate_hz", 5.0);
    startup_status_publish_rate_hz_ = declare_parameter<double>(
      "startup.status_publish_rate_hz", 2.0);
    startup_observation_freshness_s_ = declare_parameter<double>(
      "startup.observation_freshness_s", freshness_timeout_s_);
    if (startup_schema_version_ != 1 || startup_evaluation_rate_hz_ <= 0.0 ||
      startup_status_publish_rate_hz_ <= 0.0 || startup_observation_freshness_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid staged-startup configuration");
    }

    const std::string default_namespace =
      "/savo_bringup/" + std::string(savo_bringup::ToString(role_));
    output_namespace_ = declare_parameter<std::string>("output_namespace", default_namespace);
    while (output_namespace_.size() > 1U && output_namespace_.back() == '/') {
      output_namespace_.pop_back();
    }
    if (output_namespace_.empty() || output_namespace_.front() != '/') {
      throw std::invalid_argument("output_namespace must be an absolute ROS namespace");
    }
    if (startup_enabled_) {
      BuildStartupStages();
    }
  }

  savo_bringup::StartupStageTiming LoadStageTiming(const std::string & name)
  {
    const std::string role = std::string(savo_bringup::ToString(role_));
    const std::string prefix = "startup." + role + "." + name + ".";
    savo_bringup::StartupStageTiming timing;
    timing.minimum_settle_s = declare_parameter<double>(prefix + "minimum_settle_s", 0.0);
    timing.stable_ready_s = declare_parameter<double>(prefix + "stable_ready_s", 1.0);
    timing.startup_timeout_s = declare_parameter<double>(prefix + "startup_timeout_s", 10.0);
    if (!savo_bringup::ValidateStartupStageTiming(timing)) {
      throw std::invalid_argument("invalid startup timing for stage " + name);
    }
    return timing;
  }

  void AddStage(const std::string & name, std::vector<std::string> dependencies)
  {
    stages_.emplace_back(name, LoadStageTiming(name), std::move(dependencies));
  }

  void BuildStartupStages()
  {
    if (role_ == savo_bringup::HostRole::kCore) {
      AddStage("infrastructure", {});
      std::vector<std::string> hardware;
      if (require_power_) {
        hardware.push_back("power");
      }
      if (require_localization_) {
        hardware.push_back("imu_state");
        hardware.push_back("wheel_state");
      }
      if (require_perception_) {
        hardware.push_back("perception_heartbeat");
      }
      if (require_lidar_) {
        hardware.push_back("lidar_heartbeat");
      }
      if (!hardware.empty()) {
        AddStage("hardware", hardware);
      }

      std::vector<std::string> motion_safety;
      if (require_base_) {
        motion_safety.push_back("base");
      }
      if (require_control_) {
        motion_safety.push_back("control_stop");
      }
      if (require_safety_) {
        motion_safety.push_back("safety_state");
        motion_safety.push_back("safety_clear");
      }
      if (!motion_safety.empty()) {
        AddStage("motion_safety", motion_safety);
      }

      std::vector<std::string> sensors;
      if (require_localization_) {
        sensors.push_back("imu_state");
        sensors.push_back("wheel_state");
      }
      if (require_perception_) {
        sensors.push_back("perception_health");
      }
      if (require_lidar_) {
        sensors.push_back("lidar_health");
      }
      if (!sensors.empty()) {
        AddStage("sensor_stabilization", sensors);
      }
      if (require_localization_) {
        AddStage("localization", {"localization"});
      }
      if (require_supervisor_) {
        AddStage("supervisor", {"supervisor_startup", "supervisor_safe_unarmed"});
      }
      if (require_navigation_) {
        AddStage("navigation", {"navigation_startup"});
      }
      if (require_mapping_) {
        AddStage("slam_foundation", {"mapping"});
        AddStage("mapping_runtime", {"mapping"});
      }
      if (require_head_) {
        AddStage("head", {"head"});
      }
      if (require_locations_ || require_semantic_) {
        std::vector<std::string> semantic_locations;
        if (require_locations_) {
          semantic_locations.push_back("locations");
        }
        if (require_semantic_) {
          semantic_locations.push_back("semantic");
        }
        AddStage("semantic_locations", semantic_locations);
      }
      std::vector<std::string> complete;
      if (require_control_) {
        complete.push_back("control_stop");
      }
      if (require_supervisor_) {
        complete.push_back("supervisor_startup");
        complete.push_back("supervisor_safe_unarmed");
      }
      AddStage("complete", complete);
    } else if (role_ == savo_bringup::HostRole::kEdge) {
      AddStage("infrastructure", require_power_ ? std::vector<std::string>{"power"} :
        std::vector<std::string>{});
      if (require_realsense_) {
        AddStage("realsense", {"realsense"});
      }
      if (require_vo_) {
        AddStage("vo", {"vo"});
      }
      if (require_obstacle_cloud_) {
        AddStage("obstacle_cloud", {"obstacle_cloud"});
      }
      if (require_bridge_) {
        AddStage("bridge", {"bridge_heartbeat"});
      }
      std::vector<std::string> apps;
      if (require_speech_) {
        apps.push_back("speech");
      }
      if (require_ui_) {
        apps.push_back("ui");
      }
      if (!apps.empty()) {
        AddStage("optional_apps", apps);
      }
      AddStage("complete", {});
    } else {
      throw std::invalid_argument("staged startup cannot coordinate host_role=all");
    }
    if (stages_.empty()) {
      throw std::invalid_argument("staged startup has no stages");
    }
    const auto generation = std::chrono::steady_clock::now().time_since_epoch().count();
    startup_id_ = std::string(savo_bringup::ToString(role_)) + "-" +
      std::to_string(generation);
  }

  void ConfigurePublishers()
  {
    auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    state_publisher_ = create_publisher<String>(output_namespace_ + "/state", state_qos);
    startup_status_publisher_ = create_publisher<String>(
      output_namespace_ + "/startup_status", state_qos);
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
          message->data, ExtractQuality(message->data));
      }));
  }

  void SubscribeExactString(
    const std::string & key, const std::string & parameter, const std::string & topic,
    const std::string & expected)
  {
    const auto resolved_topic = declare_parameter<std::string>(parameter, topic);
    string_subscriptions_.push_back(create_subscription<String>(
      resolved_topic, rclcpp::QoS(10).reliable(),
        [this, key, expected](const String::SharedPtr message) {
          Mark(key, message->data == expected, false, message->data);
        }));
  }

  void SubscribeSupervisorState()
  {
    const auto topic = declare_parameter<std::string>(
      "supervisor_state_topic", "/savo_supervisor/state_summary");
    string_subscriptions_.push_back(create_subscription<String>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](const String::SharedPtr message) {
          const bool unarmed =
          message->data.find("\"system_armed\":false") != std::string::npos;
          const bool latch_clear =
          message->data.find("\"fault_latched\":false") != std::string::npos;
          const bool mission_idle =
          message->data.find("\"active_operation\":\"NONE\"") != std::string::npos ||
          message->data.find("\"active_operation\":\"none\"") != std::string::npos;
          Mark(
            "supervisor_safe_unarmed", unarmed && latch_clear && mission_idle,
            !latch_clear, message->data, ExtractQuality(message->data));
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
      SubscribeExactString(
        "control_stop", "control_mode_state_topic", "/savo_control/mode_state", "STOP");
    }
    if (require_safety_) {
      SubscribeString("safety_state", "safety_state_topic", "/savo_perception/safety_state", true);
      SubscribeBool("safety_clear", "safety_stop_topic", "/safety/stop", true);
    }
    if (require_lidar_) {
      SubscribeString("lidar_heartbeat", "lidar_heartbeat_topic", "/savo_lidar/heartbeat", true);
      SubscribeString("lidar_health", "lidar_health_topic", "/savo_lidar/health");
    }
    if (require_perception_) {
      SubscribeString("perception_heartbeat", "perception_heartbeat_topic",
          "/savo_perception/heartbeat", true);
      SubscribeString(
        "perception_health", "perception_health_topic", "/savo_perception/range_health");
    }
    if (require_localization_) {
      SubscribeString("imu_state", "imu_state_topic", "/savo_localization/imu_state");
      SubscribeString(
        "wheel_state", "wheel_state_topic", "/savo_localization/wheel_odom_state");
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
      SubscribeBool("supervisor_startup", "supervisor_startup_ready_topic",
          "/savo_supervisor/startup_ready");
      SubscribeSupervisorState();
    }
    if (require_mapping_) {
      SubscribeString("mapping", "mapping_readiness_topic", "/savo_mapping/readiness");
    }
    if (require_navigation_) {
      SubscribeString(
        "navigation_startup", "navigation_startup_readiness_topic",
        "/savo_nav/startup_readiness");
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
    if (require_head_) {
      SubscribeString("head", "head_status_topic", "/savo_head/status");
    }
    if (require_locations_) {
      SubscribeString("locations", "locations_status_topic", "/savo_locations/status");
    }
    if (require_semantic_) {
      SubscribeSemanticStatus();
    }
  }

  void SubscribeSemanticStatus()
  {
    const auto topic = declare_parameter<std::string>(
      "semantic_status_topic", "/savo_mapping/semantic_interruption/status");
    semantic_subscription_ =
      create_subscription<savo_msgs::msg::SemanticInterruptionStatus>(
        topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const savo_msgs::msg::SemanticInterruptionStatus::SharedPtr message) {
        const bool contract_valid = message->contract_version ==
        savo_msgs::msg::SemanticInterruptionStatus::CONTRACT_VERSION;
        const bool failed = message->state ==
        savo_msgs::msg::SemanticInterruptionStatus::STATE_FAILED;
        Mark(
            "semantic", contract_valid && message->startup_ready && !failed,
            !contract_valid || failed,
            message->state_text + ":" + message->reason,
            contract_valid && !failed ?
            std::optional<savo_bringup::QualityLevel>{
          savo_bringup::QualityLevel::kMinimum} :
            std::optional<savo_bringup::QualityLevel>{
          savo_bringup::QualityLevel::kBelowMinimum});
        });
  }

  void Mark(
    const std::string & key, const bool ready, const bool failed, std::string detail,
    std::optional<savo_bringup::QualityLevel> quality = std::nullopt)
  {
    auto & observation = observations_[key];
    observation.seen = true;
    observation.ready = ready;
    observation.failed = failed;
    observation.stamp = now();
    observation.detail = std::move(detail);
    observation.quality = quality;
  }

  bool Fresh(const Observation & observation) const
  {
    const double timeout = startup_enabled_ ? startup_observation_freshness_s_ :
      freshness_timeout_s_;
    return observation.seen && (now() - observation.stamp).seconds() <= timeout;
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

  bool ObservationReady(const std::string & key, std::string & reason) const
  {
    const auto found = observations_.find(key);
    if (found == observations_.end() || !found->second.seen) {
      reason = key + "_not_observed";
      return false;
    }
    if (!Fresh(found->second)) {
      reason = key + "_stale";
      return false;
    }
    if (found->second.failed || !found->second.ready) {
      reason = key + ":" + found->second.detail;
      return false;
    }
    return true;
  }

  savo_bringup::StartupStageInput StageInput(const Stage & stage) const
  {
    savo_bringup::StartupStageInput input;
    input.processes_started = true;
    input.dependencies_ready = true;
    std::vector<savo_bringup::QualityLevel> quality;
    if (stage.name == "infrastructure" && require_geometry_) {
      const bool model_tf_available = tf_buffer_->canTransform(
        infrastructure_parent_frame_, infrastructure_child_frame_, tf2::TimePointZero);
      input.dependencies_ready = geometry_policy_validated_ && model_tf_available;
      input.processes_started = geometry_policy_validated_;
      input.reason = !geometry_policy_validated_ ? "geometry_not_validated" :
        !model_tf_available ? "base_footprint_to_base_link_tf_unavailable" :
        "geometry_and_model_tf_validated";
    }
    for (const auto & key : stage.dependencies) {
      const auto found = observations_.find(key);
      if (found == observations_.end() || !found->second.seen) {
        input.processes_started = false;
      }
      std::string dependency_reason;
      if (!ObservationReady(key, dependency_reason)) {
        input.dependencies_ready = false;
        if (input.reason == "waiting_for_processes") {
          input.reason = dependency_reason;
        }
      } else if (found->second.quality) {
        quality.push_back(*found->second.quality);
      }
    }
    input.quality = savo_bringup::WorstRequiredQuality(quality, input.dependencies_ready);
    if (input.dependencies_ready) {
      input.reason = "all_required_stage_dependencies_ready";
    }
    return input;
  }

  static std::string JsonArray(const std::vector<std::string> & values)
  {
    std::ostringstream output;
    output << '[';
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0U) {
        output << ',';
      }
      output << '"' << JsonEscape(values[index]) << '"';
    }
    output << ']';
    return output.str();
  }

  void PublishStartupStatus(
    const std::string & stage_name,
    const savo_bringup::StartupStageDecision & decision,
    const double elapsed_s,
    const std::string & reason,
    const std::vector<std::string> & pending)
  {
    const auto publish_time = std::chrono::steady_clock::now();
    const std::string status_key = stage_name + ":" +
      std::string(savo_bringup::ToString(decision.state)) + ":" + reason + ":" +
      BoolText(startup_complete_) + ":" + BoolText(runtime_failed_);
    const bool changed = status_key != last_startup_status_key_;
    const double publish_age_s = startup_status_published_ ?
      std::chrono::duration<double>(
      publish_time - last_startup_status_published_at_).count() :
      std::numeric_limits<double>::infinity();
    if (!changed && publish_age_s < 1.0 / startup_status_publish_rate_hz_) {
      return;
    }
    startup_status_published_ = true;
    last_startup_status_published_at_ = publish_time;
    last_startup_status_key_ = status_key;

    std::vector<std::string> enabled;
    enabled.reserve(stages_.size());
    for (const auto & stage : stages_) {
      enabled.push_back(stage.name);
    }
    std::ostringstream output;
    output << std::fixed << std::setprecision(3)
           << "{\"schema_name\":\"savo_bringup_startup_status\""
           << ",\"schema_version\":" << startup_schema_version_
           << ",\"startup_id\":\"" << JsonEscape(startup_id_) << "\""
           << ",\"host_role\":\"" << savo_bringup::ToString(role_) << "\""
           << ",\"profile\":\"" << savo_bringup::ToString(profile_) << "\""
           << ",\"sequence\":" << ++startup_sequence_
           << ",\"current_stage\":\"" << JsonEscape(stage_name) << "\""
           << ",\"stage_index\":" << current_stage_
           << ",\"state\":\"" << savo_bringup::ToString(decision.state) << "\""
           << ",\"launch_released\":true"
           << ",\"process_started\":" << BoolText(decision.state !=
      savo_bringup::StartupStageState::kStarting)
           << ",\"startup_ready\":" << BoolText(startup_complete_ && !runtime_failed_)
           << ",\"stable_for_s\":" << decision.stable_for_s
           << ",\"elapsed_s\":" << elapsed_s
           << ",\"quality\":\"" << savo_bringup::ToString(decision.quality) << "\""
           << ",\"quality_reason\":\"" << JsonEscape(reason) << "\""
           << ",\"reason\":\"" << JsonEscape(reason) << "\""
           << ",\"pending_dependencies\":" << JsonArray(pending)
           << ",\"failed_dependencies\":" << (decision.failed ? JsonArray(pending) : "[]")
           << ",\"completed_stages\":" << JsonArray(completed_stages_)
           << ",\"enabled_stages\":" << JsonArray(enabled) << '}';
    String message;
    message.data = output.str();
    startup_status_publisher_->publish(message);

    Bool ready_message;
    ready_message.data = startup_complete_ && !runtime_failed_;
    ready_publisher_->publish(ready_message);
    String legacy_state;
    legacy_state.data = "state=" + std::string(savo_bringup::ToString(decision.state)) +
      ";ready=" + BoolText(ready_message.data) + ";role=" +
      std::string(savo_bringup::ToString(role_)) + ";mode=" +
      std::string(savo_bringup::ToString(mode_)) + ";profile=" +
      std::string(savo_bringup::ToString(profile_)) + ";reason=" + reason;
    state_publisher_->publish(legacy_state);
  }

  void EvaluateStartupAndPublish()
  {
    if (current_stage_ >= stages_.size()) {
      startup_complete_ = true;
      runtime_failed_ = false;
      std::vector<std::string> lost;
      for (const auto & stage : stages_) {
        const auto input = StageInput(stage);
        if (!input.dependencies_ready) {
          runtime_failed_ = true;
          lost.push_back(stage.name + ":" + input.reason);
        }
      }
      savo_bringup::StartupStageDecision decision;
      decision.state = runtime_failed_ ? savo_bringup::StartupStageState::kFailed :
        savo_bringup::StartupStageState::kReady;
      decision.ready = !runtime_failed_;
      decision.failed = runtime_failed_;
      decision.quality = runtime_failed_ ? savo_bringup::QualityLevel::kBelowMinimum :
        savo_bringup::QualityLevel::kMinimum;
      const std::string reason = runtime_failed_ ?
        "established_dependency_lost:" + Join(lost) : "bringup_complete_safe_unarmed";
      PublishStartupStatus("complete", decision, 0.0, reason, lost);
      PublishStartupHeartbeat();
      return;
    }

    std::vector<std::string> lost;
    for (std::size_t index = 0; index < current_stage_; ++index) {
      const auto established = StageInput(stages_[index]);
      if (!established.dependencies_ready) {
        lost.push_back(stages_[index].name + ":" + established.reason);
      }
    }
    auto input = StageInput(stages_[current_stage_]);
    if (!lost.empty()) {
      input.unrecoverable_failure = true;
      input.reason = "established_dependency_lost:" + Join(lost);
    }
    const auto current_time = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(current_time - stage_entered_at_).count();
    auto decision = stages_[current_stage_].tracker.Update(elapsed, input);

    std::vector<std::string> pending;
    if (!input.dependencies_ready) {
      pending.push_back(input.reason);
    }
    const std::string stage_name = stages_[current_stage_].name;
    const std::string reason = decision.reason;
    if (decision.ready) {
      completed_stages_.push_back(stage_name);
      ++current_stage_;
      stage_entered_at_ = current_time;
      if (current_stage_ >= stages_.size()) {
        startup_complete_ = true;
      }
    }
    PublishStartupStatus(stage_name, decision, elapsed, reason, pending);
    PublishStartupHeartbeat();
  }

  void PublishStartupHeartbeat()
  {
    UInt64 heartbeat_message;
    heartbeat_message.data = ++heartbeat_count_;
    heartbeat_publisher_->publish(heartbeat_message);
  }

  void EvaluateAndPublish()
  {
    if (startup_enabled_) {
      EvaluateStartupAndPublish();
      return;
    }
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
  bool require_head_{false};
  bool require_locations_{false};
  bool require_semantic_{false};
  bool startup_enabled_{false};
  bool startup_complete_{false};
  bool runtime_failed_{false};
  int startup_schema_version_{1};
  double publish_rate_hz_{2.0};
  double startup_timeout_s_{45.0};
  double freshness_timeout_s_{3.0};
  double startup_evaluation_rate_hz_{5.0};
  double startup_status_publish_rate_hz_{2.0};
  double startup_observation_freshness_s_{3.0};
  rclcpp::Time started_at_;
  std::chrono::steady_clock::time_point stage_entered_at_;
  std::uint64_t heartbeat_count_{0U};
  std::uint64_t startup_sequence_{0U};
  std::size_t current_stage_{0U};
  std::string output_namespace_;
  std::string startup_id_;
  std::string infrastructure_parent_frame_{"base_footprint"};
  std::string infrastructure_child_frame_{"base_link"};
  std::map<std::string, Observation> observations_;
  std::vector<Stage> stages_;
  std::vector<std::string> completed_stages_;
  std::string last_state_;
  std::string last_reason_;
  bool startup_status_published_{false};
  std::chrono::steady_clock::time_point last_startup_status_published_at_{};
  std::string last_startup_status_key_;

  rclcpp::Publisher<String>::SharedPtr state_publisher_;
  rclcpp::Publisher<String>::SharedPtr startup_status_publisher_;
  rclcpp::Publisher<Bool>::SharedPtr ready_publisher_;
  rclcpp::Publisher<UInt64>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::vector<rclcpp::Subscription<String>::SharedPtr> string_subscriptions_;
  std::vector<rclcpp::Subscription<Bool>::SharedPtr> bool_subscriptions_;
  std::vector<rclcpp::Subscription<UInt64>::SharedPtr> counter_subscriptions_;
  rclcpp::Subscription<savo_msgs::msg::SemanticInterruptionStatus>::SharedPtr
    semantic_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
