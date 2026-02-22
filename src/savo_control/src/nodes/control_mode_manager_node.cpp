// =============================================================================
// Robot SAVO — savo_control / src/nodes/control_mode_manager_node.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// ROS2 node wrapper for the reusable ControlModeManager policy helper.
//
// This node:
//   - receives mode requests (STOP / MANUAL / AUTO / NAV / RECOVERY)
//   - monitors safety stop and recovery state hints
//   - applies deterministic arbitration via ControlModeManager
//   - publishes current mode state / status for other nodes (e.g. twist_mux_node)
//
// Design intent
// -------------
// Keep mode policy in `control_mode_manager.hpp` (ROS-independent),
// and keep ROS wiring / parameter parsing / topic I/O in this node.
//
// Notes
// -----
// - This node does NOT mux Twist messages directly.
// - It publishes a plain mode string on /savo_control/mode_state so `twist_mux_node`
//   can consume it directly ("MANUAL", "AUTO", "NAV", "STOP").
// - It also publishes richer debug/status text on /savo_control/control_status and
//   /savo_control/control_debug for observability.
// =============================================================================

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/control_mode_manager.hpp"
#include "savo_control/topic_names.hpp"

namespace savo_control
{

class ControlModeManagerNode : public rclcpp::Node
{
public:
  ControlModeManagerNode()
  : Node("control_mode_manager_node")
  {
    declare_parameters_();
    load_parameters_();

    manager_.set_config(cfg_);
    manager_.reset();

    // -------------------------
    // Subscribers
    // -------------------------
    sub_mode_cmd_ = this->create_subscription<std_msgs::msg::String>(
      topic_names::kControlModeCmd,
      rclcpp::QoS(10),
      std::bind(&ControlModeManagerNode::on_mode_cmd_, this, std::placeholders::_1));

    sub_safety_stop_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_names::kSafetyStop,
      rclcpp::QoS(10),
      std::bind(&ControlModeManagerNode::on_safety_stop_, this, std::placeholders::_1));

    sub_recovery_trigger_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_names::kRecoveryTrigger,
      rclcpp::QoS(10),
      std::bind(&ControlModeManagerNode::on_recovery_trigger_, this, std::placeholders::_1));

    sub_recovery_state_ = this->create_subscription<std_msgs::msg::String>(
      topic_names::kRecoveryState,
      rclcpp::QoS(10),
      std::bind(&ControlModeManagerNode::on_recovery_state_, this, std::placeholders::_1));

    // Optional helper input topics
    if (!manual_override_topic_.empty()) {
      sub_manual_override_ = this->create_subscription<std_msgs::msg::Bool>(
        manual_override_topic_,
        rclcpp::QoS(10),
        std::bind(&ControlModeManagerNode::on_manual_override_, this, std::placeholders::_1));
    }

    if (!external_stop_topic_.empty()) {
      sub_external_stop_ = this->create_subscription<std_msgs::msg::Bool>(
        external_stop_topic_,
        rclcpp::QoS(10),
        std::bind(&ControlModeManagerNode::on_external_stop_, this, std::placeholders::_1));
    }

    if (!manual_available_topic_.empty()) {
      sub_manual_available_ = this->create_subscription<std_msgs::msg::Bool>(
        manual_available_topic_,
        rclcpp::QoS(10),
        std::bind(&ControlModeManagerNode::on_manual_available_, this, std::placeholders::_1));
    }

    if (!auto_available_topic_.empty()) {
      sub_auto_available_ = this->create_subscription<std_msgs::msg::Bool>(
        auto_available_topic_,
        rclcpp::QoS(10),
        std::bind(&ControlModeManagerNode::on_auto_available_, this, std::placeholders::_1));
    }

    if (!nav_available_topic_.empty()) {
      sub_nav_available_ = this->create_subscription<std_msgs::msg::Bool>(
        nav_available_topic_,
        rclcpp::QoS(10),
        std::bind(&ControlModeManagerNode::on_nav_available_, this, std::placeholders::_1));
    }

    if (!recovery_available_topic_.empty()) {
      sub_recovery_available_ = this->create_subscription<std_msgs::msg::Bool>(
        recovery_available_topic_,
        rclcpp::QoS(10),
        std::bind(&ControlModeManagerNode::on_recovery_available_, this, std::placeholders::_1));
    }

    // -------------------------
    // Publishers
    // -------------------------
    // IMPORTANT: twist_mux_node expects plain mode strings on this topic.
    pub_mode_state_ = this->create_publisher<std_msgs::msg::String>(
      topic_names::kControlModeState,
      rclcpp::QoS(10).transient_local());

    pub_control_status_ = this->create_publisher<std_msgs::msg::String>(
      topic_names::kControlStatus, rclcpp::QoS(10));

    pub_control_debug_ = this->create_publisher<std_msgs::msg::String>(
      topic_names::kControlDebug, rclcpp::QoS(10));

    if (publish_route_topic_) {
      pub_selected_route_ = this->create_publisher<std_msgs::msg::String>(
        selected_route_topic_, rclcpp::QoS(10));
    }

    // Publish startup state immediately (latched)
    // Use cfg_.startup_mode directly (ControlModeManager has no status() accessor).
    publish_plain_mode_state_(cfg_.startup_mode);

    // -------------------------
    // Main timer
    // -------------------------
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, update_rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ControlModeManagerNode::on_timer_, this));

    RCLCPP_INFO(
      get_logger(),
      "control_mode_manager_node started (%.1f Hz) | startup_mode=%s | route_topic=%s",
      update_rate_hz_,
      to_string_(cfg_.startup_mode).c_str(),
      publish_route_topic_ ? selected_route_topic_.c_str() : "<disabled>");
  }

private:
  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  void declare_parameters_()
  {
    this->declare_parameter<double>("update_rate_hz", 20.0);

    // Policy config
    this->declare_parameter<std::string>("startup_mode", "STOP");
    this->declare_parameter<bool>("manual_override_preempts_all", true);
    this->declare_parameter<bool>("recovery_preempts_auto_nav", true);
    this->declare_parameter<bool>("recovery_preempts_manual", false);
    this->declare_parameter<bool>("latch_recovery_mode", true);
    this->declare_parameter<bool>("safety_stop_forces_stop_mode", true);
    this->declare_parameter<bool>("external_stop_forces_stop_mode", true);
    this->declare_parameter<bool>("prefer_nav_over_auto", false);
    this->declare_parameter<bool>("fallback_to_stop_when_source_unavailable", true);
    this->declare_parameter<double>("min_time_sec", 0.0);
    this->declare_parameter<double>("max_time_jump_sec", 5.0);

    // Optional integration topics
    this->declare_parameter<std::string>("manual_override_topic", "");
    this->declare_parameter<std::string>("external_stop_topic", "");

    this->declare_parameter<std::string>("manual_available_topic", "");
    this->declare_parameter<std::string>("auto_available_topic", "");
    this->declare_parameter<std::string>("nav_available_topic", "");
    this->declare_parameter<std::string>("recovery_available_topic", "");

    // Optional route selection topic
    this->declare_parameter<bool>("publish_selected_route_topic", true);
    this->declare_parameter<std::string>("selected_route_topic", "/savo_control/selected_cmd_source");

    // Defaults for source availability if no availability topics exist
    this->declare_parameter<bool>("manual_source_available_default", true);
    this->declare_parameter<bool>("auto_source_available_default", true);
    this->declare_parameter<bool>("nav_source_available_default", true);
    this->declare_parameter<bool>("recovery_source_available_default", true);

    // Logging
    this->declare_parameter<bool>("log_mode_changes", true);
    this->declare_parameter<bool>("log_request_changes", false);
  }

  void load_parameters_()
  {
    update_rate_hz_ = this->get_parameter("update_rate_hz").as_double();

    cfg_.startup_mode = parse_mode_or_default_(
      this->get_parameter("startup_mode").as_string(), ControlMode::kStop);

    cfg_.manual_override_preempts_all =
      this->get_parameter("manual_override_preempts_all").as_bool();
    cfg_.recovery_preempts_auto_nav =
      this->get_parameter("recovery_preempts_auto_nav").as_bool();
    cfg_.recovery_preempts_manual =
      this->get_parameter("recovery_preempts_manual").as_bool();
    cfg_.latch_recovery_mode =
      this->get_parameter("latch_recovery_mode").as_bool();
    cfg_.safety_stop_forces_stop_mode =
      this->get_parameter("safety_stop_forces_stop_mode").as_bool();
    cfg_.external_stop_forces_stop_mode =
      this->get_parameter("external_stop_forces_stop_mode").as_bool();
    cfg_.prefer_nav_over_auto =
      this->get_parameter("prefer_nav_over_auto").as_bool();
    cfg_.fallback_to_stop_when_source_unavailable =
      this->get_parameter("fallback_to_stop_when_source_unavailable").as_bool();
    cfg_.min_time_sec =
      this->get_parameter("min_time_sec").as_double();
    cfg_.max_time_jump_sec =
      this->get_parameter("max_time_jump_sec").as_double();

    manual_override_topic_ = this->get_parameter("manual_override_topic").as_string();
    external_stop_topic_ = this->get_parameter("external_stop_topic").as_string();

    manual_available_topic_ = this->get_parameter("manual_available_topic").as_string();
    auto_available_topic_ = this->get_parameter("auto_available_topic").as_string();
    nav_available_topic_ = this->get_parameter("nav_available_topic").as_string();
    recovery_available_topic_ = this->get_parameter("recovery_available_topic").as_string();

    publish_route_topic_ = this->get_parameter("publish_selected_route_topic").as_bool();
    selected_route_topic_ = this->get_parameter("selected_route_topic").as_string();

    inputs_.manual_source_available =
      this->get_parameter("manual_source_available_default").as_bool();
    inputs_.auto_source_available =
      this->get_parameter("auto_source_available_default").as_bool();
    inputs_.nav_source_available =
      this->get_parameter("nav_source_available_default").as_bool();
    inputs_.recovery_source_available =
      this->get_parameter("recovery_source_available_default").as_bool();

    log_mode_changes_ = this->get_parameter("log_mode_changes").as_bool();
    log_request_changes_ = this->get_parameter("log_request_changes").as_bool();

    if (!(update_rate_hz_ > 0.0) || !std::isfinite(update_rate_hz_)) {
      update_rate_hz_ = 20.0;
    }
  }

  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------
  void on_mode_cmd_(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const std::string s = upper_trim_(msg->data);

    if (s == "STOP") {
      requested_mode_cmd_ = ControlMode::kStop;
    } else if (s == "MANUAL" || s == "TELEOP" || s == "MAN") {
      requested_mode_cmd_ = ControlMode::kManual;
    } else if (s == "AUTO" || s == "AUTONOMOUS" || s == "AUTON") {
      requested_mode_cmd_ = ControlMode::kAuto;
    } else if (s == "NAV" || s == "NAV2") {
      requested_mode_cmd_ = ControlMode::kNav;
    } else if (s == "RECOVERY") {
      requested_mode_cmd_ = ControlMode::kRecovery;
    } else if (s == "IDLE") {
      // Treat IDLE as STOP for compatibility with simple mux command styles
      requested_mode_cmd_ = ControlMode::kStop;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Unknown mode_cmd '%s' (expected STOP/MANUAL/AUTO/NAV/RECOVERY)",
        msg->data.c_str());
      return;
    }

    have_mode_cmd_ = true;
    last_mode_cmd_raw_ = s;
  }

  void on_safety_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    inputs_.safety_stop_active = msg->data;
  }

  void on_recovery_trigger_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    // one-shot style hint (consumed in timer and cleared)
    inputs_.recovery_triggered = msg->data;
  }

  void on_recovery_state_(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) return;

    const std::string s = upper_trim_(msg->data);
    last_recovery_state_raw_ = s;

    // Reset one-shot completion/abort edges each time we parse a state string
    inputs_.recovery_completed = false;
    inputs_.recovery_aborted = false;

    if (s.find("ABORT") != std::string::npos) {
      inputs_.recovery_active = false;
      inputs_.recovery_aborted = true;
    } else if (s.find("COMPLETE") != std::string::npos || s == "DONE") {
      inputs_.recovery_active = false;
      inputs_.recovery_completed = true;
    } else if (s.find("COOLDOWN") != std::string::npos) {
      inputs_.recovery_active = false;
    } else if (s.find("IDLE") != std::string::npos) {
      inputs_.recovery_active = false;
    } else if (s.find("ACTIVE") != std::string::npos ||
               s.find("BACK") != std::string::npos ||
               s.find("TURN") != std::string::npos ||
               s.find("SETTL") != std::string::npos ||
               s.find("ARMED") != std::string::npos)
    {
      inputs_.recovery_active = true;
    } else {
      RCLCPP_DEBUG(get_logger(), "Unrecognized recovery_state string: '%s'", s.c_str());
    }
  }

  void on_manual_override_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    inputs_.manual_override_active = msg->data;
  }

  void on_external_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    inputs_.external_stop_active = msg->data;
  }

  void on_manual_available_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    inputs_.manual_source_available = msg->data;
  }

  void on_auto_available_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    inputs_.auto_source_available = msg->data;
  }

  void on_nav_available_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    inputs_.nav_source_available = msg->data;
  }

  void on_recovery_available_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    inputs_.recovery_source_available = msg->data;
  }

  // ---------------------------------------------------------------------------
  // Main timer
  // ---------------------------------------------------------------------------
  void on_timer_()
  {
    const double now_sec = this->now().seconds();

    // Build request flags from latched mode command
    inputs_.request_stop = false;
    inputs_.request_manual = false;
    inputs_.request_auto = false;
    inputs_.request_nav = false;

    if (have_mode_cmd_) {
      switch (requested_mode_cmd_) {
        case ControlMode::kStop:
          inputs_.request_stop = true;
          break;
        case ControlMode::kManual:
          inputs_.request_manual = true;
          break;
        case ControlMode::kAuto:
          inputs_.request_auto = true;
          break;
        case ControlMode::kNav:
          inputs_.request_nav = true;
          break;
        case ControlMode::kRecovery:
          // explicit RECOVERY command maps to recovery trigger/active hint
          inputs_.recovery_triggered = true;
          if (inputs_.recovery_source_available) {
            inputs_.recovery_active = true;
          }
          break;
      }
    }

    const auto status = manager_.update(now_sec, inputs_);

    // Publish outputs
    publish_plain_mode_state_(status.active_mode);   // <-- mux-compatible plain string
    publish_control_status_(status);
    publish_control_debug_(status);

    if (publish_route_topic_) {
      publish_selected_route_(status);
    }

    if (status.mode_changed && log_mode_changes_) {
      RCLCPP_INFO(
        get_logger(),
        "Mode changed | active=%s requested=%s reason=%s recovery_latched=%s",
        to_string_(status.active_mode).c_str(),
        to_string_(status.requested_mode).c_str(),
        to_string_(status.reason).c_str(),
        status.recovery_latched ? "true" : "false");
    } else if (status.request_changed && log_request_changes_) {
      RCLCPP_INFO(
        get_logger(),
        "Mode request changed | requested=%s",
        to_string_(status.requested_mode).c_str());
    }

    // Clear one-shot edge hints after update cycle
    inputs_.recovery_triggered = false;
    inputs_.recovery_completed = false;
    inputs_.recovery_aborted = false;
  }

  // ---------------------------------------------------------------------------
  // Publishers helpers
  // ---------------------------------------------------------------------------
  void publish_plain_mode_state_(ControlMode active_mode)
  {
    std_msgs::msg::String msg;
    msg.data = plain_mux_mode_string_(active_mode);
    pub_mode_state_->publish(msg);
  }

  void publish_control_status_(const ControlModeManagerStatus & status)
  {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << "{"
       << "\"node\":\"control_mode_manager\","
       << "\"active_mode\":\"" << to_string_(status.active_mode) << "\","
       << "\"requested_mode\":\"" << to_string_(status.requested_mode) << "\","
       << "\"reason\":\"" << to_string_(status.reason) << "\","
       << "\"recovery_latched\":" << (status.recovery_latched ? "true" : "false") << ","
       << "\"mode_elapsed_sec\":" << status.mode_elapsed_sec << ","
       << "\"valid_time\":" << (status.valid_time ? "true" : "false") << ","
       << "\"route\":\"" << route_name_from_action_(status.action) << "\""
       << "}";
    msg.data = ss.str();
    pub_control_status_->publish(msg);
  }

  void publish_control_debug_(const ControlModeManagerStatus & status)
  {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << "safety_stop=" << (inputs_.safety_stop_active ? "1" : "0")
       << " external_stop=" << (inputs_.external_stop_active ? "1" : "0")
       << " manual_override=" << (inputs_.manual_override_active ? "1" : "0")
       << " recovery_active=" << (inputs_.recovery_active ? "1" : "0")
       << " manual_avail=" << (inputs_.manual_source_available ? "1" : "0")
       << " auto_avail=" << (inputs_.auto_source_available ? "1" : "0")
       << " nav_avail=" << (inputs_.nav_source_available ? "1" : "0")
       << " recovery_avail=" << (inputs_.recovery_source_available ? "1" : "0")
       << " mode_cmd=" << (have_mode_cmd_ ? last_mode_cmd_raw_ : "<none>")
       << " recovery_state=" << (last_recovery_state_raw_.empty() ? "<none>" : last_recovery_state_raw_)
       << " selected_route=" << route_name_from_action_(status.action);
    msg.data = ss.str();
    pub_control_debug_->publish(msg);
  }

  void publish_selected_route_(const ControlModeManagerStatus & status)
  {
    std_msgs::msg::String msg;
    msg.data = route_name_from_action_(status.action);
    pub_selected_route_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // String / enum helpers
  // ---------------------------------------------------------------------------
  static std::string upper_trim_(std::string s)
  {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };

    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());

    std::transform(
      s.begin(), s.end(), s.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
  }

  static ControlMode parse_mode_or_default_(const std::string & s, ControlMode def)
  {
    const auto u = upper_trim_(s);
    if (u == "STOP" || u == "IDLE") return ControlMode::kStop;
    if (u == "MANUAL" || u == "TELEOP" || u == "MAN") return ControlMode::kManual;
    if (u == "AUTO" || u == "AUTONOMOUS" || u == "AUTON") return ControlMode::kAuto;
    if (u == "NAV" || u == "NAV2") return ControlMode::kNav;
    if (u == "RECOVERY") return ControlMode::kRecovery;
    return def;
  }

  static std::string to_string_(ControlMode m)
  {
    switch (m) {
      case ControlMode::kStop: return "STOP";
      case ControlMode::kManual: return "MANUAL";
      case ControlMode::kAuto: return "AUTO";
      case ControlMode::kNav: return "NAV";
      case ControlMode::kRecovery: return "RECOVERY";
      default: return "UNKNOWN";
    }
  }

  static std::string to_string_(ControlModeReason r)
  {
    switch (r) {
      case ControlModeReason::kNone: return "NONE";
      case ControlModeReason::kStartup: return "STARTUP";
      case ControlModeReason::kRequested: return "REQUESTED";
      case ControlModeReason::kManualOverride: return "MANUAL_OVERRIDE";
      case ControlModeReason::kRecoveryActive: return "RECOVERY_ACTIVE";
      case ControlModeReason::kRecoveryLatched: return "RECOVERY_LATCHED";
      case ControlModeReason::kSafetyStopActive: return "SAFETY_STOP_ACTIVE";
      case ControlModeReason::kExternalStop: return "EXTERNAL_STOP";
      case ControlModeReason::kInvalidTime: return "INVALID_TIME";
      case ControlModeReason::kTimeout: return "TIMEOUT";
      case ControlModeReason::kNoSourceAllowed: return "NO_SOURCE_ALLOWED";
      default: return "UNKNOWN";
    }
  }

  // Map control-mode helper enum to the plain strings expected by twist_mux_node.
  static std::string plain_mux_mode_string_(ControlMode m)
  {
    switch (m) {
      case ControlMode::kManual:
        return "MANUAL";
      case ControlMode::kAuto:
        return "AUTO";
      case ControlMode::kNav:
        return "NAV";
      case ControlMode::kRecovery:
        // twist_mux_node uses recovery trigger + /cmd_vel_recovery override,
        // so plain mode should remain a non-recovery base mode.
        // STOP is the safest neutral value when recovery is active.
        return "STOP";
      case ControlMode::kStop:
      default:
        return "STOP";
    }
  }

  static std::string route_name_from_action_(const ControlModeAction & a)
  {
    if (a.select_manual)   return topic_names::kCmdVelManual;
    if (a.select_auto)     return topic_names::kCmdVelAuto;
    if (a.select_nav)      return topic_names::kCmdVelNav;
    if (a.select_recovery) return topic_names::kCmdVelRecovery;
    return "STOP";
  }

private:
  // Policy helper
  ControlModeManager manager_;
  ControlModeManagerConfig cfg_{};

  // Cached inputs to policy
  ControlModeInputs inputs_{};

  // Mode command latch
  bool have_mode_cmd_ {false};
  ControlMode requested_mode_cmd_ {ControlMode::kStop};
  std::string last_mode_cmd_raw_;
  std::string last_recovery_state_raw_;

  // Parameters
  double update_rate_hz_ {20.0};

  std::string manual_override_topic_;
  std::string external_stop_topic_;

  std::string manual_available_topic_;
  std::string auto_available_topic_;
  std::string nav_available_topic_;
  std::string recovery_available_topic_;

  bool publish_route_topic_ {true};
  std::string selected_route_topic_ {"/savo_control/selected_cmd_source"};

  bool log_mode_changes_ {true};
  bool log_request_changes_ {false};

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_recovery_trigger_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_recovery_state_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_manual_override_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_external_stop_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_manual_available_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_available_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_nav_available_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_recovery_available_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_control_status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_control_debug_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_selected_route_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::ControlModeManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}