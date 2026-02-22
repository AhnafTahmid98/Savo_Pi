// =============================================================================
// Robot SAVO — savo_control / src/nodes/twist_mux_node.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Multiplex multiple Twist command sources into a single selected command stream
// for the Robot SAVO control pipeline.
//
// Typical pipeline (current Robot SAVO architecture)
// --------------------------------------------------
// Inputs:
//   /cmd_vel_manual
//   /cmd_vel_auto
//   /cmd_vel_nav
//   /cmd_vel_recovery   (priority override when enabled/active)
//
// Flow:
//   twist_mux_node --> /cmd_vel_mux --> cmd_vel_shaper_node --> /cmd_vel
//   --> savo_perception cmd_vel_safety_gate --> /cmd_vel_safe
//
// Design goals
// ------------
// - Keep ROS-facing mux logic here; reusable control math stays elsewhere
// - Deterministic source selection with clear priority
// - Freshness (stale timeout) protection
// - Recovery override support (to avoid fighting normal modes)
// - Optional safety-stop hard-zero gate (disabled by default; main safety gate
//   remains in `savo_perception` -> /cmd_vel_safe)
//
// Notes
// -----
// - This node uses std_msgs/String for mode command/state for simplicity.
// - Supported mode_cmd strings (case-insensitive examples):
//     "MANUAL", "AUTO", "NAV", "STOP", "IDLE"
// - Recovery command can override selected mode when recovery is active and its
//   command stream is fresh.
// =============================================================================

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/topic_names.hpp"

using namespace std::chrono_literals;

namespace savo_control
{

class TwistMuxNode : public rclcpp::Node
{
public:
  TwistMuxNode()
  : Node("twist_mux_node")
  {
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);
    cmd_timeout_sec_ = this->declare_parameter<double>("cmd_timeout_sec", 0.30);
    recovery_cmd_timeout_sec_ =
      this->declare_parameter<double>("recovery_cmd_timeout_sec", 0.50);

    default_mode_ = to_upper_copy_(
      this->declare_parameter<std::string>("default_mode", "MANUAL"));

    zero_on_unknown_mode_ =
      this->declare_parameter<bool>("zero_on_unknown_mode", true);

    recovery_override_enabled_ =
      this->declare_parameter<bool>("recovery_override_enabled", true);

    latch_recovery_trigger_ =
      this->declare_parameter<bool>("latch_recovery_trigger", true);

    recovery_trigger_timeout_sec_ =
      this->declare_parameter<double>("recovery_trigger_timeout_sec", 3.0);

    use_safety_stop_gate_ =
      this->declare_parameter<bool>("use_safety_stop_gate", false);

    mode_state_publish_on_change_only_ =
      this->declare_parameter<bool>("mode_state_publish_on_change_only", false);

    // Sanitize parameters
    sanitize_params_();

    active_mode_ = normalize_mode_(default_mode_);
    if (active_mode_.empty()) {
      active_mode_ = "MANUAL";
    }

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------
    pub_cmd_mux_ = this->create_publisher<geometry_msgs::msg::Twist>(
      topic_names::kCmdVelMuxSelected, rclcpp::QoS(10));

    pub_mode_state_ = this->create_publisher<std_msgs::msg::String>(
      topic_names::kControlModeState, rclcpp::QoS(10).transient_local());

    // -------------------------------------------------------------------------
    // Subscribers: command sources
    // -------------------------------------------------------------------------
    sub_cmd_manual_ = this->create_subscription<geometry_msgs::msg::Twist>(
      topic_names::kCmdVelManual, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        on_cmd_(manual_, *msg);
      });

    sub_cmd_auto_ = this->create_subscription<geometry_msgs::msg::Twist>(
      topic_names::kCmdVelAuto, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        on_cmd_(auto_, *msg);
      });

    sub_cmd_nav_ = this->create_subscription<geometry_msgs::msg::Twist>(
      topic_names::kCmdVelNav, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        on_cmd_(nav_, *msg);
      });

    sub_cmd_recovery_ = this->create_subscription<geometry_msgs::msg::Twist>(
      topic_names::kCmdVelRecovery, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        on_cmd_(recovery_, *msg);
      });

    // -------------------------------------------------------------------------
    // Subscribers: mode / recovery / optional safety-stop
    // -------------------------------------------------------------------------
    sub_mode_cmd_ = this->create_subscription<std_msgs::msg::String>(
      topic_names::kControlModeCmd, rclcpp::QoS(10),
      std::bind(&TwistMuxNode::on_mode_cmd_, this, std::placeholders::_1));

    sub_recovery_trigger_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_names::kRecoveryTrigger, rclcpp::QoS(10),
      std::bind(&TwistMuxNode::on_recovery_trigger_, this, std::placeholders::_1));

    sub_safety_stop_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_names::kSafetyStop, rclcpp::QoS(10),
      std::bind(&TwistMuxNode::on_safety_stop_, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // Timer loop
    // -------------------------------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TwistMuxNode::on_timer_, this));

    publish_mode_state_(true);

    RCLCPP_INFO(
      this->get_logger(),
      "twist_mux_node started | mode=%s | pub=%.1f Hz | cmd_timeout=%.2fs | "
      "recovery_cmd_timeout=%.2fs | recovery_override=%s | safety_gate=%s",
      active_mode_.c_str(),
      publish_rate_hz_,
      cmd_timeout_sec_,
      recovery_cmd_timeout_sec_,
      recovery_override_enabled_ ? "true" : "false",
      use_safety_stop_gate_ ? "true" : "false");
  }

private:
  struct CmdChannel
  {
    geometry_msgs::msg::Twist last_msg{};
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    bool has_msg{false};
  };

  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------
  void on_cmd_(CmdChannel & ch, const geometry_msgs::msg::Twist & msg)
  {
    ch.last_msg = sanitize_twist_(msg);
    ch.last_stamp = this->now();
    ch.has_msg = true;
  }

  void on_mode_cmd_(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string requested = normalize_mode_(msg ? msg->data : "");
    if (requested.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Received empty/invalid mode command");
      return;
    }

    if (requested != active_mode_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Mode changed: %s -> %s",
        active_mode_.c_str(), requested.c_str());
      active_mode_ = requested;
      publish_mode_state_(true);
    }
  }

  void on_recovery_trigger_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    recovery_trigger_active_ = msg->data;
    recovery_trigger_stamp_ = this->now();

    if (!latch_recovery_trigger_ && !recovery_trigger_active_) {
      // Non-latching mode: explicit false immediately clears.
      recovery_trigger_active_ = false;
    }
  }

  void on_safety_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    safety_stop_active_ = msg->data;
    safety_stop_stamp_ = this->now();
  }

  void on_timer_()
  {
    const rclcpp::Time now = this->now();

    geometry_msgs::msg::Twist selected{};
    std::string selected_source = "NONE";
    bool selected_valid = false;

    // 1) Recovery override (highest priority when enabled + active)
    if (should_use_recovery_override_(now)) {
      if (is_fresh_(recovery_, now, recovery_cmd_timeout_sec_)) {
        selected = recovery_.last_msg;
        selected_source = "RECOVERY";
        selected_valid = true;
      } else {
        // Recovery requested but stale command -> safe zero
        selected = zero_twist_();
        selected_source = "RECOVERY_STALE_ZERO";
        selected_valid = true;
      }
    } else {
      // 2) Mode-selected source
      if (active_mode_ == "MANUAL") {
        selected_valid = select_from_channel_(
          manual_, "MANUAL", now, cmd_timeout_sec_, selected, selected_source);
      } else if (active_mode_ == "AUTO") {
        selected_valid = select_from_channel_(
          auto_, "AUTO", now, cmd_timeout_sec_, selected, selected_source);
      } else if (active_mode_ == "NAV") {
        selected_valid = select_from_channel_(
          nav_, "NAV", now, cmd_timeout_sec_, selected, selected_source);
      } else if (active_mode_ == "STOP" || active_mode_ == "IDLE") {
        selected = zero_twist_();
        selected_source = active_mode_;
        selected_valid = true;
      } else {
        // Defensive fallback
        if (zero_on_unknown_mode_) {
          selected = zero_twist_();
          selected_source = "UNKNOWN_MODE_ZERO";
          selected_valid = true;
        } else {
          selected_valid = false;
          selected_source = "UNKNOWN_MODE_NOOP";
        }
      }
    }

    // 3) Optional safety-stop hard gate
    // Main safety gate remains in savo_perception (/cmd_vel -> /cmd_vel_safe),
    // but this optional local gate can help during bringup/testing.
    if (use_safety_stop_gate_ && is_recent_safety_stop_(now) && safety_stop_active_) {
      selected = zero_twist_();
      selected_source += "+SAFETY_STOP";
      selected_valid = true;
    }

    // 4) Publish selected (or zero if nothing valid)
    if (!selected_valid) {
      selected = zero_twist_();
      selected_source = "NO_VALID_INPUT_ZERO";
    }

    pub_cmd_mux_->publish(selected);

    // 5) Mode state periodic/conditional publish
    publish_mode_state_(false);

    // 6) Diagnostics
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Mux mode=%s src=%s out[vx=%.3f vy=%.3f wz=%.3f]",
      active_mode_.c_str(), selected_source.c_str(),
      selected.linear.x, selected.linear.y, selected.angular.z);
  }

  // ---------------------------------------------------------------------------
  // Selection helpers
  // ---------------------------------------------------------------------------
  bool select_from_channel_(
    const CmdChannel & ch,
    const std::string & label,
    const rclcpp::Time & now,
    double timeout_sec,
    geometry_msgs::msg::Twist & out_twist,
    std::string & out_source) const
  {
    if (!is_fresh_(ch, now, timeout_sec)) {
      out_twist = zero_twist_();
      out_source = label + "_STALE_ZERO";
      return true;  // explicit safe output for selected mode
    }

    out_twist = ch.last_msg;
    out_source = label;
    return true;
  }

  bool is_fresh_(const CmdChannel & ch, const rclcpp::Time & now, double timeout_sec) const
  {
    if (!ch.has_msg) {
      return false;
    }
    if (!(timeout_sec > 0.0) || !std::isfinite(timeout_sec)) {
      return true;
    }

    const double age = (now - ch.last_stamp).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= timeout_sec;
  }

  bool should_use_recovery_override_(const rclcpp::Time & now)
  {
    if (!recovery_override_enabled_) {
      return false;
    }

    if (!recovery_trigger_active_) {
      return false;
    }

    // Latching behavior: auto-clear after timeout
    if (latch_recovery_trigger_ && (recovery_trigger_timeout_sec_ > 0.0)) {
      const double age = (now - recovery_trigger_stamp_).seconds();
      if (!std::isfinite(age) || age < 0.0 || age > recovery_trigger_timeout_sec_) {
        recovery_trigger_active_ = false;
        return false;
      }
    }

    return true;
  }

  bool is_recent_safety_stop_(const rclcpp::Time & now) const
  {
    if (safety_stop_stamp_.nanoseconds() == 0) {
      return false;
    }
    const double age = (now - safety_stop_stamp_).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= 1.0;
  }

  // ---------------------------------------------------------------------------
  // Data sanitation / formatting helpers
  // ---------------------------------------------------------------------------
  static geometry_msgs::msg::Twist zero_twist_()
  {
    return geometry_msgs::msg::Twist{};
  }

  static bool finite_(double v)
  {
    return std::isfinite(v);
  }

  geometry_msgs::msg::Twist sanitize_twist_(const geometry_msgs::msg::Twist & in) const
  {
    geometry_msgs::msg::Twist out = in;

    // Keep only the fields used by cmd_vel in Robot Savo control path
    out.linear.x  = finite_(out.linear.x)  ? out.linear.x  : 0.0;
    out.linear.y  = finite_(out.linear.y)  ? out.linear.y  : 0.0;
    out.angular.z = finite_(out.angular.z) ? out.angular.z : 0.0;

    // Zero unused fields defensively
    out.linear.z  = 0.0;
    out.angular.x = 0.0;
    out.angular.y = 0.0;

    return out;
  }

  static std::string to_upper_copy_(std::string s)
  {
    std::transform(
      s.begin(), s.end(), s.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
  }

  static std::string trim_copy_(std::string s)
  {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };

    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
  }

  static std::string normalize_mode_(const std::string & raw)
  {
    std::string s = to_upper_copy_(trim_copy_(raw));

    // Friendly aliases
    if (s == "TELEOP")      s = "MANUAL";
    if (s == "MAN")         s = "MANUAL";
    if (s == "AUTONOMOUS")  s = "AUTO";
    if (s == "AUTON")       s = "AUTO";
    if (s == "NAV2")        s = "NAV";

    if (s == "MANUAL" || s == "AUTO" || s == "NAV" || s == "STOP" || s == "IDLE") {
      return s;
    }
    return {};
  }

  void publish_mode_state_(bool force)
  {
    if (mode_state_publish_on_change_only_ && !force) {
      return;
    }

    if (!force && active_mode_ == last_published_mode_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = active_mode_;
    pub_mode_state_->publish(msg);
    last_published_mode_ = active_mode_;
  }

  void sanitize_params_()
  {
    if (!(publish_rate_hz_ > 0.0) || !std::isfinite(publish_rate_hz_)) {
      publish_rate_hz_ = 20.0;
    }
    if (!(cmd_timeout_sec_ > 0.0) || !std::isfinite(cmd_timeout_sec_)) {
      cmd_timeout_sec_ = 0.30;
    }
    if (!(recovery_cmd_timeout_sec_ > 0.0) || !std::isfinite(recovery_cmd_timeout_sec_)) {
      recovery_cmd_timeout_sec_ = 0.50;
    }
    if (!(recovery_trigger_timeout_sec_ > 0.0) || !std::isfinite(recovery_trigger_timeout_sec_)) {
      recovery_trigger_timeout_sec_ = 3.0;
    }

    default_mode_ = normalize_mode_(default_mode_);
    if (default_mode_.empty()) {
      default_mode_ = "MANUAL";
    }
  }

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_mux_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_state_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_manual_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_auto_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_nav_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_recovery_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_mode_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_recovery_trigger_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Command channels
  CmdChannel manual_;
  CmdChannel auto_;
  CmdChannel nav_;
  CmdChannel recovery_;

  // Mode / recovery state
  std::string active_mode_{"MANUAL"};
  std::string default_mode_{"MANUAL"};
  std::string last_published_mode_;

  bool recovery_trigger_active_{false};
  rclcpp::Time recovery_trigger_stamp_{0, 0, RCL_ROS_TIME};

  bool safety_stop_active_{false};
  rclcpp::Time safety_stop_stamp_{0, 0, RCL_ROS_TIME};

  // Parameters
  double publish_rate_hz_{20.0};
  double cmd_timeout_sec_{0.30};
  double recovery_cmd_timeout_sec_{0.50};
  double recovery_trigger_timeout_sec_{3.0};

  bool zero_on_unknown_mode_{true};
  bool recovery_override_enabled_{true};
  bool latch_recovery_trigger_{true};
  bool use_safety_stop_gate_{false};
  bool mode_state_publish_on_change_only_{false};
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::TwistMuxNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}