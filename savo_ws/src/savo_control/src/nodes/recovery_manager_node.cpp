// =============================================================================
// Robot SAVO — savo_control / src/nodes/recovery_manager_node.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// ROS2 node wrapper for the reusable RecoveryManager policy/state helper.
//
// This node:
// - Subscribes to stuck/safety/manual/cancel signals
// - Runs RecoveryManager at a fixed loop rate
// - Publishes recovery command Twist on /cmd_vel_recovery
// - Publishes recovery state/status/debug strings
// - Publishes Bool trigger/state topics for observability
//
// Design notes
// ------------
// - The recovery policy/state machine lives in recovery_manager.hpp (ROS-free).
// - This node only handles ROS I/O, parameter loading, and command generation.
// - Commands are intentionally simple and deterministic:
//     * BackingUp -> negative linear.x
//     * Settling  -> zero Twist
//     * Turning   -> angular.z with alternating sign from manager
//
// Professional usage
// ------------------
// Use this node before full Nav2 recovery behavior integration to validate local
// recovery on real hardware with your existing cmd_vel pipeline:
//
//   recovery_manager_node -> /cmd_vel_recovery
//                        -> twist_mux_node
//                        -> cmd_vel_shaper_node
//                        -> /cmd_vel
//                        -> safety gate -> /cmd_vel_safe
// =============================================================================

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/recovery_manager.hpp"
#include "savo_control/topic_names.hpp"

using namespace std::chrono_literals;

namespace savo_control
{

class RecoveryManagerNode : public rclcpp::Node
{
public:
  RecoveryManagerNode()
  : Node("recovery_manager_node")
  {
    // -------------------------------------------------------------------------
    // Parameters: topics
    // -------------------------------------------------------------------------
    topic_stuck_detected_ = this->declare_parameter<std::string>(
      "topics.stuck_detected", topic_names::kStuckDetected);

    topic_recovery_trigger_ = this->declare_parameter<std::string>(
      "topics.recovery_trigger", topic_names::kRecoveryTrigger);

    topic_safety_stop_ = this->declare_parameter<std::string>(
      "topics.safety_stop", topic_names::kSafetyStop);

    topic_cmd_vel_recovery_ = this->declare_parameter<std::string>(
      "topics.cmd_vel_recovery", topic_names::kCmdVelRecovery);

    topic_recovery_state_ = this->declare_parameter<std::string>(
      "topics.recovery_state", topic_names::kRecoveryState);

    topic_recovery_status_ = this->declare_parameter<std::string>(
      "topics.recovery_status", topic_names::kRecoveryStatus);

    // Optional/manual integration topics (defaults kept reusable)
    topic_manual_override_ = this->declare_parameter<std::string>(
      "topics.manual_override", topic_names::kControlModeCmd);  // can remap to Bool topic

    topic_motion_allowed_ = this->declare_parameter<std::string>(
      "topics.motion_allowed", "");  // optional Bool topic; empty => use param default

    topic_no_progress_ = this->declare_parameter<std::string>(
      "topics.no_progress", "");      // optional Bool input

    // -------------------------------------------------------------------------
    // Parameters: loop and input stale handling
    // -------------------------------------------------------------------------
    loop_rate_hz_ = this->declare_parameter<double>("loop_rate_hz", 30.0);
    input_timeout_sec_ = this->declare_parameter<double>("input_timeout_sec", 0.5);

    // If an input topic is enabled but stale, choose safe defaults below
    stale_stuck_as_false_ = this->declare_parameter<bool>("stale.stuck_as_false", true);
    stale_no_progress_as_false_ = this->declare_parameter<bool>("stale.no_progress_as_false", true);
    stale_safety_stop_as_true_ = this->declare_parameter<bool>("stale.safety_stop_as_true", false);
    stale_manual_override_as_true_ = this->declare_parameter<bool>("stale.manual_override_as_true", false);
    stale_motion_allowed_as_false_ = this->declare_parameter<bool>("stale.motion_allowed_as_false", false);

    // -------------------------------------------------------------------------
    // Parameters: recovery policy (RecoveryManagerConfig)
    // -------------------------------------------------------------------------
    RecoveryManagerConfig cfg{};
    cfg.trigger_confirm_sec = this->declare_parameter<double>("policy.trigger_confirm_sec", 0.20);
    cfg.backup_duration_sec = this->declare_parameter<double>("policy.backup_duration_sec", 0.60);
    cfg.settle_duration_sec = this->declare_parameter<double>("policy.settle_duration_sec", 0.20);
    cfg.turn_duration_sec = this->declare_parameter<double>("policy.turn_duration_sec", 0.40);
    cfg.enable_turn_phase = this->declare_parameter<bool>("policy.enable_turn_phase", true);
    cfg.max_recovery_duration_sec =
      this->declare_parameter<double>("policy.max_recovery_duration_sec", 3.0);
    cfg.cooldown_sec = this->declare_parameter<double>("policy.cooldown_sec", 1.0);
    cfg.max_attempts_per_streak =
      static_cast<std::uint32_t>(this->declare_parameter<int>("policy.max_attempts_per_streak", 3));
    cfg.reset_attempts_on_progress =
      this->declare_parameter<bool>("policy.reset_attempts_on_progress", true);
    cfg.min_time_sec = this->declare_parameter<double>("policy.min_time_sec", 0.0);
    cfg.max_time_jump_sec = this->declare_parameter<double>("policy.max_time_jump_sec", 5.0);

    // -------------------------------------------------------------------------
    // Parameters: command generation (node-level execution of policy action)
    // -------------------------------------------------------------------------
    backup_speed_m_s_ = this->declare_parameter<double>("command.backup_speed_m_s", 0.06);
    turn_speed_rad_s_ = this->declare_parameter<double>("command.turn_speed_rad_s", 0.35);
    publish_zero_when_idle_ = this->declare_parameter<bool>("command.publish_zero_when_idle", true);
    publish_zero_on_abort_or_done_ = this->declare_parameter<bool>("command.publish_zero_on_abort_or_done", true);

    // Optional completion hints (time-based by default from manager; these can be enabled later)
    use_external_backup_completed_ =
      this->declare_parameter<bool>("completion.use_external_backup_completed", false);
    use_external_turn_completed_ =
      this->declare_parameter<bool>("completion.use_external_turn_completed", false);

    topic_backup_completed_ = this->declare_parameter<std::string>(
      "topics.backup_completed", "");
    topic_turn_completed_ = this->declare_parameter<std::string>(
      "topics.turn_completed", "");

    sanitize_params_();
    manager_.set_config(cfg);

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------
    pub_cmd_vel_recovery_ = this->create_publisher<geometry_msgs::msg::Twist>(
      topic_cmd_vel_recovery_, rclcpp::QoS(10));

    pub_recovery_state_ = this->create_publisher<std_msgs::msg::String>(
      topic_recovery_state_, rclcpp::QoS(10).transient_local());

    pub_recovery_status_ = this->create_publisher<std_msgs::msg::String>(
      topic_recovery_status_, rclcpp::QoS(10));

    // Also publish Bool on kRecoveryTrigger for observability / downstream simple use
    // (true when active recovery action is running)
    pub_recovery_trigger_bool_ = this->create_publisher<std_msgs::msg::Bool>(
      topic_recovery_trigger_, rclcpp::QoS(10));

    // -------------------------------------------------------------------------
    // Subscribers (core inputs)
    // -------------------------------------------------------------------------
    sub_stuck_detected_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_stuck_detected_, rclcpp::QoS(10),
      std::bind(&RecoveryManagerNode::on_stuck_detected_, this, std::placeholders::_1));

    sub_safety_stop_ = this->create_subscription<std_msgs::msg::Bool>(
      topic_safety_stop_, rclcpp::QoS(10),
      std::bind(&RecoveryManagerNode::on_safety_stop_, this, std::placeholders::_1));

    // NOTE: topic_names::kControlModeCmd is string in your layout; if left default,
    // this Bool subscriber may not match any publisher (harmless). Set a real Bool
    // topic in params when you wire manual override.
    if (!topic_manual_override_.empty()) {
      sub_manual_override_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_manual_override_, rclcpp::QoS(10),
        std::bind(&RecoveryManagerNode::on_manual_override_, this, std::placeholders::_1));
    }

    if (!topic_motion_allowed_.empty()) {
      sub_motion_allowed_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_motion_allowed_, rclcpp::QoS(10),
        std::bind(&RecoveryManagerNode::on_motion_allowed_, this, std::placeholders::_1));
      has_motion_allowed_topic_ = true;
    }

    if (!topic_no_progress_.empty()) {
      sub_no_progress_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_no_progress_, rclcpp::QoS(10),
        std::bind(&RecoveryManagerNode::on_no_progress_, this, std::placeholders::_1));
      has_no_progress_topic_ = true;
    }

    if (use_external_backup_completed_ && !topic_backup_completed_.empty()) {
      sub_backup_completed_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_backup_completed_, rclcpp::QoS(10),
        std::bind(&RecoveryManagerNode::on_backup_completed_, this, std::placeholders::_1));
    }

    if (use_external_turn_completed_ && !topic_turn_completed_.empty()) {
      sub_turn_completed_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_turn_completed_, rclcpp::QoS(10),
        std::bind(&RecoveryManagerNode::on_turn_completed_, this, std::placeholders::_1));
    }

    // -------------------------------------------------------------------------
    // Timer
    // -------------------------------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&RecoveryManagerNode::on_timer_, this));

    publish_state_snapshot_("startup");
    publish_status_text_("idle");

    RCLCPP_INFO(
      this->get_logger(),
      "recovery_manager_node started | stuck=%s | safety=%s | out=%s | loop=%.1fHz",
      topic_stuck_detected_.c_str(), topic_safety_stop_.c_str(),
      topic_cmd_vel_recovery_.c_str(), loop_rate_hz_);
  }

private:
  // ---------------------------------------------------------------------------
  // Small input cache helper
  // ---------------------------------------------------------------------------
  struct TimedBool
  {
    bool value{false};
    bool has_msg{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  // ---------------------------------------------------------------------------
  // Subscribers callbacks
  // ---------------------------------------------------------------------------
  void on_stuck_detected_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    stuck_detected_.value = msg->data;
    stuck_detected_.has_msg = true;
    stuck_detected_.stamp = this->now();
  }

  void on_safety_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    safety_stop_.value = msg->data;
    safety_stop_.has_msg = true;
    safety_stop_.stamp = this->now();
  }

  void on_manual_override_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    manual_override_.value = msg->data;
    manual_override_.has_msg = true;
    manual_override_.stamp = this->now();
  }

  void on_motion_allowed_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    motion_allowed_.value = msg->data;
    motion_allowed_.has_msg = true;
    motion_allowed_.stamp = this->now();
  }

  void on_no_progress_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    no_progress_.value = msg->data;
    no_progress_.has_msg = true;
    no_progress_.stamp = this->now();
  }

  void on_backup_completed_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    backup_completed_.value = msg->data;
    backup_completed_.has_msg = true;
    backup_completed_.stamp = this->now();
  }

  void on_turn_completed_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    turn_completed_.value = msg->data;
    turn_completed_.has_msg = true;
    turn_completed_.stamp = this->now();
  }

  // ---------------------------------------------------------------------------
  // Main loop
  // ---------------------------------------------------------------------------
  void on_timer_()
  {
    const auto now_ros = this->now();
    const double now_sec = now_ros.seconds();

    // Build manager inputs using stale-aware reads
    RecoveryInputs in{};
    in.stuck_detected = read_input_(stuck_detected_, stale_stuck_as_false_ ? false : true);
    in.no_progress = has_no_progress_topic_
      ? read_input_(no_progress_, stale_no_progress_as_false_ ? false : true)
      : false;
    in.safety_stop_active = read_input_(safety_stop_, stale_safety_stop_as_true_ ? true : false);
    in.manual_override = (!topic_manual_override_.empty())
      ? read_input_(manual_override_, stale_manual_override_as_true_ ? true : false)
      : false;

    // If no topic wired, default motion_allowed=true unless parameter says otherwise via stale fallback
    if (has_motion_allowed_topic_) {
      in.motion_allowed = read_input_(motion_allowed_, stale_motion_allowed_as_false_ ? false : true);
    } else {
      in.motion_allowed = true;
    }

    // External cancel trigger: edge on /recovery_trigger bool false->true from outside is awkward
    // because this node also publishes same topic as state. To keep clean behavior, treat ONLY
    // manual_override as cancel in current wiring unless user maps a dedicated cancel topic later.
    in.external_cancel = false;

    in.backup_completed = (use_external_backup_completed_ && !topic_backup_completed_.empty())
      ? read_input_(backup_completed_, false) : false;
    in.turn_completed = (use_external_turn_completed_ && !topic_turn_completed_.empty())
      ? read_input_(turn_completed_, false) : false;

    // If system is clearly progressing again while not recovering, note progress to clear streak
    // (simple heuristic: no stuck and no no_progress and no safety stop)
    if (!in.stuck_detected && !in.no_progress && !in.safety_stop_active) {
      manager_.note_progress();
    }

    const RecoveryManagerStatus s = manager_.update(now_sec, in);

    // Convert manager action to Twist
    geometry_msgs::msg::Twist cmd = make_cmd_from_status_(s);

    // Publish command
    if (s.active || publish_zero_when_idle_) {
      pub_cmd_vel_recovery_->publish(cmd);
    }

    // Publish Bool trigger/state (true when active recovery commanding stage is active)
    std_msgs::msg::Bool trig_msg;
    trig_msg.data = s.active;
    pub_recovery_trigger_bool_->publish(trig_msg);

    // Publish status strings / debug snapshots (throttled by change + periodic)
    maybe_publish_state_and_status_(s, now_ros);

    // One-shot log edges
    if (s.started) {
      RCLCPP_WARN(
        this->get_logger(),
        "Recovery STARTED | phase=%s reason=%s attempts=%u",
        phase_to_cstr_(s.phase), reason_to_cstr_(s.reason), s.attempts_in_streak);
    }
    if (s.completed) {
      RCLCPP_INFO(
        this->get_logger(),
        "Recovery COMPLETED | attempts=%u cooldown=%.2fs",
        s.attempts_in_streak, s.cooldown_remaining_sec);
    }
    if (s.aborted) {
      RCLCPP_WARN(
        this->get_logger(),
        "Recovery ABORTED | reason=%s cooldown=%.2fs",
        reason_to_cstr_(s.reason), s.cooldown_remaining_sec);
    }
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  bool read_input_(const TimedBool & tb, bool stale_default) const
  {
    if (!tb.has_msg) {
      return stale_default;
    }
    const double age = (this->now() - tb.stamp).seconds();
    if (!std::isfinite(age) || age < 0.0 || age > input_timeout_sec_) {
      return stale_default;
    }
    return tb.value;
  }

  geometry_msgs::msg::Twist make_cmd_from_status_(const RecoveryManagerStatus & s) const
  {
    geometry_msgs::msg::Twist cmd{};

    if (!s.active) {
      if (publish_zero_on_abort_or_done_) {
        return cmd;  // zero
      }
      return cmd;    // still zero by design
    }

    // Manager gives action flags + turn sign; node maps to actual magnitudes
    if (s.action.command_backup) {
      cmd.linear.x = -std::abs(backup_speed_m_s_);
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
    } else if (s.action.command_turn) {
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = static_cast<double>(s.action.turn_sign) * std::abs(turn_speed_rad_s_);
    } else if (s.action.command_stop) {
      // explicit zero twist
    } else {
      // default zero
    }

    return cmd;
  }

  void maybe_publish_state_and_status_(const RecoveryManagerStatus & s, const rclcpp::Time & now)
  {
    // State snapshot string
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(3);

    ss << "phase=" << phase_to_cstr_(s.phase)
       << " reason=" << reason_to_cstr_(s.reason)
       << " active=" << (s.active ? "true" : "false")
       << " cooldown=" << (s.in_cooldown ? "true" : "false")
       << " trigger_pending=" << (s.trigger_pending ? "true" : "false")
       << " attempts=" << s.attempts_in_streak
       << " phase_t=" << s.phase_elapsed_sec
       << " active_t=" << s.active_elapsed_sec
       << " cooldown_left=" << s.cooldown_remaining_sec
       << " action=[backup:" << (s.action.command_backup ? "1" : "0")
       << ",turn:" << (s.action.command_turn ? "1" : "0")
       << ",stop:" << (s.action.command_stop ? "1" : "0")
       << ",turn_sign:" << s.action.turn_sign << "]";

    const std::string snapshot = ss.str();
    const bool changed = (snapshot != last_state_snapshot_);
    const bool periodic = (!last_state_pub_time_.nanoseconds()) ||
                          ((now - last_state_pub_time_).seconds() >= 0.5);

    if (changed || periodic) {
      std_msgs::msg::String msg;
      msg.data = snapshot;
      pub_recovery_state_->publish(msg);
      last_state_snapshot_ = snapshot;
      last_state_pub_time_ = now;
    }

    // Short status text (phase/reason)
    std::ostringstream status;
    status << phase_to_cstr_(s.phase) << ":" << reason_to_cstr_(s.reason);
    publish_status_text_(status.str());
  }

  void publish_state_snapshot_(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    pub_recovery_state_->publish(msg);
    last_state_snapshot_ = text;
    last_state_pub_time_ = this->now();
  }

  void publish_status_text_(const std::string & text)
  {
    if (text == last_status_text_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    pub_recovery_status_->publish(msg);
    last_status_text_ = text;
  }

  void sanitize_params_()
  {
    if (!std::isfinite(loop_rate_hz_) || loop_rate_hz_ <= 0.0) {
      loop_rate_hz_ = 30.0;
    }
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0) {
      input_timeout_sec_ = 0.5;
    }
    if (!std::isfinite(backup_speed_m_s_) || backup_speed_m_s_ < 0.0) {
      backup_speed_m_s_ = 0.06;
    }
    if (!std::isfinite(turn_speed_rad_s_) || turn_speed_rad_s_ < 0.0) {
      turn_speed_rad_s_ = 0.35;
    }
    if (topic_stuck_detected_.empty()) topic_stuck_detected_ = topic_names::kStuckDetected;
    if (topic_safety_stop_.empty()) topic_safety_stop_ = topic_names::kSafetyStop;
    if (topic_cmd_vel_recovery_.empty()) topic_cmd_vel_recovery_ = topic_names::kCmdVelRecovery;
    if (topic_recovery_state_.empty()) topic_recovery_state_ = topic_names::kRecoveryState;
    if (topic_recovery_status_.empty()) topic_recovery_status_ = topic_names::kRecoveryStatus;
    if (topic_recovery_trigger_.empty()) topic_recovery_trigger_ = topic_names::kRecoveryTrigger;
  }

  static const char * phase_to_cstr_(RecoveryPhase p)
  {
    switch (p) {
      case RecoveryPhase::kIdle: return "idle";
      case RecoveryPhase::kArmed: return "armed";
      case RecoveryPhase::kBackingUp: return "backing_up";
      case RecoveryPhase::kSettling: return "settling";
      case RecoveryPhase::kTurning: return "turning";
      case RecoveryPhase::kComplete: return "complete";
      case RecoveryPhase::kAborted: return "aborted";
      case RecoveryPhase::kCooldown: return "cooldown";
      default: return "unknown";
    }
  }

  static const char * reason_to_cstr_(RecoveryReason r)
  {
    switch (r) {
      case RecoveryReason::kNone: return "none";
      case RecoveryReason::kStuckDetected: return "stuck_detected";
      case RecoveryReason::kNoProgress: return "no_progress";
      case RecoveryReason::kSafetyStopActive: return "safety_stop_active";
      case RecoveryReason::kTooManyAttempts: return "too_many_attempts";
      case RecoveryReason::kCooldownActive: return "cooldown_active";
      case RecoveryReason::kInvalidTime: return "invalid_time";
      case RecoveryReason::kExternalCancel: return "external_cancel";
      case RecoveryReason::kManualMode: return "manual_mode";
      case RecoveryReason::kTimeout: return "timeout";
      default: return "unknown";
    }
  }

private:
  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_recovery_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_recovery_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_recovery_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_recovery_trigger_bool_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stuck_detected_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_manual_override_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_motion_allowed_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_no_progress_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_backup_completed_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_turn_completed_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Topic names
  std::string topic_stuck_detected_;
  std::string topic_recovery_trigger_;
  std::string topic_safety_stop_;
  std::string topic_cmd_vel_recovery_;
  std::string topic_recovery_state_;
  std::string topic_recovery_status_;
  std::string topic_manual_override_;
  std::string topic_motion_allowed_;
  std::string topic_no_progress_;
  std::string topic_backup_completed_;
  std::string topic_turn_completed_;

  // Params
  double loop_rate_hz_{30.0};
  double input_timeout_sec_{0.5};

  bool stale_stuck_as_false_{true};
  bool stale_no_progress_as_false_{true};
  bool stale_safety_stop_as_true_{false};
  bool stale_manual_override_as_true_{false};
  bool stale_motion_allowed_as_false_{false};

  double backup_speed_m_s_{0.06};
  double turn_speed_rad_s_{0.35};
  bool publish_zero_when_idle_{true};
  bool publish_zero_on_abort_or_done_{true};

  bool use_external_backup_completed_{false};
  bool use_external_turn_completed_{false};

  // Runtime
  RecoveryManager manager_{};

  TimedBool stuck_detected_{};
  TimedBool safety_stop_{};
  TimedBool manual_override_{};
  TimedBool motion_allowed_{};
  TimedBool no_progress_{};
  TimedBool backup_completed_{};
  TimedBool turn_completed_{};

  bool has_motion_allowed_topic_{false};
  bool has_no_progress_topic_{false};

  std::string last_state_snapshot_;
  std::string last_status_text_;
  rclcpp::Time last_state_pub_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::RecoveryManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}