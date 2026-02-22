// =============================================================================
// Robot SAVO — savo_control / src/nodes/backup_escape_node.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Simple local backup/escape behavior node for Robot Savo.
//
// This node is a practical pre-Nav2 recovery primitive. It can be used:
// - standalone (manual trigger)
// - as a helper test node while validating your recovery pipeline
// - later alongside recovery_manager_node (manager decides *when*;
//   this node performs a deterministic backup/turn maneuver)
//
// Behavior (default)
// ------------------
// IDLE -> BACKUP -> SETTLE -> TURN -> DONE -> IDLE
//
// Safety
// ------
// - Subscribes to /safety/stop and immediately aborts motion if active
// - Command timeout style behavior is implicit through state machine timing
// - Publishes zero Twist when inactive (configurable)
//
// Notes
// -----
// - This node publishes to /cmd_vel_recovery by default
// - It is ROS-only orchestration; no direct motor/PCA9685 code here
// - For real hardware, keep speeds very low at first
// =============================================================================

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/topic_names.hpp"

using namespace std::chrono_literals;

namespace savo_control
{

class BackupEscapeNode : public rclcpp::Node
{
public:
  BackupEscapeNode()
  : Node("backup_escape_node")
  {
    // -------------------------------------------------------------------------
    // Parameters: topics
    // -------------------------------------------------------------------------
    topic_trigger_ = declare_parameter<std::string>(
      "topics.trigger", topic_names::kRecoveryTrigger);        // Bool (manual trigger)
    topic_cancel_ = declare_parameter<std::string>(
      "topics.cancel", "");                                    // optional Bool
    topic_safety_stop_ = declare_parameter<std::string>(
      "topics.safety_stop", topic_names::kSafetyStop);
    topic_slowdown_factor_ = declare_parameter<std::string>(
      "topics.slowdown_factor", topic_names::kSafetySlowdownFactor);  // optional Float32
    topic_cmd_out_ = declare_parameter<std::string>(
      "topics.cmd_out", topic_names::kCmdVelRecovery);
    topic_state_ = declare_parameter<std::string>(
      "topics.state", topic_names::kRecoveryState);
    topic_status_ = declare_parameter<std::string>(
      "topics.status", topic_names::kRecoveryStatus);

    // Optional completion feedback topics (useful if recovery_manager_node consumes them later)
    topic_backup_done_ = declare_parameter<std::string>(
      "topics.backup_completed", "/savo_control/backup_completed");
    topic_turn_done_ = declare_parameter<std::string>(
      "topics.turn_completed", "/savo_control/turn_completed");

    // -------------------------------------------------------------------------
    // Parameters: loop + state durations
    // -------------------------------------------------------------------------
    loop_rate_hz_ = declare_parameter<double>("loop_rate_hz", 30.0);

    backup_duration_sec_ = declare_parameter<double>("behavior.backup_duration_sec", 0.50);
    settle_duration_sec_ = declare_parameter<double>("behavior.settle_duration_sec", 0.20);
    turn_duration_sec_   = declare_parameter<double>("behavior.turn_duration_sec", 0.35);

    enable_turn_phase_ = declare_parameter<bool>("behavior.enable_turn_phase", true);
    auto_reset_after_done_ = declare_parameter<bool>("behavior.auto_reset_after_done", true);
    done_hold_sec_ = declare_parameter<double>("behavior.done_hold_sec", 0.20);

    // -------------------------------------------------------------------------
    // Parameters: command magnitudes
    // -------------------------------------------------------------------------
    backup_speed_m_s_ = declare_parameter<double>("command.backup_speed_m_s", 0.05);
    turn_speed_rad_s_ = declare_parameter<double>("command.turn_speed_rad_s", 0.30);
    turn_sign_ = declare_parameter<int>("command.turn_sign", +1);  // +1 CCW, -1 CW
    respect_slowdown_factor_ = declare_parameter<bool>("command.respect_slowdown_factor", false);
    min_slowdown_factor_ = declare_parameter<double>("command.min_slowdown_factor", 0.20);

    publish_zero_when_idle_ = declare_parameter<bool>("publish_zero_when_idle", true);

    // Trigger behavior
    rising_edge_trigger_only_ = declare_parameter<bool>("trigger.rising_edge_only", true);
    allow_retrigger_in_active_ = declare_parameter<bool>("trigger.allow_retrigger_in_active", false);

    // Input stale handling
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.50);
    stale_safety_stop_as_true_ = declare_parameter<bool>("stale.safety_stop_as_true", false);
    stale_slowdown_as_one_ = declare_parameter<bool>("stale.slowdown_as_one", true);

    sanitize_params_();

    // -------------------------------------------------------------------------
    // Publishers
    // -------------------------------------------------------------------------
    pub_cmd_out_ = create_publisher<geometry_msgs::msg::Twist>(topic_cmd_out_, rclcpp::QoS(10));
    pub_state_   = create_publisher<std_msgs::msg::String>(topic_state_, rclcpp::QoS(10).transient_local());
    pub_status_  = create_publisher<std_msgs::msg::String>(topic_status_, rclcpp::QoS(10));
    pub_backup_done_ = create_publisher<std_msgs::msg::Bool>(topic_backup_done_, rclcpp::QoS(10));
    pub_turn_done_   = create_publisher<std_msgs::msg::Bool>(topic_turn_done_, rclcpp::QoS(10));

    // -------------------------------------------------------------------------
    // Subscribers
    // -------------------------------------------------------------------------
    sub_trigger_ = create_subscription<std_msgs::msg::Bool>(
      topic_trigger_, rclcpp::QoS(10),
      std::bind(&BackupEscapeNode::on_trigger_, this, std::placeholders::_1));

    if (!topic_cancel_.empty()) {
      sub_cancel_ = create_subscription<std_msgs::msg::Bool>(
        topic_cancel_, rclcpp::QoS(10),
        std::bind(&BackupEscapeNode::on_cancel_, this, std::placeholders::_1));
    }

    sub_safety_stop_ = create_subscription<std_msgs::msg::Bool>(
      topic_safety_stop_, rclcpp::QoS(10),
      std::bind(&BackupEscapeNode::on_safety_stop_, this, std::placeholders::_1));

    if (!topic_slowdown_factor_.empty()) {
      sub_slowdown_factor_ = create_subscription<std_msgs::msg::Float32>(
        topic_slowdown_factor_, rclcpp::QoS(10),
        std::bind(&BackupEscapeNode::on_slowdown_factor_, this, std::placeholders::_1));
      has_slowdown_topic_ = true;
    }

    // -------------------------------------------------------------------------
    // Timer
    // -------------------------------------------------------------------------
    const auto period = std::chrono::duration<double>(1.0 / loop_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&BackupEscapeNode::on_timer_, this));

    publish_state_("startup");
    publish_status_("idle");
    publish_done_edges_(false, false);

    RCLCPP_INFO(
      get_logger(),
      "backup_escape_node started | trigger=%s safety=%s out=%s | loop=%.1fHz",
      topic_trigger_.c_str(), topic_safety_stop_.c_str(), topic_cmd_out_.c_str(), loop_rate_hz_);
  }

private:
  // ---------------------------------------------------------------------------
  // Internal state machine
  // ---------------------------------------------------------------------------
  enum class Phase : uint8_t
  {
    kIdle = 0,
    kBackingUp,
    kSettling,
    kTurning,
    kDone,
    kAborted
  };

  struct TimedBool
  {
    bool value{false};
    bool has_msg{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct TimedFloat
  {
    float value{1.0F};
    bool has_msg{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------
  void on_trigger_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    trigger_in_.value = msg->data;
    trigger_in_.has_msg = true;
    trigger_in_.stamp = now();
  }

  void on_cancel_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    cancel_in_.value = msg->data;
    cancel_in_.has_msg = true;
    cancel_in_.stamp = now();
  }

  void on_safety_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    safety_stop_in_.value = msg->data;
    safety_stop_in_.has_msg = true;
    safety_stop_in_.stamp = now();
  }

  void on_slowdown_factor_(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!msg) return;
    slowdown_in_.value = msg->data;
    slowdown_in_.has_msg = true;
    slowdown_in_.stamp = now();
  }

  // ---------------------------------------------------------------------------
  // Main loop
  // ---------------------------------------------------------------------------
  void on_timer_()
  {
    const rclcpp::Time now_ros = now();
    const double now_sec = now_ros.seconds();

    // Inputs (stale-aware)
    const bool trigger_level = read_bool_(trigger_in_, false);
    const bool cancel_level = (!topic_cancel_.empty()) ? read_bool_(cancel_in_, false) : false;
    const bool safety_stop = read_bool_(safety_stop_in_, stale_safety_stop_as_true_ ? true : false);
    const double slowdown = read_slowdown_();

    // Trigger edge logic
    bool trigger_fire = false;
    if (rising_edge_trigger_only_) {
      trigger_fire = (trigger_level && !prev_trigger_level_);
    } else {
      trigger_fire = trigger_level;
    }
    prev_trigger_level_ = trigger_level;

    // Safety has top priority
    if (safety_stop && is_active_phase_(phase_)) {
      transition_to_(Phase::kAborted, now_sec, "safety_stop");
    }

    // Cancel
    if (cancel_level && is_active_phase_(phase_)) {
      transition_to_(Phase::kAborted, now_sec, "cancel");
    }

    // Trigger start
    if (trigger_fire) {
      if (phase_ == Phase::kIdle || phase_ == Phase::kDone || phase_ == Phase::kAborted) {
        start_sequence_(now_sec);
      } else if (allow_retrigger_in_active_) {
        start_sequence_(now_sec);  // restart behavior
      }
    }

    // Phase progression
    const double phase_elapsed = now_sec - phase_start_sec_;
    bool backup_done_edge = false;
    bool turn_done_edge = false;

    switch (phase_) {
      case Phase::kIdle:
        break;

      case Phase::kBackingUp:
        if (phase_elapsed >= backup_duration_sec_) {
          backup_done_edge = true;
          transition_to_(Phase::kSettling, now_sec, "backup_done");
        }
        break;

      case Phase::kSettling:
        if (phase_elapsed >= settle_duration_sec_) {
          if (enable_turn_phase_) {
            transition_to_(Phase::kTurning, now_sec, "settle_done");
          } else {
            transition_to_(Phase::kDone, now_sec, "settle_done_no_turn");
          }
        }
        break;

      case Phase::kTurning:
        if (phase_elapsed >= turn_duration_sec_) {
          turn_done_edge = true;
          transition_to_(Phase::kDone, now_sec, "turn_done");
        }
        break;

      case Phase::kDone:
        if (auto_reset_after_done_ && phase_elapsed >= done_hold_sec_) {
          transition_to_(Phase::kIdle, now_sec, "auto_reset");
        }
        break;

      case Phase::kAborted:
        // Stay aborted briefly, then idle
        if (phase_elapsed >= done_hold_sec_) {
          transition_to_(Phase::kIdle, now_sec, "abort_reset");
        }
        break;

      default:
        transition_to_(Phase::kIdle, now_sec, "invalid_phase");
        break;
    }

    // Publish command
    geometry_msgs::msg::Twist cmd;
    if (is_active_phase_(phase_) && !safety_stop) {
      const double scale = respect_slowdown_factor_ ? slowdown : 1.0;

      if (phase_ == Phase::kBackingUp) {
        cmd.linear.x = -std::abs(backup_speed_m_s_) * scale;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
      } else if (phase_ == Phase::kSettling) {
        // zero twist
      } else if (phase_ == Phase::kTurning) {
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = static_cast<double>(normalized_turn_sign_()) *
                        std::abs(turn_speed_rad_s_) * scale;
      }
      pub_cmd_out_->publish(cmd);
    } else if (publish_zero_when_idle_) {
      pub_cmd_out_->publish(cmd);  // zero
    }

    // Publish done edges (one loop pulse)
    publish_done_edges_(backup_done_edge, turn_done_edge);

    // Publish state/status (change-driven + periodic)
    publish_state_and_status_maybe_(now_ros, phase_elapsed, safety_stop, trigger_level, slowdown);
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  void start_sequence_(double now_sec)
  {
    transition_to_(Phase::kBackingUp, now_sec, "triggered");
  }

  void transition_to_(Phase next, double now_sec, const std::string & reason)
  {
    if (phase_ != next) {
      last_transition_reason_ = reason;
      phase_ = next;
      phase_start_sec_ = now_sec;
      state_changed_ = true;
    }
  }

  bool is_active_phase_(Phase p) const
  {
    return p == Phase::kBackingUp || p == Phase::kSettling || p == Phase::kTurning;
  }

  bool read_bool_(const TimedBool & in, bool stale_default) const
  {
    if (!in.has_msg) {
      return stale_default;
    }
    const double age = (now() - in.stamp).seconds();
    if (!std::isfinite(age) || age < 0.0 || age > input_timeout_sec_) {
      return stale_default;
    }
    return in.value;
  }

  double read_slowdown_() const
  {
    if (!has_slowdown_topic_) {
      return 1.0;
    }
    if (!slowdown_in_.has_msg) {
      return stale_slowdown_as_one_ ? 1.0 : min_slowdown_factor_;
    }
    const double age = (now() - slowdown_in_.stamp).seconds();
    if (!std::isfinite(age) || age < 0.0 || age > input_timeout_sec_) {
      return stale_slowdown_as_one_ ? 1.0 : min_slowdown_factor_;
    }

    double x = static_cast<double>(slowdown_in_.value);
    if (!std::isfinite(x)) {
      x = stale_slowdown_as_one_ ? 1.0 : min_slowdown_factor_;
    }
    if (x < min_slowdown_factor_) x = min_slowdown_factor_;
    if (x > 1.0) x = 1.0;
    return x;
  }

  int normalized_turn_sign_() const
  {
    return (turn_sign_ >= 0) ? +1 : -1;
  }

  static const char * phase_to_cstr_(Phase p)
  {
    switch (p) {
      case Phase::kIdle: return "idle";
      case Phase::kBackingUp: return "backing_up";
      case Phase::kSettling: return "settling";
      case Phase::kTurning: return "turning";
      case Phase::kDone: return "done";
      case Phase::kAborted: return "aborted";
      default: return "unknown";
    }
  }

  void publish_done_edges_(bool backup_done, bool turn_done)
  {
    std_msgs::msg::Bool bmsg;
    bmsg.data = backup_done;
    pub_backup_done_->publish(bmsg);

    std_msgs::msg::Bool tmsg;
    tmsg.data = turn_done;
    pub_turn_done_->publish(tmsg);
  }

  void publish_state_(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    pub_state_->publish(msg);
    last_state_text_ = text;
    last_state_pub_time_ = now();
  }

  void publish_status_(const std::string & text)
  {
    if (text == last_status_text_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    pub_status_->publish(msg);
    last_status_text_ = text;
  }

  void publish_state_and_status_maybe_(
    const rclcpp::Time & now_ros,
    double phase_elapsed,
    bool safety_stop,
    bool trigger_level,
    double slowdown)
  {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);

    oss << "phase=" << phase_to_cstr_(phase_)
        << " active=" << (is_active_phase_(phase_) ? "true" : "false")
        << " t=" << phase_elapsed
        << " trigger=" << (trigger_level ? "1" : "0")
        << " safety_stop=" << (safety_stop ? "1" : "0")
        << " slowdown=" << slowdown
        << " reason=" << last_transition_reason_;

    const std::string state = oss.str();
    const bool changed = state_changed_ || (state != last_state_text_);
    const bool periodic = (!last_state_pub_time_.nanoseconds()) ||
                          ((now_ros - last_state_pub_time_).seconds() >= 0.5);

    if (changed || periodic) {
      publish_state_(state);
      state_changed_ = false;
    }

    std::ostringstream s;
    s << phase_to_cstr_(phase_) << ":" << last_transition_reason_;
    publish_status_(s.str());
  }

  void sanitize_params_()
  {
    if (!std::isfinite(loop_rate_hz_) || loop_rate_hz_ <= 0.0) loop_rate_hz_ = 30.0;
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0) input_timeout_sec_ = 0.5;

    if (!std::isfinite(backup_duration_sec_) || backup_duration_sec_ < 0.0) backup_duration_sec_ = 0.50;
    if (!std::isfinite(settle_duration_sec_) || settle_duration_sec_ < 0.0) settle_duration_sec_ = 0.20;
    if (!std::isfinite(turn_duration_sec_) || turn_duration_sec_ < 0.0) turn_duration_sec_ = 0.35;
    if (!std::isfinite(done_hold_sec_) || done_hold_sec_ < 0.0) done_hold_sec_ = 0.20;

    if (!std::isfinite(backup_speed_m_s_) || backup_speed_m_s_ < 0.0) backup_speed_m_s_ = 0.05;
    if (!std::isfinite(turn_speed_rad_s_) || turn_speed_rad_s_ < 0.0) turn_speed_rad_s_ = 0.30;

    if (!std::isfinite(min_slowdown_factor_) || min_slowdown_factor_ < 0.0) min_slowdown_factor_ = 0.20;
    if (min_slowdown_factor_ > 1.0) min_slowdown_factor_ = 1.0;

    if (topic_trigger_.empty()) topic_trigger_ = topic_names::kRecoveryTrigger;
    if (topic_safety_stop_.empty()) topic_safety_stop_ = topic_names::kSafetyStop;
    if (topic_cmd_out_.empty()) topic_cmd_out_ = topic_names::kCmdVelRecovery;
    if (topic_state_.empty()) topic_state_ = topic_names::kRecoveryState;
    if (topic_status_.empty()) topic_status_ = topic_names::kRecoveryStatus;
    if (topic_backup_done_.empty()) topic_backup_done_ = "/savo_control/backup_completed";
    if (topic_turn_done_.empty()) topic_turn_done_ = "/savo_control/turn_completed";
  }

private:
  // ROS pub/sub
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_out_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_backup_done_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_turn_done_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_trigger_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cancel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_slowdown_factor_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Topics
  std::string topic_trigger_;
  std::string topic_cancel_;
  std::string topic_safety_stop_;
  std::string topic_slowdown_factor_;
  std::string topic_cmd_out_;
  std::string topic_state_;
  std::string topic_status_;
  std::string topic_backup_done_;
  std::string topic_turn_done_;

  // Params
  double loop_rate_hz_{30.0};
  double input_timeout_sec_{0.5};

  double backup_duration_sec_{0.50};
  double settle_duration_sec_{0.20};
  double turn_duration_sec_{0.35};
  bool enable_turn_phase_{true};
  bool auto_reset_after_done_{true};
  double done_hold_sec_{0.20};

  double backup_speed_m_s_{0.05};
  double turn_speed_rad_s_{0.30};
  int turn_sign_{+1};
  bool respect_slowdown_factor_{false};
  double min_slowdown_factor_{0.20};

  bool publish_zero_when_idle_{true};
  bool rising_edge_trigger_only_{true};
  bool allow_retrigger_in_active_{false};

  bool stale_safety_stop_as_true_{false};
  bool stale_slowdown_as_one_{true};

  // Runtime inputs
  TimedBool trigger_in_{};
  TimedBool cancel_in_{};
  TimedBool safety_stop_in_{};
  TimedFloat slowdown_in_{};
  bool has_slowdown_topic_{false};

  // Runtime state
  Phase phase_{Phase::kIdle};
  double phase_start_sec_{0.0};
  bool prev_trigger_level_{false};
  bool state_changed_{false};
  std::string last_transition_reason_{"init"};

  std::string last_state_text_;
  std::string last_status_text_;
  rclcpp::Time last_state_pub_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::BackupEscapeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}