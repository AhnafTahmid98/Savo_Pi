// =============================================================================
// Robot SAVO — savo_control / src/nodes/velocity_test_pattern_node.cpp (ROS 2 Jazzy)
// =============================================================================
// Purpose
// -------
// Publish deterministic velocity test patterns for bringup and validation of
// the Robot SAVO control pipeline.
//
// Typical usage
// -------------
// This node can publish to a selectable topic (default: /cmd_vel_test_pattern)
// and is usually wired into the control pipeline via twist_mux_node (AUTO mode)
// or directly during diagnostics.
//
// Examples:
//   ros2 run savo_control velocity_test_pattern_node
//   ros2 run savo_control velocity_test_pattern_node --ros-args -p mode:=square
//   ros2 run savo_control velocity_test_pattern_node --ros-args -p output_topic:=/cmd_vel_auto
//
// What it helps verify
// --------------------
// - twist_mux source selection (AUTO input)
// - cmd_vel_shaper limiting/slew behavior
// - savo_perception safety gate (/cmd_vel_safe)
// - mecanum holonomic motion signs (vx / vy / wz)
// - localization response during controlled motion tests
//
// Safety notes
// ------------
// - Start with small velocities and short durations.
// - Test with wheels lifted first, then floor tests in open area.
// - Keep E-stop / STOP command ready.
// =============================================================================

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/topic_names.hpp"

namespace savo_control
{

class VelocityTestPatternNode : public rclcpp::Node
{
public:
  VelocityTestPatternNode()
  : Node("velocity_test_pattern_node")
  {
    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", topic_names::kCmdVelTestPattern);

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 20.0);
    auto_start_ = this->declare_parameter<bool>("auto_start", false);
    loop_pattern_ = this->declare_parameter<bool>("loop_pattern", true);
    stop_on_safety_stop_ = this->declare_parameter<bool>("stop_on_safety_stop", true);
    zero_on_disable_ = this->declare_parameter<bool>("zero_on_disable", true);

    // Mode: "square", "axes", "spin", "straight", "custom_sequence"
    mode_ = to_upper_copy_(this->declare_parameter<std::string>("mode", "AXES"));

    // General motion amplitudes (safe defaults for indoor robot testing)
    vx_m_s_ = this->declare_parameter<double>("vx_m_s", 0.08);
    vy_m_s_ = this->declare_parameter<double>("vy_m_s", 0.06);
    wz_rad_s_ = this->declare_parameter<double>("wz_rad_s", 0.45);

    // Segment timing
    move_sec_ = this->declare_parameter<double>("move_sec", 2.0);
    pause_sec_ = this->declare_parameter<double>("pause_sec", 1.0);
    startup_delay_sec_ = this->declare_parameter<double>("startup_delay_sec", 0.5);

    // Optional total runtime limit (0 = unlimited)
    max_runtime_sec_ = this->declare_parameter<double>("max_runtime_sec", 0.0);

    // Topic subscriptions for runtime control / safety
    enable_topic_ = this->declare_parameter<std::string>(
      "enable_topic", topic_names::kAutoTestEnable);
    state_topic_ = this->declare_parameter<std::string>(
      "state_topic", topic_names::kAutoTestState);
    safety_stop_topic_ = this->declare_parameter<std::string>(
      "safety_stop_topic", topic_names::kSafetyStop);

    // Pattern-specific extras
    square_include_rotation_ =
      this->declare_parameter<bool>("square.include_rotation", false);
    axes_include_diagonals_ =
      this->declare_parameter<bool>("axes.include_diagonals", false);
    axes_include_reverse_ =
      this->declare_parameter<bool>("axes.include_reverse", true);

    spin_alternate_direction_ =
      this->declare_parameter<bool>("spin.alternate_direction", true);

    sanitize_params_();

    // -------------------------------------------------------------------------
    // Publishers / Subscribers
    // -------------------------------------------------------------------------
    pub_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
      output_topic_, rclcpp::QoS(10));

    pub_state_ = this->create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(10).transient_local());

    sub_enable_ = this->create_subscription<std_msgs::msg::Bool>(
      enable_topic_, rclcpp::QoS(10),
      std::bind(&VelocityTestPatternNode::on_enable_, this, std::placeholders::_1));

    sub_safety_stop_ = this->create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_, rclcpp::QoS(10),
      std::bind(&VelocityTestPatternNode::on_safety_stop_, this, std::placeholders::_1));

    // -------------------------------------------------------------------------
    // Build pattern + timer
    // -------------------------------------------------------------------------
    build_pattern_();

    enabled_ = auto_start_;
    if (enabled_) {
      reset_run_state_(this->now());
    }

    const auto period_ms =
      static_cast<int>(std::round(1000.0 / std::max(1.0, publish_rate_hz_)));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(std::max(1, period_ms)),
      std::bind(&VelocityTestPatternNode::on_timer_, this));

    publish_state_(true);

    RCLCPP_INFO(
      this->get_logger(),
      "velocity_test_pattern_node started | mode=%s | out=%s | rate=%.1f Hz | auto_start=%s",
      mode_.c_str(),
      output_topic_.c_str(),
      publish_rate_hz_,
      auto_start_ ? "true" : "false");
  }

private:
  struct Segment
  {
    std::string name;
    geometry_msgs::msg::Twist cmd;
    double duration_sec{1.0};
  };

  // ---------------------------------------------------------------------------
  // Subscribers
  // ---------------------------------------------------------------------------
  void on_enable_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const bool new_enabled = msg->data;
    if (new_enabled == enabled_) {
      return;
    }

    enabled_ = new_enabled;

    if (enabled_) {
      reset_run_state_(this->now());
      RCLCPP_INFO(this->get_logger(), "Velocity test pattern ENABLED");
    } else {
      RCLCPP_INFO(this->get_logger(), "Velocity test pattern DISABLED");
      if (zero_on_disable_) {
        pub_twist_->publish(zero_twist_());
      }
    }

    publish_state_(true);
  }

  void on_safety_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    safety_stop_active_ = msg->data;
    safety_stop_stamp_ = this->now();
  }

  // ---------------------------------------------------------------------------
  // Timer loop
  // ---------------------------------------------------------------------------
  void on_timer_()
  {
    const rclcpp::Time now = this->now();

    geometry_msgs::msg::Twist out = zero_twist_();

    if (!enabled_) {
      // Keep publishing zero when disabled for predictable behavior
      pub_twist_->publish(out);
      publish_state_(false);
      return;
    }

    // Optional runtime limit
    if (max_runtime_sec_ > 0.0 && run_start_stamp_.nanoseconds() != 0) {
      const double run_age = (now - run_start_stamp_).seconds();
      if (std::isfinite(run_age) && run_age >= max_runtime_sec_) {
        enabled_ = false;
        RCLCPP_INFO(this->get_logger(), "Velocity test pattern stopped (max_runtime_sec reached)");
        pub_twist_->publish(zero_twist_());
        publish_state_(true);
        return;
      }
    }

    // Startup delay
    if (startup_delay_sec_ > 0.0 && cycle_start_stamp_.nanoseconds() != 0) {
      const double startup_age = (now - cycle_start_stamp_).seconds();
      if (std::isfinite(startup_age) && startup_age < startup_delay_sec_) {
        pub_twist_->publish(zero_twist_());
        publish_state_(false);
        return;
      }
    }

    // Optional safety stop behavior
    if (stop_on_safety_stop_ && is_recent_safety_stop_(now) && safety_stop_active_) {
      out = zero_twist_();
      pub_twist_->publish(out);

      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1500,
        "Safety stop active -> publishing zero test command");
      publish_state_(false);
      return;
    }

    if (segments_.empty()) {
      pub_twist_->publish(zero_twist_());
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "No pattern segments configured");
      publish_state_(false);
      return;
    }

    // Advance current segment based on elapsed time
    advance_segments_(now);

    // Output current segment command
    if (segment_index_ < segments_.size()) {
      out = segments_[segment_index_].cmd;
    } else {
      out = zero_twist_();
    }

    pub_twist_->publish(out);

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Pattern[%zu/%zu] %s -> vx=%.3f vy=%.3f wz=%.3f",
      static_cast<size_t>(segment_index_ + 1), segments_.size(),
      (segment_index_ < segments_.size() ? segments_[segment_index_].name.c_str() : "DONE"),
      out.linear.x, out.linear.y, out.angular.z);

    publish_state_(false);
  }

  // ---------------------------------------------------------------------------
  // Pattern builder
  // ---------------------------------------------------------------------------
  void build_pattern_()
  {
    segments_.clear();

    if (mode_ == "STRAIGHT") {
      add_move_pause_("forward",  +vx_m_s_, 0.0, 0.0);
      if (axes_include_reverse_) {
        add_move_pause_("backward", -vx_m_s_, 0.0, 0.0);
      }
    } else if (mode_ == "SPIN") {
      add_move_pause_("spin_ccw", 0.0, 0.0, +wz_rad_s_);
      if (spin_alternate_direction_) {
        add_move_pause_("spin_cw", 0.0, 0.0, -wz_rad_s_);
      }
    } else if (mode_ == "SQUARE") {
      // Holonomic square path (vx/vy only), optional corner spin
      add_move_pause_("forward",  +vx_m_s_, 0.0, 0.0);
      if (square_include_rotation_) add_move_pause_("turn_ccw", 0.0, 0.0, +wz_rad_s_);

      add_move_pause_("left",     0.0, +vy_m_s_, 0.0);
      if (square_include_rotation_) add_move_pause_("turn_ccw", 0.0, 0.0, +wz_rad_s_);

      add_move_pause_("backward", -vx_m_s_, 0.0, 0.0);
      if (square_include_rotation_) add_move_pause_("turn_ccw", 0.0, 0.0, +wz_rad_s_);

      add_move_pause_("right",    0.0, -vy_m_s_, 0.0);
      if (square_include_rotation_) add_move_pause_("turn_ccw", 0.0, 0.0, +wz_rad_s_);
    } else {  // Default AXES
      add_move_pause_("forward",  +vx_m_s_, 0.0, 0.0);
      if (axes_include_reverse_) add_move_pause_("backward", -vx_m_s_, 0.0, 0.0);

      add_move_pause_("left", 0.0, +vy_m_s_, 0.0);
      if (axes_include_reverse_) add_move_pause_("right", 0.0, -vy_m_s_, 0.0);

      add_move_pause_("spin_ccw", 0.0, 0.0, +wz_rad_s_);
      if (axes_include_reverse_) add_move_pause_("spin_cw", 0.0, 0.0, -wz_rad_s_);

      if (axes_include_diagonals_) {
        add_move_pause_("diag_fwd_left",  +vx_m_s_, +vy_m_s_, 0.0);
        add_move_pause_("diag_fwd_right", +vx_m_s_, -vy_m_s_, 0.0);
        if (axes_include_reverse_) {
          add_move_pause_("diag_back_left",  -vx_m_s_, +vy_m_s_, 0.0);
          add_move_pause_("diag_back_right", -vx_m_s_, -vy_m_s_, 0.0);
        }
      }
    }

    if (segments_.empty()) {
      // Safe fallback
      Segment s;
      s.name = "idle";
      s.cmd = zero_twist_();
      s.duration_sec = 1.0;
      segments_.push_back(s);
    }
  }

  void add_move_pause_(const std::string & name, double vx, double vy, double wz)
  {
    Segment move;
    move.name = name;
    move.cmd = make_twist_(vx, vy, wz);
    move.duration_sec = std::max(0.01, move_sec_);
    segments_.push_back(move);

    if (pause_sec_ > 0.0) {
      Segment pause;
      pause.name = name + "_pause";
      pause.cmd = zero_twist_();
      pause.duration_sec = std::max(0.01, pause_sec_);
      segments_.push_back(pause);
    }
  }

  // ---------------------------------------------------------------------------
  // Segment timing / sequence progression
  // ---------------------------------------------------------------------------
  void reset_run_state_(const rclcpp::Time & now)
  {
    run_start_stamp_ = now;
    cycle_start_stamp_ = now;
    segment_start_stamp_ = now;
    segment_index_ = 0;
    cycle_count_ = 0;
  }

  void advance_segments_(const rclcpp::Time & now)
  {
    if (segments_.empty()) {
      return;
    }

    if (segment_start_stamp_.nanoseconds() == 0) {
      segment_start_stamp_ = now;
    }

    // Startup delay shifts the effective start of pattern execution.
    const double startup_elapsed =
      (cycle_start_stamp_.nanoseconds() == 0) ? 0.0 : (now - cycle_start_stamp_).seconds();

    if (startup_delay_sec_ > 0.0 && std::isfinite(startup_elapsed) && startup_elapsed < startup_delay_sec_) {
      return;
    }

    // Advance across multiple segments if timer period is larger than segment duration.
    while (segment_index_ < segments_.size()) {
      const double age = (now - segment_start_stamp_).seconds();
      const double dur = std::max(0.01, segments_[segment_index_].duration_sec);

      if (!std::isfinite(age) || age < dur) {
        break;
      }

      segment_start_stamp_ = segment_start_stamp_ + rclcpp::Duration::from_seconds(dur);
      ++segment_index_;

      if (segment_index_ >= segments_.size()) {
        if (loop_pattern_) {
          ++cycle_count_;
          segment_index_ = 0;
          cycle_start_stamp_ = now;
          segment_start_stamp_ = now;
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Velocity test pattern loop cycle=%zu", cycle_count_);
        } else {
          enabled_ = false;
          pub_twist_->publish(zero_twist_());
          RCLCPP_INFO(this->get_logger(), "Velocity test pattern completed");
          publish_state_(true);
          break;
        }
      }
    }
  }

  // ---------------------------------------------------------------------------
  // State publishing (simple human-readable string)
  // ---------------------------------------------------------------------------
  void publish_state_(bool force)
  {
    std::string state = build_state_string_();

    if (!force && state == last_state_msg_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = state;
    pub_state_->publish(msg);
    last_state_msg_ = std::move(state);
  }

  std::string build_state_string_() const
  {
    std::string s = "enabled=" + bool_str_(enabled_);
    s += " mode=" + mode_;
    s += " safety_stop=" + bool_str_(safety_stop_active_);
    s += " segment=";

    if (segment_index_ < segments_.size()) {
      s += segments_[segment_index_].name;
      s += "(" + std::to_string(segment_index_ + 1) + "/" + std::to_string(segments_.size()) + ")";
    } else {
      s += "none";
    }

    s += " loop=" + bool_str_(loop_pattern_);
    s += " cycles=" + std::to_string(cycle_count_);
    return s;
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  void sanitize_params_()
  {
    if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
      publish_rate_hz_ = 20.0;
    }

    if (!std::isfinite(vx_m_s_)) {
      vx_m_s_ = 0.08;
    }
    if (!std::isfinite(vy_m_s_)) {
      vy_m_s_ = 0.06;
    }
    if (!std::isfinite(wz_rad_s_)) {
      wz_rad_s_ = 0.45;
    }

    // Keep magnitudes positive; sign is applied by pattern builder.
    vx_m_s_ = std::abs(vx_m_s_);
    vy_m_s_ = std::abs(vy_m_s_);
    wz_rad_s_ = std::abs(wz_rad_s_);

    if (!std::isfinite(move_sec_) || move_sec_ <= 0.0) {
      move_sec_ = 2.0;
    }
    if (!std::isfinite(pause_sec_) || pause_sec_ < 0.0) {
      pause_sec_ = 1.0;
    }
    if (!std::isfinite(startup_delay_sec_) || startup_delay_sec_ < 0.0) {
      startup_delay_sec_ = 0.5;
    }
    if (!std::isfinite(max_runtime_sec_) || max_runtime_sec_ < 0.0) {
      max_runtime_sec_ = 0.0;
    }

    if (mode_ != "AXES" && mode_ != "SQUARE" && mode_ != "SPIN" &&
        mode_ != "STRAIGHT" && mode_ != "CUSTOM_SEQUENCE")
    {
      RCLCPP_WARN(this->get_logger(), "Unknown mode '%s', falling back to AXES", mode_.c_str());
      mode_ = "AXES";
    }

    if (output_topic_.empty()) {
      output_topic_ = topic_names::kCmdVelTestPattern;
    }
    if (enable_topic_.empty()) {
      enable_topic_ = topic_names::kAutoTestEnable;
    }
    if (state_topic_.empty()) {
      state_topic_ = topic_names::kAutoTestState;
    }
    if (safety_stop_topic_.empty()) {
      safety_stop_topic_ = topic_names::kSafetyStop;
    }
  }

  bool is_recent_safety_stop_(const rclcpp::Time & now) const
  {
    if (safety_stop_stamp_.nanoseconds() == 0) {
      return false;
    }
    const double age = (now - safety_stop_stamp_).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= 1.0;
  }

  static geometry_msgs::msg::Twist make_twist_(double vx, double vy, double wz)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = finite_or_zero_(vx);
    t.linear.y = finite_or_zero_(vy);
    t.linear.z = 0.0;
    t.angular.x = 0.0;
    t.angular.y = 0.0;
    t.angular.z = finite_or_zero_(wz);
    return t;
  }

  static geometry_msgs::msg::Twist zero_twist_()
  {
    return geometry_msgs::msg::Twist{};
  }

  static double finite_or_zero_(double v)
  {
    return std::isfinite(v) ? v : 0.0;
  }

  static std::string to_upper_copy_(std::string s)
  {
    std::transform(
      s.begin(), s.end(), s.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
  }

  static std::string bool_str_(bool v)
  {
    return v ? "true" : "false";
  }

private:
  // Publishers/Subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_safety_stop_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params
  std::string output_topic_;
  std::string enable_topic_;
  std::string state_topic_;
  std::string safety_stop_topic_;
  std::string mode_{"AXES"};

  double publish_rate_hz_{20.0};
  bool auto_start_{false};
  bool loop_pattern_{true};
  bool stop_on_safety_stop_{true};
  bool zero_on_disable_{true};

  double vx_m_s_{0.08};
  double vy_m_s_{0.06};
  double wz_rad_s_{0.45};

  double move_sec_{2.0};
  double pause_sec_{1.0};
  double startup_delay_sec_{0.5};
  double max_runtime_sec_{0.0};

  bool square_include_rotation_{false};
  bool axes_include_diagonals_{false};
  bool axes_include_reverse_{true};
  bool spin_alternate_direction_{true};

  // Runtime state
  bool enabled_{false};
  bool safety_stop_active_{false};
  rclcpp::Time safety_stop_stamp_{0, 0, RCL_ROS_TIME};

  std::vector<Segment> segments_;
  size_t segment_index_{0};
  size_t cycle_count_{0};

  rclcpp::Time run_start_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time cycle_start_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time segment_start_stamp_{0, 0, RCL_ROS_TIME};

  std::string last_state_msg_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_control::VelocityTestPatternNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}