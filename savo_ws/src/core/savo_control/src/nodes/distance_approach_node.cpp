// Production front-distance approach controller.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/topic_names.hpp"

namespace savo_control
{

class DistanceApproachNode : public rclcpp::Node
{
public:
  DistanceApproachNode()
  : Node("distance_approach_node")
  {
    load_parameters_();
    sanitize_parameters_();

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_out_topic_, rclcpp::QoS(10));

    mode_pub_ = this->create_publisher<std_msgs::msg::String>(
      mode_cmd_topic_, rclcpp::QoS(10));

    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(10));

    distance_sub_f32_ = this->create_subscription<std_msgs::msg::Float32>(
      distance_topic_, rclcpp::QoS(10),
      std::bind(&DistanceApproachNode::on_distance_f32_, this, std::placeholders::_1));

    distance_sub_f64_ = this->create_subscription<std_msgs::msg::Float64>(
      distance_topic_, rclcpp::QoS(10),
      std::bind(&DistanceApproachNode::on_distance_f64_, this, std::placeholders::_1));

    safety_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_, rclcpp::QoS(10),
      std::bind(&DistanceApproachNode::on_safety_stop_, this, std::placeholders::_1));

    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      enable_topic_, rclcpp::QoS(10),
      std::bind(&DistanceApproachNode::on_enable_, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      target_topic_, rclcpp::QoS(10),
      std::bind(&DistanceApproachNode::on_target_, this, std::placeholders::_1));

    if (!enabled_) {
      state_ = "DISABLED";
    } else if (auto_start_) {
      start_approach_();
    }

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / loop_hz_));

    timer_ = this->create_wall_timer(
      period,
      std::bind(&DistanceApproachNode::on_timer_, this));

    RCLCPP_INFO(
      this->get_logger(),
      "distance_approach_node started | distance=%s | output=%s | target=%.2f m",
      distance_topic_.c_str(),
      cmd_vel_out_topic_.c_str(),
      target_distance_m_);
  }

  ~DistanceApproachNode() override
  {
    publish_zero_();
  }

private:
  struct PidState
  {
    double integral{0.0};
    double previous_error{0.0};
    double derivative_filtered{0.0};
    rclcpp::Time previous_time{};
    bool has_previous{false};
  };

  void load_parameters_()
  {
    enabled_ = this->declare_parameter<bool>("enabled", true);
    auto_start_ = this->declare_parameter<bool>("auto_start", false);

    distance_topic_ = this->declare_parameter<std::string>(
      "distance_topic", topic_names::kDepthMinFront);
    cmd_vel_out_topic_ = this->declare_parameter<std::string>(
      "cmd_vel_out_topic", topic_names::kCmdVelAuto);
    mode_cmd_topic_ = this->declare_parameter<std::string>(
      "mode_cmd_topic", topic_names::kControlModeCmd);
    safety_stop_topic_ = this->declare_parameter<std::string>(
      "safety_stop_topic", topic_names::kSafetyStop);
    status_topic_ = this->declare_parameter<std::string>(
      "status_topic", "/savo_control/distance_approach_status");

    enable_topic_ = this->declare_parameter<std::string>(
      "enable_topic", "/savo_control/distance_approach_enable");
    target_topic_ = this->declare_parameter<std::string>(
      "target_topic", "/savo_control/distance_approach_target");

    request_auto_mode_on_start_ = this->declare_parameter<bool>(
      "request_auto_mode_on_start", true);
    required_mode_ = this->declare_parameter<std::string>("required_mode", "AUTO");

    stop_on_goal_ = this->declare_parameter<bool>("stop_on_goal", true);
    stop_if_too_close_ = this->declare_parameter<bool>("stop_if_too_close", true);
    start_on_target_update_ = this->declare_parameter<bool>(
      "start_on_target_update", false);

    target_distance_m_ = this->declare_parameter<double>("target_distance_m", 0.60);
    tolerance_m_ = this->declare_parameter<double>("tolerance_m", 0.04);
    hold_time_s_ = this->declare_parameter<double>("hold_time_s", 0.40);
    hard_min_distance_m_ = this->declare_parameter<double>("hard_min_distance_m", 0.35);

    min_valid_distance_m_ = this->declare_parameter<double>("min_valid_distance_m", 0.05);
    max_valid_distance_m_ = this->declare_parameter<double>("max_valid_distance_m", 3.00);
    distance_timeout_s_ = this->declare_parameter<double>("distance_timeout_s", 0.40);

    smoothing_enabled_ = this->declare_parameter<bool>("smoothing_enabled", true);
    smoothing_alpha_ = this->declare_parameter<double>("smoothing_alpha", 0.45);

    kp_ = this->declare_parameter<double>("kp", 0.45);
    ki_ = this->declare_parameter<double>("ki", 0.00);
    kd_ = this->declare_parameter<double>("kd", 0.03);

    integral_limit_ = this->declare_parameter<double>("integral_limit", 0.15);
    integral_active_error_m_ = this->declare_parameter<double>(
      "integral_active_error_m", 0.20);
    derivative_filter_alpha_ = this->declare_parameter<double>(
      "derivative_filter_alpha", 0.35);

    max_forward_vx_ = this->declare_parameter<double>("max_forward_vx", 0.10);
    allow_reverse_ = this->declare_parameter<bool>("allow_reverse", false);
    max_reverse_vx_ = this->declare_parameter<double>("max_reverse_vx", 0.05);

    min_vx_when_active_ = this->declare_parameter<double>("min_vx_when_active", 0.04);
    disable_min_vx_below_error_m_ = this->declare_parameter<double>(
      "disable_min_vx_below_error_m", 0.08);

    loop_hz_ = this->declare_parameter<double>("loop_hz", 30.0);
    timeout_s_ = this->declare_parameter<double>("timeout_s", 10.0);
    stop_hold_s_ = this->declare_parameter<double>("stop_hold_s", 0.50);
    shutdown_zero_count_ = this->declare_parameter<int>("shutdown_zero_count", 5);

    respect_safety_stop_ = this->declare_parameter<bool>("respect_safety_stop", true);
    zero_on_invalid_distance_ = this->declare_parameter<bool>(
      "zero_on_invalid_distance", true);
    zero_on_distance_timeout_ = this->declare_parameter<bool>(
      "zero_on_distance_timeout", true);

    publish_status_ = this->declare_parameter<bool>("publish_status", true);
    status_hz_ = this->declare_parameter<double>("status_hz", 5.0);
    log_throttle_s_ = this->declare_parameter<double>("log_throttle_s", 2.0);
  }

  void sanitize_parameters_()
  {
    loop_hz_ = std::clamp(loop_hz_, 1.0, 100.0);
    status_hz_ = std::clamp(status_hz_, 0.2, 30.0);

    target_distance_m_ = std::max(0.05, target_distance_m_);
    tolerance_m_ = std::max(0.005, tolerance_m_);
    hold_time_s_ = std::max(0.0, hold_time_s_);
    hard_min_distance_m_ = std::max(0.01, hard_min_distance_m_);

    min_valid_distance_m_ = std::max(0.01, min_valid_distance_m_);
    max_valid_distance_m_ = std::max(min_valid_distance_m_ + 0.01, max_valid_distance_m_);
    distance_timeout_s_ = std::max(0.05, distance_timeout_s_);

    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.0, 1.0);
    derivative_filter_alpha_ = std::clamp(derivative_filter_alpha_, 0.0, 1.0);

    integral_limit_ = std::abs(integral_limit_);
    integral_active_error_m_ = std::abs(integral_active_error_m_);

    max_forward_vx_ = std::abs(max_forward_vx_);
    max_reverse_vx_ = std::abs(max_reverse_vx_);
    min_vx_when_active_ = std::abs(min_vx_when_active_);
    disable_min_vx_below_error_m_ = std::abs(disable_min_vx_below_error_m_);

    timeout_s_ = std::max(0.1, timeout_s_);
    stop_hold_s_ = std::max(0.0, stop_hold_s_);
    shutdown_zero_count_ = std::max(1, shutdown_zero_count_);
    log_throttle_s_ = std::max(0.1, log_throttle_s_);
  }

  void on_distance_f32_(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (msg) {
      update_distance_(static_cast<double>(msg->data));
    }
  }

  void on_distance_f64_(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (msg) {
      update_distance_(msg->data);
    }
  }

  void on_safety_stop_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg) {
      safety_stop_active_ = msg->data;
    }
  }

  void on_enable_(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (msg->data) {
      start_approach_();
      return;
    }

    state_ = "IDLE";
    pid_ = PidState{};
    publish_zero_();
    publish_status_(true);
  }

  void on_target_(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!msg || !std::isfinite(msg->data) || msg->data <= 0.0) {
      throttled_warn_("Ignored invalid distance target");
      return;
    }

    target_distance_m_ = msg->data;
    pid_ = PidState{};
    goal_entered_ = false;

    if (start_on_target_update_) {
      start_approach_();
    }
  }

  void update_distance_(double distance_m)
  {
    const auto now = this->now();
    last_distance_stamp_ = now;
    have_distance_msg_ = true;

    if (!is_valid_distance_(distance_m)) {
      distance_valid_ = false;
      latest_distance_m_ = std::numeric_limits<double>::quiet_NaN();
      state_ = "SENSOR_INVALID";
      throttled_warn_("Invalid distance reading");
      return;
    }

    distance_valid_ = true;
    latest_distance_m_ = distance_m;

    if (!have_filtered_distance_ || !smoothing_enabled_) {
      filtered_distance_m_ = distance_m;
      have_filtered_distance_ = true;
      return;
    }

    filtered_distance_m_ =
      smoothing_alpha_ * distance_m + (1.0 - smoothing_alpha_) * filtered_distance_m_;
  }

  void on_timer_()
  {
    if (!enabled_) {
      state_ = "DISABLED";
      publish_zero_();
      publish_status_(false);
      return;
    }

    if (state_ == "IDLE") {
      publish_zero_();
      publish_status_(false);
      return;
    }

    if (respect_safety_stop_ && safety_stop_active_) {
      state_ = "SAFETY_STOP";
      publish_zero_();
      publish_status_(false);
      return;
    }

    double distance_m = 0.0;
    if (!get_current_distance_(distance_m)) {
      publish_status_(false);
      return;
    }

    if (has_timed_out_()) {
      state_ = "TIMEOUT";
      publish_zero_();
      publish_status_(false);
      throttled_warn_("Distance approach timed out");
      return;
    }

    if (stop_if_too_close_ && distance_m < hard_min_distance_m_) {
      state_ = "STOPPED_TOO_CLOSE";
      publish_zero_();
      publish_status_(false);
      throttled_warn_("Distance below hard minimum");
      return;
    }

    const double error_m = distance_m - target_distance_m_;

    if (stop_on_goal_ && std::abs(error_m) <= tolerance_m_) {
      if (!goal_entered_) {
        goal_entered_ = true;
        goal_enter_stamp_ = this->now();
      }

      if ((this->now() - goal_enter_stamp_).seconds() >= hold_time_s_) {
        state_ = "GOAL_REACHED";
        publish_zero_();
        publish_status_(false);
        return;
      }
    } else {
      goal_entered_ = false;
    }

    state_ = "RUNNING";
    last_cmd_vx_ = compute_pid_vx_(error_m);
    publish_cmd_(last_cmd_vx_);
    publish_status_(false);
  }

  void start_approach_()
  {
    if (!enabled_) {
      state_ = "DISABLED";
      return;
    }

    pid_ = PidState{};
    started_stamp_ = this->now();
    have_started_stamp_ = true;
    goal_entered_ = false;
    state_ = "RUNNING";

    if (request_auto_mode_on_start_) {
      std_msgs::msg::String msg;
      msg.data = required_mode_;
      mode_pub_->publish(msg);
    }

    publish_status_(true);
  }

  bool get_current_distance_(double & distance_m)
  {
    if (!have_distance_msg_) {
      state_ = "SENSOR_STALE";
      if (zero_on_distance_timeout_) {
        publish_zero_();
      }
      throttled_warn_("No distance data received");
      return false;
    }

    if ((this->now() - last_distance_stamp_).seconds() > distance_timeout_s_) {
      state_ = "SENSOR_STALE";
      if (zero_on_distance_timeout_) {
        publish_zero_();
      }
      throttled_warn_("Distance data stale");
      return false;
    }

    if (!distance_valid_ || !have_filtered_distance_) {
      state_ = "SENSOR_INVALID";
      if (zero_on_invalid_distance_) {
        publish_zero_();
      }
      return false;
    }

    if (!is_valid_distance_(filtered_distance_m_)) {
      state_ = "SENSOR_INVALID";
      if (zero_on_invalid_distance_) {
        publish_zero_();
      }
      return false;
    }

    distance_m = filtered_distance_m_;
    return true;
  }

  bool has_timed_out_() const
  {
    if (!have_started_stamp_) {
      return false;
    }

    return (this->now() - started_stamp_).seconds() > timeout_s_;
  }

  double compute_pid_vx_(double error_m)
  {
    const auto now = this->now();
    double dt = 1.0 / loop_hz_;

    if (pid_.has_previous) {
      dt = std::max(1.0e-4, (now - pid_.previous_time).seconds());
    }

    if (std::abs(error_m) <= integral_active_error_m_) {
      pid_.integral += error_m * dt;
      pid_.integral = std::clamp(pid_.integral, -integral_limit_, integral_limit_);
    } else {
      pid_.integral = 0.0;
    }

    double derivative = 0.0;
    if (pid_.has_previous) {
      derivative = (error_m - pid_.previous_error) / dt;
    }

    pid_.derivative_filtered =
      derivative_filter_alpha_ * derivative +
      (1.0 - derivative_filter_alpha_) * pid_.derivative_filtered;

    const double raw =
      kp_ * error_m +
      ki_ * pid_.integral +
      kd_ * pid_.derivative_filtered;

    pid_.previous_error = error_m;
    pid_.previous_time = now;
    pid_.has_previous = true;

    double vx = 0.0;
    if (raw >= 0.0) {
      vx = std::min(raw, max_forward_vx_);
    } else if (allow_reverse_) {
      vx = std::max(raw, -max_reverse_vx_);
    }

    if (
      vx > 0.0 &&
      std::abs(error_m) > disable_min_vx_below_error_m_ &&
      vx < min_vx_when_active_)
    {
      vx = min_vx_when_active_;
    }

    return safe_float_(vx);
  }

  void publish_cmd_(double vx)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = safe_float_(vx);
    cmd_pub_->publish(msg);
  }

  void publish_zero_()
  {
    publish_cmd_(0.0);
    last_cmd_vx_ = 0.0;
  }

  void publish_status_(bool force)
  {
    if (!publish_status_) {
      return;
    }

    const auto now = this->now();
    const double period_s = 1.0 / status_hz_;

    if (
      !force &&
      have_last_status_stamp_ &&
      (now - last_status_stamp_).seconds() < period_s)
    {
      return;
    }

    have_last_status_stamp_ = true;
    last_status_stamp_ = now;

    std_msgs::msg::String msg;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "state=" << state_;
    out << "; distance_m=";

    if (have_filtered_distance_ && distance_valid_) {
      out << filtered_distance_m_;
    } else {
      out << "nan";
    }

    out << "; target_m=" << target_distance_m_;
    out << "; vx=" << last_cmd_vx_;
    out << "; safety_stop=" << (safety_stop_active_ ? "true" : "false");

    msg.data = out.str();
    status_pub_->publish(msg);
  }

  bool is_valid_distance_(double distance_m) const
  {
    return (
      std::isfinite(distance_m) &&
      distance_m >= min_valid_distance_m_ &&
      distance_m <= max_valid_distance_m_);
  }

  double safe_float_(double value) const
  {
    if (!std::isfinite(value)) {
      return 0.0;
    }
    return value;
  }

  void throttled_warn_(const std::string & text)
  {
    const auto now = this->now();
    if (
      !have_last_log_stamp_ ||
      (now - last_log_stamp_).seconds() >= log_throttle_s_)
    {
      have_last_log_stamp_ = true;
      last_log_stamp_ = now;
      RCLCPP_WARN(this->get_logger(), "%s", text.c_str());
    }
  }

  bool enabled_{true};
  bool auto_start_{false};
  bool request_auto_mode_on_start_{true};
  bool stop_on_goal_{true};
  bool stop_if_too_close_{true};
  bool start_on_target_update_{false};
  bool smoothing_enabled_{true};
  bool allow_reverse_{false};
  bool respect_safety_stop_{true};
  bool zero_on_invalid_distance_{true};
  bool zero_on_distance_timeout_{true};
  bool publish_status_{true};

  std::string distance_topic_{topic_names::kDepthMinFront};
  std::string cmd_vel_out_topic_{topic_names::kCmdVelAuto};
  std::string mode_cmd_topic_{topic_names::kControlModeCmd};
  std::string safety_stop_topic_{topic_names::kSafetyStop};
  std::string status_topic_{"/savo_control/distance_approach_status"};
  std::string enable_topic_{"/savo_control/distance_approach_enable"};
  std::string target_topic_{"/savo_control/distance_approach_target"};
  std::string required_mode_{"AUTO"};

  double target_distance_m_{0.60};
  double tolerance_m_{0.04};
  double hold_time_s_{0.40};
  double hard_min_distance_m_{0.35};

  double min_valid_distance_m_{0.05};
  double max_valid_distance_m_{3.00};
  double distance_timeout_s_{0.40};
  double smoothing_alpha_{0.45};

  double kp_{0.45};
  double ki_{0.0};
  double kd_{0.03};
  double integral_limit_{0.15};
  double integral_active_error_m_{0.20};
  double derivative_filter_alpha_{0.35};

  double max_forward_vx_{0.10};
  double max_reverse_vx_{0.05};
  double min_vx_when_active_{0.04};
  double disable_min_vx_below_error_m_{0.08};

  double loop_hz_{30.0};
  double timeout_s_{10.0};
  double stop_hold_s_{0.50};
  double status_hz_{5.0};
  double log_throttle_s_{2.0};

  int shutdown_zero_count_{5};

  std::string state_{"IDLE"};
  bool safety_stop_active_{false};

  bool have_distance_msg_{false};
  bool distance_valid_{false};
  bool have_filtered_distance_{false};
  double latest_distance_m_{std::numeric_limits<double>::quiet_NaN()};
  double filtered_distance_m_{std::numeric_limits<double>::quiet_NaN()};

  bool have_started_stamp_{false};
  bool goal_entered_{false};
  bool have_last_status_stamp_{false};
  bool have_last_log_stamp_{false};

  double last_cmd_vx_{0.0};

  rclcpp::Time last_distance_stamp_{};
  rclcpp::Time started_stamp_{};
  rclcpp::Time goal_enter_stamp_{};
  rclcpp::Time last_status_stamp_{};
  rclcpp::Time last_log_stamp_{};

  PidState pid_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_f32_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_f64_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::DistanceApproachNode>());
  rclcpp::shutdown();
  return 0;
}
