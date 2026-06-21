#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_control/command_limiter.hpp"
#include "savo_control/topic_names.hpp"

namespace
{

double now_seconds(const rclcpp::Node & node)
{
  return node.get_clock()->now().seconds();
}

geometry_msgs::msg::Twist zero_twist()
{
  geometry_msgs::msg::Twist msg;
  return msg;
}

geometry_msgs::msg::Twist command_to_twist(const savo_control::TwistCommand & command)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = command.vx;
  msg.linear.y = command.vy;
  msg.angular.z = command.wz;
  return msg;
}

std_msgs::msg::String string_msg(const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = value;
  return msg;
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

namespace savo_control
{

class VelocityTestPatternNode : public rclcpp::Node
{
public:
  VelocityTestPatternNode()
  : Node("velocity_test_pattern_node")
  {
    declare_parameters();
    load_parameters();

    limiter_.set_limits(make_limits());

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        const bool was_enabled = active_;
        active_ = msg->data;

        if (active_ && !was_enabled) {
          start_s_ = now_seconds(*this);
          last_phase_switch_s_ = start_s_;
          phase_active_ = true;
          completed_ = false;
        }

        if (!active_) {
          completed_ = false;
          cmd_pub_->publish(zero_twist());
          publish_state("disabled_by_command", now_seconds(*this), TwistCommand{});
        }
      });

    safety_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      safety_stop_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        safety_stop_ = msg->data;
        safety_stop_seen_ = true;
        safety_stop_stamp_s_ = now_seconds(*this);
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() {
        on_timer();
      });

    if (start_enabled_) {
      active_ = true;
      start_s_ = now_seconds(*this);
      last_phase_switch_s_ = start_s_;
      phase_active_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "velocity_test_pattern_node started | output=%s enable=%s",
      output_topic_.c_str(),
      enable_topic_.c_str());
  }

private:
  void declare_parameters()
  {
    declare_parameter<double>("publish_hz", 20.0);
    declare_parameter<bool>("start_enabled", false);
    declare_parameter<bool>("repeat", false);
    declare_parameter<bool>("publish_zero_when_inactive", true);
    declare_parameter<bool>("safety_stop_blocks_motion", true);

    declare_parameter<std::string>("pattern", "constant");

    declare_parameter<double>("vx", 0.08);
    declare_parameter<double>("vy", 0.0);
    declare_parameter<double>("wz", 0.0);

    declare_parameter<double>("duration_s", 2.0);
    declare_parameter<double>("active_duration_s", 0.50);
    declare_parameter<double>("rest_duration_s", 0.50);

    declare_parameter<double>("max_vx", 0.15);
    declare_parameter<double>("max_vy", 0.15);
    declare_parameter<double>("max_wz", 0.40);

    declare_parameter<std::string>("output_topic", topics::AUTO_TEST_CMD);
    declare_parameter<std::string>("enable_topic", topics::AUTO_TEST_ENABLE);
    declare_parameter<std::string>("state_topic", topics::AUTO_TEST_STATE);
    declare_parameter<std::string>("status_topic", "/savo_control/velocity_test_pattern_status");
    declare_parameter<std::string>("safety_stop_topic", topics::SAFETY_STOP);
  }

  void load_parameters()
  {
    publish_hz_ = positive_param("publish_hz", 20.0);

    start_enabled_ = get_parameter("start_enabled").as_bool();
    repeat_ = get_parameter("repeat").as_bool();
    publish_zero_when_inactive_ = get_parameter("publish_zero_when_inactive").as_bool();
    safety_stop_blocks_motion_ = get_parameter("safety_stop_blocks_motion").as_bool();

    pattern_ = get_parameter("pattern").as_string();

    vx_ = finite_param("vx", 0.08);
    vy_ = finite_param("vy", 0.0);
    wz_ = finite_param("wz", 0.0);

    duration_s_ = positive_param("duration_s", 2.0);
    active_duration_s_ = positive_param("active_duration_s", 0.50);
    rest_duration_s_ = nonnegative_param("rest_duration_s", 0.50);

    max_vx_ = nonnegative_param("max_vx", 0.15);
    max_vy_ = nonnegative_param("max_vy", 0.15);
    max_wz_ = nonnegative_param("max_wz", 0.40);

    output_topic_ = get_parameter("output_topic").as_string();
    enable_topic_ = get_parameter("enable_topic").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    safety_stop_topic_ = get_parameter("safety_stop_topic").as_string();
  }

  CommandLimits make_limits() const
  {
    CommandLimits limits;
    limits.max_vx = max_vx_;
    limits.max_vy = max_vy_;
    limits.max_wz = max_wz_;
    limits.use_symmetric_limits = true;
    return limits.sanitized();
  }

  void on_timer()
  {
    const double now_s = now_seconds(*this);

    if (safety_stop_blocks_motion_ && safety_stop_active(now_s)) {
      cmd_pub_->publish(zero_twist());
      publish_state("safety_stop", now_s, TwistCommand{});
      return;
    }

    if (!active_) {
      if (publish_zero_when_inactive_) {
        cmd_pub_->publish(zero_twist());
      }
      publish_state(completed_ ? "completed" : "inactive", now_s, TwistCommand{});
      return;
    }

    const double elapsed_s = now_s - start_s_;
    if (elapsed_s >= duration_s_) {
      if (repeat_) {
        start_s_ = now_s;
        last_phase_switch_s_ = now_s;
        phase_active_ = true;
        completed_ = false;
      } else {
        active_ = false;
        completed_ = true;
        cmd_pub_->publish(zero_twist());
        publish_state("completed", now_s, TwistCommand{});
        return;
      }
    }

    const TwistCommand command = command_for_now(now_s);
    const TwistCommand limited = limiter_.limit(command);

    cmd_pub_->publish(command_to_twist(limited));
    publish_state("running", now_s, limited);
  }

  TwistCommand command_for_now(const double now_s)
  {
    const std::string key = normalized_pattern();

    if (key == "PULSE") {
      update_pulse_phase(now_s);
      if (!phase_active_) {
        return TwistCommand{};
      }
    } else if (key == "REVERSE") {
      return make_twist_command(-std::abs(vx_), vy_, wz_);
    } else if (key == "STRAFE_LEFT") {
      return make_twist_command(0.0, std::abs(vy_ > 0.0 ? vy_ : vx_), wz_);
    } else if (key == "STRAFE_RIGHT") {
      return make_twist_command(0.0, -std::abs(vy_ > 0.0 ? vy_ : vx_), wz_);
    } else if (key == "ROTATE_LEFT") {
      return make_twist_command(0.0, 0.0, std::abs(wz_ > 0.0 ? wz_ : 0.25));
    } else if (key == "ROTATE_RIGHT") {
      return make_twist_command(0.0, 0.0, -std::abs(wz_ > 0.0 ? wz_ : 0.25));
    }

    return make_twist_command(vx_, vy_, wz_);
  }

  void update_pulse_phase(const double now_s)
  {
    const double phase_elapsed_s = now_s - last_phase_switch_s_;

    if (phase_active_ && phase_elapsed_s >= active_duration_s_) {
      phase_active_ = false;
      last_phase_switch_s_ = now_s;
      return;
    }

    if (!phase_active_ && phase_elapsed_s >= rest_duration_s_) {
      phase_active_ = true;
      last_phase_switch_s_ = now_s;
    }
  }

  std::string normalized_pattern() const
  {
    std::string out;
    out.reserve(pattern_.size());

    for (const char ch : pattern_) {
      if (ch == '-' || ch == ' ') {
        out.push_back('_');
      } else {
        out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
      }
    }

    return out;
  }

  bool safety_stop_active(const double now_s) const
  {
    if (!safety_stop_seen_) {
      return false;
    }

    if ((now_s - safety_stop_stamp_s_) > 0.50) {
      return false;
    }

    return safety_stop_;
  }

  void publish_state(
    const std::string & reason,
    const double now_s,
    const TwistCommand & command)
  {
    std::ostringstream state;
    state << "active=" << bool_text(active_)
          << "; completed=" << bool_text(completed_)
          << "; reason=" << reason
          << "; pattern=" << pattern_;

    state_pub_->publish(string_msg(state.str()));

    std::ostringstream status;
    status << "active=" << bool_text(active_)
           << "; completed=" << bool_text(completed_)
           << "; repeat=" << bool_text(repeat_)
           << "; phase_active=" << bool_text(phase_active_)
           << "; safety_stop=" << bool_text(safety_stop_active(now_s))
           << "; reason=" << reason
           << "; pattern=" << pattern_
           << "; elapsed_s=" << (active_ ? now_s - start_s_ : 0.0)
           << "; vx=" << command.vx
           << "; vy=" << command.vy
           << "; wz=" << command.wz
           << "; now_s=" << now_s;

    status_pub_->publish(string_msg(status.str()));
  }

  double finite_param(const std::string & name, const double fallback) const
  {
    const double value = get_parameter(name).as_double();
    return std::isfinite(value) ? value : fallback;
  }

  double positive_param(const std::string & name, const double fallback) const
  {
    const double value = get_parameter(name).as_double();
    if (!std::isfinite(value) || value <= 0.0) {
      return fallback;
    }

    return value;
  }

  double nonnegative_param(const std::string & name, const double fallback) const
  {
    const double value = get_parameter(name).as_double();
    if (!std::isfinite(value) || value < 0.0) {
      return fallback;
    }

    return value;
  }

  double publish_hz_{20.0};

  bool start_enabled_{false};
  bool active_{false};
  bool completed_{false};
  bool repeat_{false};
  bool phase_active_{true};
  bool publish_zero_when_inactive_{true};
  bool safety_stop_blocks_motion_{true};

  std::string pattern_{"constant"};

  double vx_{0.08};
  double vy_{0.0};
  double wz_{0.0};

  double duration_s_{2.0};
  double active_duration_s_{0.50};
  double rest_duration_s_{0.50};

  double max_vx_{0.15};
  double max_vy_{0.15};
  double max_wz_{0.40};

  double start_s_{0.0};
  double last_phase_switch_s_{0.0};

  std::string output_topic_{topics::AUTO_TEST_CMD};
  std::string enable_topic_{topics::AUTO_TEST_ENABLE};
  std::string state_topic_{topics::AUTO_TEST_STATE};
  std::string status_topic_{"/savo_control/velocity_test_pattern_status"};
  std::string safety_stop_topic_{topics::SAFETY_STOP};

  bool safety_stop_{false};
  bool safety_stop_seen_{false};
  double safety_stop_stamp_s_{0.0};

  CommandLimiter limiter_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_stop_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace savo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<savo_control::VelocityTestPatternNode>());
  rclcpp::shutdown();
  return 0;
}
