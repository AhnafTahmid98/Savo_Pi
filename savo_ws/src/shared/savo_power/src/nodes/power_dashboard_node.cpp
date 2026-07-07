#include "savo_power/constants.hpp"
#include "savo_power/power_node_params.hpp"
#include "savo_power/topic_names.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <sstream>
#include <string>

namespace savo_power
{
namespace
{

double clamp_rate_hz(double value)
{
  return std::clamp(value, 0.1, 10.0);
}

struct TextSource
{
  bool seen{false};
  rclcpp::Time last_seen{};
  std::string text{};
};

}  // namespace

class PowerDashboardNode final : public rclcpp::Node
{
public:
  PowerDashboardNode()
  : rclcpp::Node(constants::kPowerDashboardNodeName)
  {
    load_params();

    publisher_ = create_publisher<std_msgs::msg::String>(
      topics::kDashboard,
      rclcpp::QoS(10).reliable());

    dashboard_subscription_ = create_subscription<std_msgs::msg::String>(
      topics::kDashboardText,
      rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        update_source(dashboard_text_, msg->data);
      });

    health_subscription_ = create_subscription<std_msgs::msg::String>(
      topics::kHealth,
      rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        update_source(health_text_, msg->data);
      });

    const double rate_hz = clamp_rate_hz(params_.publish_rate_hz);
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {
        publish_dashboard();
      });

    RCLCPP_INFO(
      get_logger(),
      "Started %s subscribing [%s, %s] publishing %s",
      constants::kPowerDashboardNodeName,
      topics::kDashboardText,
      topics::kHealth,
      topics::kDashboard);
  }

private:
  void load_params()
  {
    params_ = PowerDashboardNodeParams{};

    params_.publish_rate_hz = declare_parameter<double>(
      params::kPublishRateHz,
      params_.publish_rate_hz);

    params_.stale_timeout_s = declare_parameter<double>(
      params::kStaleTimeoutS,
      params_.stale_timeout_s);
  }

  void update_source(TextSource & source, const std::string & text)
  {
    source.seen = true;
    source.last_seen = now();
    source.text = text;
  }

  bool source_is_stale(const TextSource & source) const
  {
    if (!source.seen) {
      return true;
    }

    return (now() - source.last_seen).seconds() > params_.stale_timeout_s;
  }

  std::string source_age_text(const TextSource & source) const
  {
    if (!source.seen) {
      return "never";
    }

    const double age_s = (now() - source.last_seen).seconds();

    std::ostringstream out;
    out << static_cast<int>(age_s) << "s";
    return out.str();
  }

  std::string make_dashboard_output() const
  {
    std::ostringstream out;

    out << "Robot Savo Power Dashboard\n";
    out << "==========================\n";

    out << "Health: ";

    if (!health_text_.seen) {
      out << "missing\n";
    } else if (source_is_stale(health_text_)) {
      out << "stale, age=" << source_age_text(health_text_) << "\n";
    } else {
      out << health_text_.text << "\n";
    }

    out << "\n";

    if (!dashboard_text_.seen) {
      out << "Power status: missing\n";
    } else if (source_is_stale(dashboard_text_)) {
      out << "Power status: stale, age="
          << source_age_text(dashboard_text_)
          << "\n";
    } else {
      out << dashboard_text_.text;
      if (!dashboard_text_.text.empty() &&
        dashboard_text_.text.back() != '\n')
      {
        out << "\n";
      }
    }

    return out.str();
  }

  void publish_dashboard()
  {
    std_msgs::msg::String message;
    message.data = make_dashboard_output();

    publisher_->publish(message);

    if (source_is_stale(health_text_) || source_is_stale(dashboard_text_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Power dashboard has stale or missing input");
    }
  }

  PowerDashboardNodeParams params_{};

  TextSource dashboard_text_{};
  TextSource health_text_{};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dashboard_subscription_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_subscription_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

}  // namespace savo_power

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<savo_power::PowerDashboardNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
