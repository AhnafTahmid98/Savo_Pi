#ifndef SAVO_POWER__POWER_HEALTH_NODE_HPP_
#define SAVO_POWER__POWER_HEALTH_NODE_HPP_

#include "savo_power/power_health.hpp"
#include "savo_power/power_node_params.hpp"
#include "savo_power/power_state.hpp"
#include "savo_power/visibility_control.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace savo_power
{

class SAVO_POWER_PUBLIC PowerHealthNode : public rclcpp::Node
{
public:
  explicit PowerHealthNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  PowerHealthNode(const PowerHealthNode &) = delete;
  PowerHealthNode & operator=(const PowerHealthNode &) = delete;

private:
  void load_params();

  void update_status(
    const std::string & status_text);

  double status_age_s() const;

  PowerHealthInput make_health_input() const;

  void publish_health();

  PowerHealthNodeParams params_{};
  PowerHealth health_{};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_{};
  rclcpp::TimerBase::SharedPtr timer_{};

  bool status_seen_{false};
  rclcpp::Time last_status_seen_{};
  std::string last_status_text_{};
};

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_HEALTH_NODE_HPP_
