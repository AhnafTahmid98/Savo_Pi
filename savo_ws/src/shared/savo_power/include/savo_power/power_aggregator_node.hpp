#ifndef SAVO_POWER__POWER_AGGREGATOR_NODE_HPP_
#define SAVO_POWER__POWER_AGGREGATOR_NODE_HPP_

#include "savo_power/power_aggregator.hpp"
#include "savo_power/power_node_params.hpp"
#include "savo_power/power_state.hpp"
#include "savo_power/visibility_control.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

namespace savo_power
{

class SAVO_POWER_PUBLIC PowerAggregatorNode : public rclcpp::Node
{
public:
  explicit PowerAggregatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  PowerAggregatorNode(const PowerAggregatorNode &) = delete;
  PowerAggregatorNode & operator=(const PowerAggregatorNode &) = delete;

private:
  struct CachedSource
  {
    BatterySource source{BatterySource::UNKNOWN};
    bool seen{false};
    PowerState state{PowerState::UNKNOWN};
    rclcpp::Time last_seen{};
    std::string text{};
  };

  void load_params();

  void update_source(
    CachedSource & source,
    const std::string & text);

  PowerAggregatorInputs make_inputs() const;

  TimedPowerSourceInput make_source_input(
    const CachedSource & source) const;

  double source_age_s(
    const CachedSource & source) const;

  void publish_status();

  static PowerState state_from_text(
    const std::string & text);

  PowerAggregatorNodeParams params_{};
  PowerAggregator aggregator_{};

  CachedSource core_{};
  CachedSource edge_{};
  CachedSource base_{};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr core_subscription_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr edge_subscription_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr base_subscription_{};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dashboard_publisher_{};

  rclcpp::TimerBase::SharedPtr timer_{};
};

}  // namespace savo_power

#endif  // SAVO_POWER__POWER_AGGREGATOR_NODE_HPP_
