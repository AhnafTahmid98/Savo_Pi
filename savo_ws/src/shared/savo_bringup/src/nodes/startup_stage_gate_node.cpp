// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

constexpr char kNodeName[] = "startup_stage_gate_node";

class StartupStageGateNode final : public rclcpp::Node
{
public:
  StartupStageGateNode()
  : rclcpp::Node(kNodeName)
  {
    target_stage_ = declare_parameter<std::string>("target_stage", "");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/savo_bringup/core/startup_status");
    if (target_stage_.empty() || status_topic_.empty() || status_topic_.front() != '/') {
      throw std::invalid_argument("target_stage and absolute status_topic are required");
    }
    subscription_ = create_subscription<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {OnStatus(message->data);});
    RCLCPP_INFO(get_logger(), "Waiting for startup stage '%s'", target_stage_.c_str());
  }

  [[nodiscard]] int exit_code() const noexcept
  {
    return exit_code_;
  }

private:
  bool Completed(const std::string & payload) const
  {
    constexpr char marker[] = "\"completed_stages\":[";
    const auto begin = payload.find(marker);
    if (begin == std::string::npos) {
      return false;
    }
    const auto end = payload.find(']', begin);
    if (end == std::string::npos) {
      return false;
    }
    const std::string target = "\"" + target_stage_ + "\"";
    const auto found = payload.find(target, begin);
    return found != std::string::npos && found < end;
  }

  void OnStatus(const std::string & payload)
  {
    const std::string current = "\"current_stage\":\"" + target_stage_ + "\"";
    if (payload.find(current) != std::string::npos &&
      payload.find("\"state\":\"FAILED\"") != std::string::npos)
    {
      exit_code_ = 2;
      RCLCPP_ERROR(
        get_logger(), "Startup stage '%s' failed: %s", target_stage_.c_str(), payload.c_str());
      rclcpp::shutdown();
      return;
    }
    if (Completed(payload)) {
      exit_code_ = 0;
      RCLCPP_INFO(get_logger(), "Startup stage '%s' is ready", target_stage_.c_str());
      rclcpp::shutdown();
    }
  }

  std::string target_stage_;
  std::string status_topic_;
  // Only an observed completed-stage snapshot may release the next launch
  // group. An external shutdown or unexpected executor exit must fail closed.
  int exit_code_{3};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto node = std::make_shared<StartupStageGateNode>();
    rclcpp::spin(node);
    return node->exit_code();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger(kNodeName), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
}
