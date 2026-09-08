// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/nav2_startup_readiness.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

std::string JsonEscape(const std::string & value)
{
  std::string output;
  output.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '\\': output += "\\\\"; break;
      case '"': output += "\\\""; break;
      case '\n': output += "\\n"; break;
      case '\r': output += "\\r"; break;
      case '\t': output += "\\t"; break;
      default: output += character; break;
    }
  }
  return output;
}

class Nav2StartupReadinessNode final : public rclcpp::Node
{
public:
  Nav2StartupReadinessNode()
  : rclcpp::Node("nav2_startup_readiness_node")
  {
    const auto lifecycle_nodes = declare_parameter<std::vector<std::string>>(
      "required_lifecycle_nodes",
      {"controller_server", "planner_server", "behavior_server", "bt_navigator",
        "waypoint_follower"});
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/savo_nav/startup_readiness");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    const auto navigate_action = declare_parameter<std::string>(
      "navigate_to_pose_action", "/navigate_to_pose");
    const auto waypoints_action = declare_parameter<std::string>(
      "follow_waypoints_action", "/follow_waypoints");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 2.0);
    if (lifecycle_nodes.empty() || output_topic_.empty() || odom_frame_.empty() ||
      base_frame_.empty() || navigate_action.empty() || waypoints_action.empty() ||
      publish_rate_hz <= 0.0)
    {
      throw std::invalid_argument("invalid Nav2 startup-readiness configuration");
    }

    for (const auto & name : lifecycle_nodes) {
      LifecycleEndpoint endpoint;
      endpoint.name = name;
      endpoint.client = create_client<lifecycle_msgs::srv::GetState>(
        "/" + name + "/get_state");
      lifecycle_.push_back(std::move(endpoint));
    }
    navigate_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, navigate_action);
    waypoints_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
      this, waypoints_action);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    publisher_ = create_publisher<std_msgs::msg::String>(
      output_topic_, rclcpp::QoS(1).reliable().transient_local());
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() {Tick();});
    Tick();
  }

private:
  struct LifecycleEndpoint
  {
    std::string name;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client;
    bool active{false};
    bool request_pending{false};
  };

  void RequestLifecycleStates()
  {
    for (std::size_t index = 0; index < lifecycle_.size(); ++index) {
      auto & endpoint = lifecycle_[index];
      if (endpoint.request_pending || !endpoint.client->service_is_ready()) {
        endpoint.active = false;
        continue;
      }
      endpoint.request_pending = true;
      endpoint.client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
        [this, index](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
          auto & result = lifecycle_.at(index);
          result.request_pending = false;
          try {
            result.active = future.get()->current_state.id ==
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
          } catch (const std::exception &) {
            result.active = false;
          }
        });
    }
  }

  void Tick()
  {
    RequestLifecycleStates();
    savo_nav::Nav2StartupEvidence evidence;
    for (const auto & endpoint : lifecycle_) {
      if (!endpoint.active) {
        evidence.inactive_lifecycle_nodes.push_back(endpoint.name);
      }
    }
    evidence.navigate_to_pose_action_ready = navigate_client_->action_server_is_ready();
    evidence.follow_waypoints_action_ready = waypoints_client_->action_server_is_ready();
    evidence.odom_to_base_footprint_tf_ready = tf_buffer_->canTransform(
      odom_frame_, base_frame_, tf2::TimePointZero);
    const auto decision = savo_nav::EvaluateNav2Startup(evidence);

    std::ostringstream json;
    json << "{\"schema_name\":\"savo_nav_startup_readiness\""
         << ",\"schema_version\":1"
         << ",\"process_started\":" << (decision.process_started ? "true" : "false")
         << ",\"startup_ready\":" << (decision.startup_ready ? "true" : "false")
         << ",\"ready\":" << (decision.startup_ready ? "true" : "false")
         << ",\"quality\":\"" << savo_nav::ToString(decision.quality) << "\""
         << ",\"quality_reason\":\"" << JsonEscape(decision.reason) << "\""
         << ",\"reason\":\"" << JsonEscape(decision.reason) << "\""
         << ",\"map_required\":false"
         << ",\"odom_frame\":\"" << JsonEscape(odom_frame_) << "\""
         << ",\"base_frame\":\"" << JsonEscape(base_frame_) << "\"}";
    std_msgs::msg::String message;
    message.data = json.str();
    publisher_->publish(message);
  }

  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::vector<LifecycleEndpoint> lifecycle_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_client_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoints_client_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Nav2StartupReadinessNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("nav2_startup_readiness_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
