// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_nav/nav2_startup_readiness.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
    const double request_timeout_s = declare_parameter<double>("lifecycle_request_timeout_s", 1.0);
    const double confirmation_freshness_s = declare_parameter<double>(
      "lifecycle_confirmation_freshness_s", 2.0);
    if (!std::isfinite(request_timeout_s) || request_timeout_s <= 0.0 ||
      !std::isfinite(confirmation_freshness_s) || confirmation_freshness_s <= 0.0)
    {
      throw std::invalid_argument("invalid lifecycle poll timeouts");
    }
    request_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(request_timeout_s));
    confirmation_freshness_ =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(confirmation_freshness_s));
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
    savo_nav::LifecycleObservation observation;
    bool service_available{false};
    std::int64_t request_id{-1};
    std::uint64_t generation{0};
  };

  void RequestLifecycleStates()
  {
    const auto now = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < lifecycle_.size(); ++index) {
      auto & endpoint = lifecycle_[index];
      endpoint.service_available = endpoint.client->service_is_ready();
      if (endpoint.observation.RequestTimedOut(now, request_timeout_)) {
        if (endpoint.request_id >= 0) {
          endpoint.client->remove_pending_request(endpoint.request_id);
        }
        endpoint.request_id = -1;
        endpoint.observation.RequestCancelled();
        ++endpoint.generation;  // Ignore any already-queued late callback.
      }
      if (endpoint.observation.request_pending() || !endpoint.service_available) {
        continue;
      }
      endpoint.observation.RequestStarted(now);
      const auto generation = ++endpoint.generation;
      try {
        const auto request = endpoint.client->async_send_request(
          std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
          [this, index, generation](
            rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
            auto & result = lifecycle_.at(index);
            if (generation != result.generation ||
            !result.observation.request_pending())
            {
              return;
            }
            result.request_id = -1;
            try {
              const bool active = future.get()->current_state.id ==
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
              result.observation.ResponseReceived(active, std::chrono::steady_clock::now());
            } catch (const std::exception &) {
              result.observation.Invalidate();
            }
          });
        endpoint.request_id = request.request_id;
      } catch (const std::exception & error) {
        endpoint.observation.Invalidate();
        endpoint.request_id = -1;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Lifecycle state request failed for %s: %s", endpoint.name.c_str(), error.what());
      }
    }
  }

  void Tick()
  {
    RequestLifecycleStates();
    savo_nav::Nav2StartupEvidence evidence;
    const auto now = std::chrono::steady_clock::now();
    for (const auto & endpoint : lifecycle_) {
      if (!endpoint.service_available) {
        evidence.unconfirmed_lifecycle_nodes.push_back(endpoint.name);
        continue;
      }
      switch (endpoint.observation.StateAt(now, confirmation_freshness_)) {
        case savo_nav::LifecycleObservationState::kActive:
          break;
        case savo_nav::LifecycleObservationState::kInactive:
          evidence.inactive_lifecycle_nodes.push_back(endpoint.name);
          break;
        case savo_nav::LifecycleObservationState::kUnknown:
        case savo_nav::LifecycleObservationState::kStale:
          evidence.unconfirmed_lifecycle_nodes.push_back(endpoint.name);
          break;
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
         << ",\"lifecycle_confirmed_active\":" <<
      (evidence.inactive_lifecycle_nodes.empty() &&
    evidence.unconfirmed_lifecycle_nodes.empty() ? "true" : "false")
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
  std::chrono::steady_clock::duration request_timeout_{};
  std::chrono::steady_clock::duration confirmation_freshness_{};
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
