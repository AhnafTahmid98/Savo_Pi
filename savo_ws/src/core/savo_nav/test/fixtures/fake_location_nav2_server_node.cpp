// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

class FakeLocationNav2ServerNode final : public rclcpp::Node
{
public:
  FakeLocationNav2ServerNode()
  : Node("fake_location_nav2_server_node")
  {
    action_name_ = declare_parameter<std::string>(
      "action_name", "/navigate_to_pose");
    goal_topic_ = declare_parameter<std::string>(
      "goal_topic", "/savo_nav/test/last_location_goal");
    step_count_ = declare_parameter<int>("step_count", 10);
    step_delay_ms_ = declare_parameter<int>("step_delay_ms", 50);

    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(1).reliable().transient_local());

    server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name_,
      [](
        const rclcpp_action::GoalUUID &,
        const std::shared_ptr<const NavigateToPose::Goal>)
      {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](
        const std::shared_ptr<GoalHandle>)
      {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle)
      {
        std::thread{
          [this, goal_handle]()
          {
            execute(goal_handle);
          }}.detach();
      });
  }

private:
  void execute(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    goal_publisher_->publish(goal_handle->get_goal()->pose);

    for (int step = 0; step < step_count_; ++step) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<NavigateToPose::Result>();
        result->error_code = NavigateToPose::Result::NONE;
        result->error_msg = "fake_location_nav2_canceled";
        goal_handle->canceled(result);
        return;
      }

      auto feedback = std::make_shared<NavigateToPose::Feedback>();
      feedback->distance_remaining =
        static_cast<float>(step_count_ - step) * 0.1F;
      feedback->number_of_recoveries = 0;
      goal_handle->publish_feedback(feedback);
      std::this_thread::sleep_for(
        std::chrono::milliseconds(step_delay_ms_));
    }

    auto result = std::make_shared<NavigateToPose::Result>();
    result->error_code = NavigateToPose::Result::NONE;
    result->error_msg = "fake_location_nav2_succeeded";
    goal_handle->succeed(result);
  }

  std::string action_name_{};
  std::string goal_topic_{};
  int step_count_{10};
  int step_delay_ms_{50};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_publisher_{};
  rclcpp_action::Server<NavigateToPose>::SharedPtr server_{};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLocationNav2ServerNode>());
  rclcpp::shutdown();
  return 0;
}
