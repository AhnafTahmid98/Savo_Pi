// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <memory>
#include <thread>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using NavigateToPose =
  nav2_msgs::action::NavigateToPose;

using GoalHandle =
  rclcpp_action::ServerGoalHandle<
  NavigateToPose>;

class FakeNav2ServerNode final : public rclcpp::Node
{
public:
  FakeNav2ServerNode()
  : Node("fake_nav2_server_node")
  {
    step_count_ =
      declare_parameter<int>(
      "step_count",
      30);

    step_delay_ms_ =
      declare_parameter<int>(
      "step_delay_ms",
      50);

    server_ =
      rclcpp_action::create_server<
      NavigateToPose>(
      this,
      "/navigate_to_pose",
      [](
        const rclcpp_action::GoalUUID &,
        const std::shared_ptr<
          const NavigateToPose::Goal>)
      {
        return
          rclcpp_action::GoalResponse::
          ACCEPT_AND_EXECUTE;
      },
      [](
        const std::shared_ptr<GoalHandle>)
      {
        return
          rclcpp_action::CancelResponse::
          ACCEPT;
      },
      [this](
        const std::shared_ptr<
          GoalHandle> goal_handle)
      {
        std::thread{
          [this, goal_handle]()
          {
            Execute(goal_handle);
          }}.detach();
      });

    RCLCPP_INFO(
      get_logger(),
      "Fake Nav2 NavigateToPose server started");
  }

private:
  void Execute(
    const std::shared_ptr<
      GoalHandle> & goal_handle)
  {
    for (int step = 0; step < step_count_; ++step) {
      if (goal_handle->is_canceling()) {
        auto result =
          std::make_shared<
          NavigateToPose::Result>();

        result->error_code =
          NavigateToPose::Result::NONE;

        result->error_msg =
          "fake_nav2_canceled";

        goal_handle->canceled(result);
        return;
      }

      auto feedback =
        std::make_shared<
        NavigateToPose::Feedback>();

      feedback->distance_remaining =
        static_cast<float>(
        step_count_ - step) * 0.1F;

      feedback->number_of_recoveries = 0;

      goal_handle->publish_feedback(feedback);

      std::this_thread::sleep_for(
        std::chrono::milliseconds(
          step_delay_ms_));
    }

    auto result =
      std::make_shared<
      NavigateToPose::Result>();

    result->error_code =
      NavigateToPose::Result::NONE;

    result->error_msg =
      "fake_nav2_succeeded";

    goal_handle->succeed(result);
  }

  int step_count_{30};
  int step_delay_ms_{50};

  rclcpp_action::Server<
    NavigateToPose>::SharedPtr server_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      FakeNav2ServerNode>());

  rclcpp::shutdown();

  return 0;
}
