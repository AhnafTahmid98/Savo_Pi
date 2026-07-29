// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

class FakeFollowPathServerNode final : public rclcpp::Node
{
public:
  using FollowPath = nav2_msgs::action::FollowPath;

  using GoalHandle =
    rclcpp_action::ServerGoalHandle<FollowPath>;

  FakeFollowPathServerNode()
  : Node("fake_follow_path_server_node")
  {
    const std::string action_name =
      declare_parameter<std::string>(
      "action_name",
      "/test/follow_path");

    const std::string mode_topic =
      declare_parameter<std::string>(
      "mode_topic",
      "/test/follow_path_mode");

    auto mode_qos =
      rclcpp::QoS(rclcpp::KeepLast(1));

    mode_qos.reliable();
    mode_qos.transient_local();

    mode_subscription_ =
      create_subscription<std_msgs::msg::String>(
      mode_topic,
      mode_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        if (
          message != nullptr &&
          IsSupportedMode(message->data))
        {
          std::lock_guard<std::mutex> lock(mutex_);
          selected_mode_ = message->data;
        }
      });

    server_ =
      rclcpp_action::create_server<FollowPath>(
      this,
      action_name,
      [this](
        const rclcpp_action::GoalUUID & uuid,
        const std::shared_ptr<
          const FollowPath::Goal> goal)
      {
        return HandleGoal(uuid, goal);
      },
      [this](
        const std::shared_ptr<GoalHandle> goal_handle)
      {
        return HandleCancel(goal_handle);
      },
      [this](
        const std::shared_ptr<GoalHandle> goal_handle)
      {
        HandleAccepted(goal_handle);
      });

    RCLCPP_INFO(
      get_logger(),
      "Fake FollowPath server ready: %s",
      action_name.c_str());
  }

private:
  static bool IsSupportedMode(
    const std::string & mode)
  {
    return
      mode == "test_success" ||
      mode == "test_reject" ||
      mode == "test_hold" ||
      mode == "test_cancel_ack" ||
      mode == "test_execution_timeout" ||
      mode == "test_stale" ||
      mode == "test_late_success";
  }

  std::string SelectedMode() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return selected_mode_;
  }

  void SetActiveMode(
    const std::string & mode)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_mode_ = mode;
  }

  std::string ActiveMode() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_mode_;
  }

  rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<
      const FollowPath::Goal>)
  {
    const std::string mode = SelectedMode();

    if (!IsSupportedMode(mode)) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting unsupported fake mode: %s",
        mode.c_str());

      return rclcpp_action::GoalResponse::REJECT;
    }

    if (mode == "test_reject") {
      return rclcpp_action::GoalResponse::REJECT;
    }

    SetActiveMode(mode);

    return
      rclcpp_action::GoalResponse::
      ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<GoalHandle>)
  {
    if (ActiveMode() == "test_late_success") {
      RCLCPP_INFO(
        get_logger(),
        "Rejecting cancel for late-success fixture");

      return rclcpp_action::CancelResponse::REJECT;
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    const std::string mode = ActiveMode();

    std::thread(
      [this, goal_handle, mode]()
      {
        Execute(goal_handle, mode);
      }).detach();
  }

  static std::shared_ptr<FollowPath::Result>
  MakeResult(
    const std::string & error_message = {})
  {
    auto result =
      std::make_shared<FollowPath::Result>();

    result->error_code = 0;
    result->error_msg = error_message;

    return result;
  }

  static void PublishFeedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const float distance,
    const float speed)
  {
    auto feedback =
      std::make_shared<FollowPath::Feedback>();

    feedback->distance_to_goal = distance;
    feedback->speed = speed;

    goal_handle->publish_feedback(feedback);
  }

  void Execute(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const std::string & mode)
  {
    using namespace std::chrono_literals;

    if (mode == "test_success") {
      constexpr std::array<float, 3> distances{
        6.0F,
        3.0F,
        0.0F
      };

      for (const float distance : distances) {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(MakeResult());
          return;
        }

        PublishFeedback(
          goal_handle,
          distance,
          0.25F);

        std::this_thread::sleep_for(100ms);
      }

      goal_handle->succeed(MakeResult());
      return;
    }

    const auto started =
      std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(MakeResult());
        return;
      }

      const double elapsed_seconds =
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() -
        started).count();

      if (
        mode == "test_late_success" &&
        elapsed_seconds >= 1.5)
      {
        goal_handle->succeed(MakeResult());
        return;
      }

      if (elapsed_seconds >= 8.0) {
        goal_handle->abort(
          MakeResult("fake_backend_guard_timeout"));

        return;
      }

      if (mode != "test_stale") {
        PublishFeedback(
          goal_handle,
          5.0F,
          0.10F);
      }

      std::this_thread::sleep_for(100ms);
    }
  }

  mutable std::mutex mutex_;

  std::string selected_mode_{"test_success"};
  std::string active_mode_{"test_success"};

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    mode_subscription_;

  rclcpp_action::Server<FollowPath>::SharedPtr server_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<FakeFollowPathServerNode>());

  rclcpp::shutdown();
  return 0;
}
