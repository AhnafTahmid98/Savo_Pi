#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace savo_mapping::test
{

class FakeSavoNavExplorationServerNode final
  : public rclcpp::Node
{
public:
  using NavigateToPose =
    nav2_msgs::action::NavigateToPose;

  using GoalHandle =
    rclcpp_action::ServerGoalHandle<
      NavigateToPose>;

  FakeSavoNavExplorationServerNode()
  : Node(
      "fake_savo_nav_exploration_server_node")
  {
    action_name_ =
      declare_parameter<std::string>(
      "action_name",
      "/savo_nav/exploration/navigate_to_pose");

    declare_parameter<std::string>(
      "mode",
      "success");

    declare_parameter<std::int64_t>(
      "feedback_steps",
      5);

    declare_parameter<std::int64_t>(
      "feedback_period_ms",
      200);

    declare_parameter<std::int64_t>(
      "goal_response_delay_ms",
      3000);

    declare_parameter<std::int64_t>(
      "abort_error_code",
      100);

    declare_parameter<std::string>(
      "abort_error_message",
      "fake_savo_nav_abort");

    if (action_name_.empty()) {
      throw std::invalid_argument(
              "action_name_empty");
    }

    action_server_ =
      rclcpp_action::create_server<
        NavigateToPose>(
        this,
        action_name_,
        std::bind(
          &FakeSavoNavExplorationServerNode::
          handle_goal,
          this,
          std::placeholders::_1,
          std::placeholders::_2),
        std::bind(
          &FakeSavoNavExplorationServerNode::
          handle_cancel,
          this,
          std::placeholders::_1),
        std::bind(
          &FakeSavoNavExplorationServerNode::
          handle_accepted,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "fake savo_nav exploration server ready: %s",
      action_name_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "test modes: success, reject, abort, delayed_accept");
  }

  ~FakeSavoNavExplorationServerNode() override
  {
    stopping_.store(true);

    std::lock_guard<std::mutex> lock{
      worker_mutex_};

    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<
      const NavigateToPose::Goal> goal)
  {
    const std::string mode =
      get_parameter("mode")
      .as_string();

    RCLCPP_INFO(
      get_logger(),
      "fake savo_nav received goal: "
      "mode=%s frame=%s x=%.3f y=%.3f",
      mode.c_str(),
      goal->pose.header.frame_id.c_str(),
      goal->pose.pose.position.x,
      goal->pose.pose.position.y);

    if (active_.load()) {
      RCLCPP_WARN(
        get_logger(),
        "rejecting concurrent test goal");

      return
        rclcpp_action::GoalResponse::REJECT;
    }

    if (mode == "reject") {
      RCLCPP_INFO(
        get_logger(),
        "rejecting goal by configured test mode");

      return
        rclcpp_action::GoalResponse::REJECT;
    }

    if (mode != "success" &&
        mode != "abort" &&
        mode != "delayed_accept")
    {
      RCLCPP_ERROR(
        get_logger(),
        "invalid fake server mode: %s",
        mode.c_str());

      return
        rclcpp_action::GoalResponse::REJECT;
    }

    if (mode == "delayed_accept") {
      const std::int64_t configured_delay =
        get_parameter(
          "goal_response_delay_ms")
        .as_int();

      const std::int64_t delay_ms =
        std::clamp<std::int64_t>(
          configured_delay,
          20,
          60000);

      RCLCPP_WARN(
        get_logger(),
        "delaying fake goal response by %ld ms",
        static_cast<long>(delay_ms));

      std::this_thread::sleep_for(
        std::chrono::milliseconds(
          delay_ms));
    }

    return
      rclcpp_action::GoalResponse::
      ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(
      get_logger(),
      "fake savo_nav accepted cancel request");

    return
      rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandle>
    goal_handle)
  {
    active_.store(true);

    const std::string mode =
      get_parameter("mode")
      .as_string();

    {
      std::lock_guard<std::mutex> lock{
        worker_mutex_};

      if (worker_.joinable()) {
        worker_.join();
      }

      worker_ =
        std::thread(
        &FakeSavoNavExplorationServerNode::
        execute_goal,
        this,
        goal_handle,
        mode);
    }
  }

  static void set_duration(
    builtin_interfaces::msg::Duration & target,
    const double seconds)
  {
    const double safe_seconds =
      std::max(0.0, seconds);

    const auto whole_seconds =
      static_cast<std::int32_t>(
      std::floor(safe_seconds));

    const double fractional =
      safe_seconds -
      static_cast<double>(
      whole_seconds);

    target.sec =
      whole_seconds;

    target.nanosec =
      static_cast<std::uint32_t>(
      fractional * 1.0e9);
  }

  void execute_goal(
    const std::shared_ptr<GoalHandle>
    goal_handle,
    const std::string mode)
  {
    const std::int64_t configured_steps =
      get_parameter("feedback_steps")
      .as_int();

    const std::int64_t configured_period =
      get_parameter("feedback_period_ms")
      .as_int();

    const std::int64_t steps =
      std::clamp<std::int64_t>(
      configured_steps,
      1,
      1000);

    const std::int64_t period_ms =
      std::clamp<std::int64_t>(
      configured_period,
      20,
      5000);

    const auto goal =
      goal_handle->get_goal();

    const double target_distance =
      std::max(
      1.0,
      std::hypot(
        goal->pose.pose.position.x,
        goal->pose.pose.position.y));

    const auto started =
      std::chrono::steady_clock::now();

    for (std::int64_t step = 0;
      step < steps;
      ++step)
    {
      if (stopping_.load() ||
          !rclcpp::ok())
      {
        active_.store(false);
        return;
      }

      if (goal_handle->is_canceling()) {
        auto result =
          std::make_shared<
          NavigateToPose::Result>();

        result->error_code = 0U;
        result->error_msg =
          "fake_savo_nav_canceled";

        goal_handle->canceled(result);
        active_.store(false);

        RCLCPP_INFO(
          get_logger(),
          "fake exploration goal canceled");

        return;
      }

      const double progress =
        static_cast<double>(
          step + 1) /
        static_cast<double>(
          steps);

      const auto now =
        std::chrono::steady_clock::now();

      const double elapsed =
        std::chrono::duration<double>(
          now - started)
        .count();

      const double remaining =
        static_cast<double>(
          steps - step - 1) *
        (
          static_cast<double>(
            period_ms) /
          1000.0
        );

      auto feedback =
        std::make_shared<
        NavigateToPose::Feedback>();

      feedback->current_pose =
        goal->pose;

      feedback->
      current_pose.pose.position.x =
        goal->pose.pose.position.x *
        progress;

      feedback->
      current_pose.pose.position.y =
        goal->pose.pose.position.y *
        progress;

      feedback->distance_remaining =
        static_cast<float>(
        target_distance *
        (1.0 - progress));

      feedback->number_of_recoveries =
        0;

      set_duration(
        feedback->navigation_time,
        elapsed);

      set_duration(
        feedback->
        estimated_time_remaining,
        remaining);

      goal_handle->publish_feedback(
        feedback);

      std::this_thread::sleep_for(
        std::chrono::milliseconds(
          period_ms));
    }

    if (goal_handle->is_canceling()) {
      auto result =
        std::make_shared<
        NavigateToPose::Result>();

      result->error_code = 0U;
      result->error_msg =
        "fake_savo_nav_canceled";

      goal_handle->canceled(result);
      active_.store(false);
      return;
    }

    auto result =
      std::make_shared<
      NavigateToPose::Result>();

    if (mode == "abort") {
      const auto configured_error =
        get_parameter(
          "abort_error_code")
        .as_int();

      result->error_code =
        static_cast<std::uint16_t>(
        std::clamp<std::int64_t>(
          configured_error,
          1,
          65535));

      result->error_msg =
        get_parameter(
          "abort_error_message")
        .as_string();

      goal_handle->abort(result);

      RCLCPP_INFO(
        get_logger(),
        "fake exploration goal aborted");
    } else {
      result->error_code = 0U;
      result->error_msg.clear();

      goal_handle->succeed(result);

      RCLCPP_INFO(
        get_logger(),
        "fake exploration goal succeeded");
    }

    active_.store(false);
  }

  std::string action_name_;

  std::atomic_bool active_{false};
  std::atomic_bool stopping_{false};

  std::mutex worker_mutex_;
  std::thread worker_;

  rclcpp_action::Server<
    NavigateToPose>::SharedPtr
    action_server_;
};

}  // namespace savo_mapping::test

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      savo_mapping::test::
      FakeSavoNavExplorationServerNode>());

  rclcpp::shutdown();
  return 0;
}
