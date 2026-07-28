// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <thread>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>

#include "savo_nav/action_names.hpp"

namespace
{

class FakeGuardedGatewayServerNode final
  : public rclcpp::Node
{
public:
  using NavigateToPose =
    nav2_msgs::action::NavigateToPose;

  using GoalHandle =
    rclcpp_action::ServerGoalHandle<NavigateToPose>;

  FakeGuardedGatewayServerNode()
  : Node("fake_guarded_gateway_server_node")
  {
    auto retained_qos =
      rclcpp::QoS(rclcpp::KeepLast(1));

    retained_qos.reliable();
    retained_qos.transient_local();

    cancel_count_publisher_ =
      create_publisher<std_msgs::msg::Int32>(
      "/test/internal_gateway/cancel_count",
      retained_qos);

    navigation_server_ = CreateServer(
      std::string(
        savo_nav::actions::kInternalNavigationNavigateToPose));

    exploration_server_ = CreateServer(
      std::string(
        savo_nav::actions::kInternalExplorationNavigateToPose));

    PublishCancelCount();
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr
  CreateServer(const std::string & action_name)
  {
    return rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name,
      [](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToPose::Goal>)
      {
        return rclcpp_action::GoalResponse::
               ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>)
      {
        ++cancel_count_;
        PublishCancelCount();

        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle)
      {
        std::thread(
          [this, handle]()
          {
            Execute(handle);
          }).detach();
      });
  }

  void Execute(const std::shared_ptr<GoalHandle> handle)
  {
    const double x =
      handle->get_goal()->pose.pose.position.x;

    const bool hold_until_cancel =
      std::abs(x - 2.0) < 0.01;

    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::seconds(8);

    while (rclcpp::ok()) {
      if (handle->is_canceling()) {
        auto result =
          std::make_shared<NavigateToPose::Result>();

        result->error_code = 0;
        result->error_msg = "internal_goal_canceled";

        handle->canceled(result);
        return;
      }

      if (!hold_until_cancel) {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(200));

        auto result =
          std::make_shared<NavigateToPose::Result>();

        result->error_code = 0;
        result->error_msg.clear();

        handle->succeed(result);
        return;
      }

      if (std::chrono::steady_clock::now() > deadline) {
        auto result =
          std::make_shared<NavigateToPose::Result>();

        result->error_code = 9001;
        result->error_msg = "fake_internal_timeout";

        handle->abort(result);
        return;
      }

      std::this_thread::sleep_for(
        std::chrono::milliseconds(20));
    }
  }

  void PublishCancelCount()
  {
    std_msgs::msg::Int32 message;
    message.data = cancel_count_;
    cancel_count_publisher_->publish(message);
  }

  std::int32_t cancel_count_{0};

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
    cancel_count_publisher_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr
    navigation_server_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr
    exploration_server_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<FakeGuardedGatewayServerNode>());
  rclcpp::shutdown();
  return 0;
}
