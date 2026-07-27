// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_nav/control_mode_client.hpp"
#include "savo_nav/control_recovery_guard.hpp"
#include "savo_nav/recovery_bridge.hpp"
#include "savo_nav/topic_names.hpp"

namespace
{

class ControlRecoveryGuardNode final
  : public rclcpp::Node
{
public:
  ControlRecoveryGuardNode()
  : Node("control_recovery_guard_node")
  {
    publish_hz_ =
      declare_parameter<double>(
      "publish_hz",
      10.0);

    control_timeout_seconds_ =
      declare_parameter<double>(
      "control_mode_timeout_seconds",
      1.5);

    recovery_timeout_seconds_ =
      declare_parameter<double>(
      "recovery_timeout_seconds",
      1.0);

    if (
      !std::isfinite(publish_hz_) ||
      publish_hz_ <= 0.0)
    {
      throw std::invalid_argument(
              "publish_hz must be positive");
    }

    if (
      !std::isfinite(control_timeout_seconds_) ||
      control_timeout_seconds_ <= 0.0)
    {
      throw std::invalid_argument(
              "control_mode_timeout_seconds "
              "must be positive");
    }

    if (
      !std::isfinite(recovery_timeout_seconds_) ||
      recovery_timeout_seconds_ <= 0.0)
    {
      throw std::invalid_argument(
              "recovery_timeout_seconds "
              "must be positive");
    }

    const std::string control_mode_state_topic =
      declare_parameter<std::string>(
      "control_mode_state_topic",
      std::string(
        savo_nav::topics::kControlModeState));

    const std::string control_mode_reason_topic =
      declare_parameter<std::string>(
      "control_mode_reason_topic",
      std::string(
        savo_nav::topics::kControlModeReason));

    const std::string control_status_topic =
      declare_parameter<std::string>(
      "control_status_topic",
      std::string(
        savo_nav::topics::kControlStatus));

    const std::string recovery_active_topic =
      declare_parameter<std::string>(
      "recovery_active_topic",
      std::string(
        savo_nav::topics::kRecoveryActive));

    const std::string recovery_state_topic =
      declare_parameter<std::string>(
      "recovery_state_topic",
      std::string(
        savo_nav::topics::kControlRecoveryState));

    const std::string recovery_status_topic =
      declare_parameter<std::string>(
      "recovery_status_topic",
      std::string(
        savo_nav::topics::kRecoveryStatus));

    const std::string navigation_allowed_topic =
      declare_parameter<std::string>(
      "navigation_allowed_topic",
      std::string(
        savo_nav::topics::kControlRecoveryAllowed));

    const std::string navigation_reason_topic =
      declare_parameter<std::string>(
      "navigation_reason_topic",
      std::string(
        savo_nav::topics::kControlRecoveryReason));

    const std::string navigation_status_topic =
      declare_parameter<std::string>(
      "navigation_status_topic",
      std::string(
        savo_nav::topics::kControlRecoveryStatus));

    auto retained_qos =
      rclcpp::QoS(rclcpp::KeepLast(1));

    retained_qos.reliable();
    retained_qos.transient_local();

    const auto event_qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).
      reliable();

    control_mode_subscription_ =
      create_subscription<std_msgs::msg::String>(
      control_mode_state_topic,
      retained_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);

        static_cast<void>(
          control_mode_.UpdateMode(
            message->data,
            now().seconds()));
      });

    control_reason_subscription_ =
      create_subscription<std_msgs::msg::String>(
      control_mode_reason_topic,
      event_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);

        control_mode_.UpdateReason(
          message->data);
      });

    control_status_subscription_ =
      create_subscription<std_msgs::msg::String>(
      control_status_topic,
      event_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);

        control_mode_.UpdateStatus(
          message->data);
      });

    recovery_active_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      recovery_active_topic,
      event_qos,
      [this](
        const std_msgs::msg::Bool::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);

        static_cast<void>(
          recovery_.UpdateActive(
            message->data,
            now().seconds()));
      });

    recovery_state_subscription_ =
      create_subscription<std_msgs::msg::String>(
      recovery_state_topic,
      event_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);

        recovery_.UpdateState(
          message->data);
      });

    recovery_status_subscription_ =
      create_subscription<std_msgs::msg::String>(
      recovery_status_topic,
      event_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);

        recovery_.UpdateStatus(
          message->data);
      });

    allowed_publisher_ =
      create_publisher<std_msgs::msg::Bool>(
      navigation_allowed_topic,
      retained_qos);

    reason_publisher_ =
      create_publisher<std_msgs::msg::String>(
      navigation_reason_topic,
      retained_qos);

    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      navigation_status_topic,
      retained_qos);

    timer_ =
      create_wall_timer(
      std::chrono::duration<double>(
        1.0 / publish_hz_),
      [this]()
      {
        PublishDecision();
      });

    PublishDecision();

    RCLCPP_INFO(
      get_logger(),
      "Read-only control/recovery guard started");
  }

private:
  void PublishDecision()
  {
    savo_nav::ControlRecoveryDecision decision;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      const double now_seconds =
        now().seconds();

      const auto control =
        control_mode_.Evaluate(
        now_seconds,
        control_timeout_seconds_);

      const auto recovery =
        recovery_.Evaluate(
        now_seconds,
        recovery_timeout_seconds_);

      decision =
        savo_nav::ControlRecoveryGuard::Evaluate(
        control,
        recovery);
    }

    std_msgs::msg::Bool allowed_message;
    allowed_message.data =
      decision.navigation_allowed;

    allowed_publisher_->publish(
      allowed_message);

    std_msgs::msg::String reason_message;
    reason_message.data = decision.reason;

    reason_publisher_->publish(
      reason_message);

    std_msgs::msg::String status_message;

    std::ostringstream stream;

    stream
      << "allowed="
      << (
      decision.navigation_allowed ?
      "true" :
      "false"
      )
      << ";cancel_active_goal="
      << (
      decision.cancel_active_goal ?
      "true" :
      "false"
      )
      << ";reason="
      << decision.reason
      << ";control_observed="
      << (
      decision.control.observed ?
      "true" :
      "false"
      )
      << ";control_fresh="
      << (
      decision.control.fresh ?
      "true" :
      "false"
      )
      << ";control_mode="
      << decision.control.mode_text
      << ";control_age_seconds="
      << decision.control.age_seconds
      << ";recovery_observed="
      << (
      decision.recovery.observed ?
      "true" :
      "false"
      )
      << ";recovery_fresh="
      << (
      decision.recovery.fresh ?
      "true" :
      "false"
      )
      << ";recovery_active="
      << (
      decision.recovery.active ?
      "true" :
      "false"
      )
      << ";recovery_age_seconds="
      << decision.recovery.age_seconds;

    status_message.data = stream.str();

    status_publisher_->publish(
      status_message);
  }

  std::mutex mutex_;

  double publish_hz_{10.0};

  double control_timeout_seconds_{1.5};
  double recovery_timeout_seconds_{1.0};

  savo_nav::ControlModeClient control_mode_;
  savo_nav::RecoveryBridge recovery_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    control_mode_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    control_reason_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    control_status_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    recovery_active_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    recovery_state_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    recovery_status_subscription_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    allowed_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    reason_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    status_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    rclcpp::spin(
      std::make_shared<
        ControlRecoveryGuardNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "control_recovery_guard_node"),
      "Fatal error: %s",
      exception.what());

    exit_code = 1;
  }

  rclcpp::shutdown();

  return exit_code;
}
