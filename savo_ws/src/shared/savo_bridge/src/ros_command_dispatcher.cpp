// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/ros_command_dispatcher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <variant>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace savo_bridge
{

namespace
{

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

[[nodiscard]] std::string normalize_upper(
  std::string value)
{
  value.erase(
    value.begin(),
    std::find_if(
      value.begin(),
      value.end(),
      [](const unsigned char character)
      {
        return std::isspace(character) == 0;
      }));

  value.erase(
    std::find_if(
      value.rbegin(),
      value.rend(),
      [](const unsigned char character)
      {
        return std::isspace(character) == 0;
      }).base(),
    value.end());

  std::transform(
    value.begin(),
    value.end(),
    value.begin(),
    [](const unsigned char character)
    {
      return static_cast<char>(
        std::toupper(character));
    });

  return value;
}

[[nodiscard]] bool finite_twist(
  const geometry_msgs::msg::Twist & message) noexcept
{
  return
    std::isfinite(message.linear.x) &&
    std::isfinite(message.linear.y) &&
    std::isfinite(message.linear.z) &&
    std::isfinite(message.angular.x) &&
    std::isfinite(message.angular.y) &&
    std::isfinite(message.angular.z);
}

[[nodiscard]] bool twist_is_zero(
  const geometry_msgs::msg::Twist & message,
  const double epsilon) noexcept
{
  if (!finite_twist(message)) {
    return false;
  }

  const double limit = std::abs(epsilon);

  return
    std::abs(message.linear.x) <= limit &&
    std::abs(message.linear.y) <= limit &&
    std::abs(message.linear.z) <= limit &&
    std::abs(message.angular.x) <= limit &&
    std::abs(message.angular.y) <= limit &&
    std::abs(message.angular.z) <= limit;
}

[[nodiscard]] bool time_point_present(
  const TimePoint value) noexcept
{
  return value != TimePoint{};
}

[[nodiscard]] bool observation_fresh(
  const TimePoint observed_at,
  const TimePoint now,
  const std::int64_t timeout_ms) noexcept
{
  if (
    !time_point_present(observed_at) ||
    timeout_ms <= 0 ||
    now < observed_at)
  {
    return false;
  }

  const auto age =
    std::chrono::duration_cast<
    std::chrono::milliseconds>(
    now - observed_at);

  return age.count() <= timeout_ms;
}

[[nodiscard]] std::int64_t observation_age_ms(
  const TimePoint observed_at,
  const TimePoint now) noexcept
{
  if (
    !time_point_present(observed_at) ||
    now < observed_at)
  {
    return -1;
  }

  const auto age =
    std::chrono::duration_cast<
    std::chrono::milliseconds>(
    now - observed_at);

  return age.count();
}

[[nodiscard]] bool valid_topic(
  const std::string & topic) noexcept
{
  return
    !topic.empty() &&
    topic.front() == '/' &&
    topic.find('\0') == std::string::npos;
}

[[nodiscard]] bool valid_timeout(
  const std::int64_t timeout_ms) noexcept
{
  return
    timeout_ms > 0 &&
    timeout_ms <=
    std::numeric_limits<std::int32_t>::max();
}

[[nodiscard]] CommandDispatchResult rejection(
  std::string reason,
  const bool dispatch_attempted = true,
  const std::size_t ros_publications = 0U)
{
  CommandDispatchResult result;

  result.accepted = false;
  result.state = "rejected";
  result.reason = std::move(reason);
  result.dispatch_attempted = dispatch_attempted;
  result.ros_publications = ros_publications;

  return result;
}

[[nodiscard]] CommandDispatchResult acceptance(
  std::string reason,
  const std::size_t ros_publications)
{
  CommandDispatchResult result;

  result.accepted = true;
  result.state = "accepted";
  result.reason = std::move(reason);
  result.dispatch_attempted = true;
  result.ros_publications = ros_publications;

  return result;
}

}  // namespace

class RosCommandDispatcher::Impl
{
public:
  Impl(
    rclcpp::Node & node,
    RosCommandDispatcherConfig config)
  : node_(node),
    config_(std::move(config))
  {
    validate_config();

    auto latched_state_qos =
      rclcpp::QoS(rclcpp::KeepLast(1));

    latched_state_qos.reliable();
    latched_state_qos.transient_local();

    auto reliable_qos =
      rclcpp::QoS(rclcpp::KeepLast(10));

    reliable_qos.reliable();

    mode_command_publisher_ =
      node_.create_publisher<std_msgs::msg::String>(
      config_.mode_command_topic,
      reliable_qos);

    external_stop_publisher_ =
      node_.create_publisher<std_msgs::msg::Bool>(
      config_.external_stop_topic,
      reliable_qos);

    manual_velocity_publisher_ =
      node_.create_publisher<geometry_msgs::msg::Twist>(
      config_.manual_velocity_topic,
      reliable_qos);

    mode_state_subscription_ =
      node_.create_subscription<std_msgs::msg::String>(
      config_.mode_state_topic,
      latched_state_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        const TimePoint now = SteadyClock::now();

        {
          std::lock_guard<std::mutex> lock(mutex_);

          mode_state_observed_ = true;
          mode_state_ = normalize_upper(message->data);
          mode_state_observed_at_ = now;
        }

        observation_changed_.notify_all();
      });

    external_stop_subscription_ =
      node_.create_subscription<std_msgs::msg::Bool>(
      config_.external_stop_topic,
      reliable_qos,
      [this](
        const std_msgs::msg::Bool::SharedPtr message)
      {
        const TimePoint now = SteadyClock::now();

        {
          std::lock_guard<std::mutex> lock(mutex_);

          external_stop_observed_ = true;
          external_stop_active_ = message->data;
          external_stop_observed_at_ = now;
        }

        observation_changed_.notify_all();
      });

    safety_stop_subscription_ =
      node_.create_subscription<std_msgs::msg::Bool>(
      config_.safety_stop_topic,
      reliable_qos,
      [this](
        const std_msgs::msg::Bool::SharedPtr message)
      {
        const TimePoint now = SteadyClock::now();

        {
          std::lock_guard<std::mutex> lock(mutex_);

          safety_stop_observed_ = true;
          safety_stop_active_ = message->data;
          safety_stop_observed_at_ = now;
        }

        observation_changed_.notify_all();
      });

    safe_velocity_subscription_ =
      node_.create_subscription<geometry_msgs::msg::Twist>(
      config_.safe_velocity_topic,
      reliable_qos,
      [this](
        const geometry_msgs::msg::Twist::SharedPtr message)
      {
        const TimePoint now = SteadyClock::now();

        {
          std::lock_guard<std::mutex> lock(mutex_);

          safe_velocity_observed_ = true;

          safe_velocity_zero_ = twist_is_zero(
            *message,
            config_.safe_zero_epsilon);

          safe_velocity_observed_at_ = now;
        }

        observation_changed_.notify_all();
      });

    navigation_readiness_subscription_ =
      node_.create_subscription<std_msgs::msg::String>(
      config_.navigation_readiness_topic,
      latched_state_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        const TimePoint now = SteadyClock::now();

        {
          std::lock_guard<std::mutex> lock(mutex_);

          navigation_readiness_observed_ = true;

          navigation_readiness_ =
            normalize_upper(message->data);

          navigation_readiness_observed_at_ = now;
        }

        observation_changed_.notify_all();
      });

    {
      std::lock_guard<std::mutex> lock(mutex_);

      last_reason_ =
        "ros_command_dispatcher_stop_foundation_ready";
    }

    RCLCPP_INFO(
      node_.get_logger(),
      "ROS command dispatcher STOP foundation created; "
      "live teleop and navigation remain fail-closed");
  }

  ~Impl()
  {
    shutdown();
  }

  [[nodiscard]] CommandDispatchResult dispatch(
    const ValidatedCommand & command)
  {
    if (command.command_type == CommandType::Stop) {
      return dispatch_stop(command);
    }

    if (
      command.command_type ==
      CommandType::TeleopNudge)
    {
      return dispatch_teleop(command);
    }

    if (
      command.command_type ==
      CommandType::CancelAction)
    {
      return dispatch_cancel_action(command);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (shutdown_requested_) {
        ++rejected_command_count_;

        last_reason_ =
          "ros_command_dispatcher_shutdown";

        return rejection(
          "ros_command_dispatcher_shutdown");
      }

      ++rejected_command_count_;

      last_reason_ =
        "bridge_command_handler_not_enabled";
    }

    return rejection(
      "bridge_command_handler_not_enabled",
      false,
      0U);
  }

  [[nodiscard]] RosCommandDispatcherSnapshot snapshot() const
  {
    const TimePoint now = SteadyClock::now();

    std::lock_guard<std::mutex> lock(mutex_);

    RosCommandDispatcherSnapshot result;

    result.shutdown_requested = shutdown_requested_;

    result.command_active = command_active_;
    result.teleop_active = teleop_active_;

    result.teleop_cancel_requested =
      teleop_cancel_requested_;

    result.navigation_goal_active =
      navigation_goal_active_;

    result.navigation_cancel_requested =
      navigation_cancel_requested_;

    result.active_command_id = active_command_id_;
    result.active_command_type = active_command_type_;

    result.last_terminal_command_id =
      last_terminal_command_id_;

    result.mode_state_observed =
      mode_state_observed_;

    result.mode_state = mode_state_;

    result.mode_state_age_ms =
      observation_age_ms(
      mode_state_observed_at_,
      now);

    result.external_stop_observed =
      external_stop_observed_;

    result.external_stop_active =
      external_stop_active_;

    result.external_stop_age_ms =
      observation_age_ms(
      external_stop_observed_at_,
      now);

    result.safety_stop_observed =
      safety_stop_observed_;

    result.safety_stop_active =
      safety_stop_active_;

    result.safety_stop_age_ms =
      observation_age_ms(
      safety_stop_observed_at_,
      now);

    result.safe_velocity_observed =
      safe_velocity_observed_;

    result.safe_velocity_zero =
      safe_velocity_zero_;

    result.safe_velocity_age_ms =
      observation_age_ms(
      safe_velocity_observed_at_,
      now);

    result.navigation_readiness_observed =
      navigation_readiness_observed_;

    result.navigation_readiness =
      navigation_readiness_;

    result.navigation_readiness_age_ms =
      observation_age_ms(
      navigation_readiness_observed_at_,
      now);

    result.accepted_command_count =
      accepted_command_count_;

    result.rejected_command_count =
      rejected_command_count_;

    result.ros_publication_count =
      ros_publication_count_;

    result.last_reason = last_reason_;

    return result;
  }

  void shutdown() noexcept
  {
    bool already_shutdown = false;
    std::thread teleop_to_join;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      already_shutdown = shutdown_requested_;

      shutdown_requested_ = true;
      teleop_cancel_requested_ = true;
      navigation_cancel_requested_ = true;

      if (teleop_thread_.joinable()) {
        teleop_to_join =
          std::move(teleop_thread_);
      }

      last_reason_ =
        "ros_command_dispatcher_shutdown";
    }

    observation_changed_.notify_all();

    if (teleop_to_join.joinable()) {
      teleop_to_join.join();
    }

    if (already_shutdown) {
      return;
    }

    try {
      std::size_t ignored_publications = 0U;

      publish_zero_manual_velocity(
        ignored_publications);

      publish_external_stop(
        true,
        ignored_publications);

      publish_mode(
        config_.stop_mode,
        ignored_publications);
    } catch (...) {
      // Destructors and shutdown paths must not throw.
    }
  }

private:
  void validate_config()
  {
    const std::string topics[] = {
      config_.mode_command_topic,
      config_.mode_state_topic,
      config_.external_stop_topic,
      config_.safety_stop_topic,
      config_.manual_velocity_topic,
      config_.safe_velocity_topic,
      config_.navigation_readiness_topic,
    };

    for (const std::string & topic : topics) {
      if (!valid_topic(topic)) {
        throw std::invalid_argument(
                "ros_command_dispatcher_topic_invalid");
      }
    }

    if (
      config_.manual_velocity_topic ==
      config_.safe_velocity_topic)
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_manual_safe_topic_collision");
    }

    if (
      config_.manual_velocity_topic == "/cmd_vel" ||
      config_.manual_velocity_topic == "/cmd_vel_safe" ||
      config_.manual_velocity_topic == "/cmd_vel_nav" ||
      config_.manual_velocity_topic == "/cmd_vel_recovery")
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_manual_topic_forbidden");
    }

    if (
      normalize_upper(config_.stop_mode) != "STOP" ||
      normalize_upper(config_.manual_mode) != "MANUAL" ||
      normalize_upper(config_.navigation_mode) != "NAV")
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_mode_contract_invalid");
    }

    if (
      !valid_timeout(config_.observed_state_timeout_ms) ||
      !valid_timeout(config_.mode_transition_timeout_ms) ||
      !valid_timeout(config_.stop_confirmation_timeout_ms))
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_timeout_invalid");
    }

    if (
      !std::isfinite(config_.safe_zero_epsilon) ||
      config_.safe_zero_epsilon < 0.0)
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_zero_epsilon_invalid");
    }

    if (config_.final_zero_publication_count == 0U) {
      throw std::invalid_argument(
              "ros_command_dispatcher_zero_count_invalid");
    }

    if (
      !std::isfinite(
        config_.maximum_linear_speed_mps) ||
      config_.maximum_linear_speed_mps <= 0.0 ||
      config_.maximum_linear_speed_mps >
      MAX_TELEOP_LINEAR_SPEED_MPS)
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_linear_limit_invalid");
    }

    if (
      !std::isfinite(
        config_.maximum_angular_speed_radps) ||
      config_.maximum_angular_speed_radps <= 0.0 ||
      config_.maximum_angular_speed_radps >
      MAX_TELEOP_ANGULAR_SPEED_RADPS)
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_angular_limit_invalid");
    }

    if (
      config_.maximum_teleop_duration_ms <= 0 ||
      config_.maximum_teleop_duration_ms >
      MAX_TELEOP_DURATION_MS ||
      config_.teleop_publish_period_ms <= 0 ||
      config_.teleop_publish_period_ms >
      config_.maximum_teleop_duration_ms)
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_teleop_timing_invalid");
    }

    if (
      !valid_timeout(
        config_.teleop_cancel_timeout_ms))
    {
      throw std::invalid_argument(
              "ros_command_dispatcher_teleop_cancel_timeout_invalid");
    }
  }

  [[nodiscard]] bool teleop_authorized(
    const ValidatedCommand & command) const
  {
    if (
      command.priority ==
      CommandPriority::Emergency)
    {
      return false;
    }

    return
      command.origin_agent.has_value() &&
      command.origin_agent.value() ==
      "teleop_agent";
  }

  [[nodiscard]] bool validate_teleop_payload(
    const TeleopNudgeCommandPayload & payload,
    std::string & reason) const
  {
    if (
      !std::isfinite(payload.linear_x_mps) ||
      !std::isfinite(payload.linear_y_mps) ||
      !std::isfinite(payload.angular_z_radps))
    {
      reason =
        "bridge_teleop_payload_non_finite";

      return false;
    }

    if (
      std::abs(payload.linear_x_mps) >
      config_.maximum_linear_speed_mps ||
      std::abs(payload.linear_y_mps) >
      config_.maximum_linear_speed_mps ||
      std::abs(payload.angular_z_radps) >
      config_.maximum_angular_speed_radps)
    {
      reason =
        "bridge_teleop_speed_limit_exceeded";

      return false;
    }

    if (
      payload.duration_ms <= 0 ||
      payload.duration_ms >
      config_.maximum_teleop_duration_ms)
    {
      reason =
        "bridge_teleop_duration_invalid";

      return false;
    }

    constexpr double axis_epsilon = 1.0e-9;

    const bool linear_x_active =
      std::abs(payload.linear_x_mps) >
      axis_epsilon;

    const bool linear_y_active =
      std::abs(payload.linear_y_mps) >
      axis_epsilon;

    const bool angular_z_active =
      std::abs(payload.angular_z_radps) >
      axis_epsilon;

    const int active_axis_count =
      static_cast<int>(linear_x_active) +
      static_cast<int>(linear_y_active) +
      static_cast<int>(angular_z_active);

    if (active_axis_count != 1) {
      reason =
        "bridge_teleop_requires_single_axis";

      return false;
    }

    const bool direction_matches =
      (
        payload.direction == "forward" &&
        payload.linear_x_mps > 0.0
      ) ||
      (
        payload.direction == "backward" &&
        payload.linear_x_mps < 0.0
      ) ||
      (
        payload.direction == "strafe_left" &&
        payload.linear_y_mps > 0.0
      ) ||
      (
        payload.direction == "strafe_right" &&
        payload.linear_y_mps < 0.0
      ) ||
      (
        payload.direction == "turn_left" &&
        payload.angular_z_radps > 0.0
      ) ||
      (
        payload.direction == "turn_right" &&
        payload.angular_z_radps < 0.0
      );

    if (!direction_matches) {
      reason =
        "bridge_teleop_direction_mismatch";

      return false;
    }

    reason.clear();
    return true;
  }

  void join_finished_teleop_thread()
  {
    std::thread completed;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        teleop_thread_.joinable() &&
        !teleop_active_)
      {
        completed =
          std::move(teleop_thread_);
      }
    }

    if (completed.joinable()) {
      completed.join();
    }
  }

  [[nodiscard]] bool runtime_allows_teleop_locked(
    const TimePoint now) const
  {
    return
      safety_stop_observed_ &&
      observation_fresh(
        safety_stop_observed_at_,
        now,
        config_.observed_state_timeout_ms) &&
      !safety_stop_active_ &&
      external_stop_observed_ &&
      observation_fresh(
        external_stop_observed_at_,
        now,
        config_.observed_state_timeout_ms) &&
      !external_stop_active_ &&
      mode_state_observed_ &&
      observation_fresh(
        mode_state_observed_at_,
        now,
        config_.observed_state_timeout_ms) &&
      mode_state_ ==
      normalize_upper(config_.manual_mode) &&
      safe_velocity_observed_ &&
      observation_fresh(
        safe_velocity_observed_at_,
        now,
        config_.observed_state_timeout_ms);
  }

  [[nodiscard]] CommandDispatchResult dispatch_teleop(
    const ValidatedCommand & command)
  {
    join_finished_teleop_thread();

    if (!teleop_authorized(command)) {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_authority_rejected";
      }

      return rejection(
        "bridge_teleop_authority_rejected",
        false,
        0U);
    }

    const auto * payload =
      std::get_if<TeleopNudgeCommandPayload>(
      &command.payload);

    if (payload == nullptr) {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_payload_type_mismatch";
      }

      return rejection(
        "bridge_teleop_payload_type_mismatch",
        false,
        0U);
    }

    std::string validation_reason;

    if (
      !validate_teleop_payload(
        *payload,
        validation_reason))
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        ++rejected_command_count_;
        last_reason_ = validation_reason;
      }

      return rejection(
        validation_reason,
        false,
        0U);
    }

    const TimePoint admission_checked_at =
      SteadyClock::now();

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (shutdown_requested_) {
        ++rejected_command_count_;

        last_reason_ =
          "ros_command_dispatcher_shutdown";

        return rejection(
          "ros_command_dispatcher_shutdown",
          false,
          0U);
      }

      if (command_active_) {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_command_already_active";

        return rejection(
          "bridge_command_already_active",
          false,
          0U);
      }

      if (
        !safety_stop_observed_ ||
        !observation_fresh(
          safety_stop_observed_at_,
          admission_checked_at,
          config_.observed_state_timeout_ms))
      {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_safety_state_stale";

        return rejection(
          "bridge_teleop_safety_state_stale",
          false,
          0U);
      }

      if (safety_stop_active_) {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_safety_stop_active";

        return rejection(
          "bridge_teleop_safety_stop_active",
          false,
          0U);
      }

      if (
        !external_stop_observed_ ||
        !observation_fresh(
          external_stop_observed_at_,
          admission_checked_at,
          config_.observed_state_timeout_ms))
      {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_external_stop_state_stale";

        return rejection(
          "bridge_teleop_external_stop_state_stale",
          false,
          0U);
      }

      if (
        !mode_state_observed_ ||
        !observation_fresh(
          mode_state_observed_at_,
          admission_checked_at,
          config_.observed_state_timeout_ms))
      {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_mode_state_stale";

        return rejection(
          "bridge_teleop_mode_state_stale",
          false,
          0U);
      }

      if (
        !safe_velocity_observed_ ||
        !observation_fresh(
          safe_velocity_observed_at_,
          admission_checked_at,
          config_.observed_state_timeout_ms))
      {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_safe_velocity_stale";

        return rejection(
          "bridge_teleop_safe_velocity_stale",
          false,
          0U);
      }

      if (!safe_velocity_zero_) {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_robot_not_stationary";

        return rejection(
          "bridge_teleop_robot_not_stationary",
          false,
          0U);
      }

      command_active_ = true;
      teleop_active_ = true;
      teleop_cancel_requested_ = false;

      active_command_id_ =
        command.command_id;

      active_command_type_ =
        "teleop_nudge";

      last_reason_ =
        "bridge_teleop_admission_started";
    }

    const TimePoint requested_at =
      SteadyClock::now();

    std::size_t publications = 0U;

    // Release the bridge-owned STOP before requesting MANUAL.
    // Motion is not published until MANUAL is observed downstream.
    publish_external_stop(
      false,
      publications);

    publish_mode(
      config_.manual_mode,
      publications);

    try {
      std::thread worker(
        [this, command_id = command.command_id,
          teleop_payload = *payload,
          requested_at]()
        {
          run_teleop(
            command_id,
            teleop_payload,
            requested_at);
        });

      {
        std::lock_guard<std::mutex> lock(mutex_);

        teleop_thread_ =
          std::move(worker);

        ++accepted_command_count_;

        last_reason_ =
          "bridge_teleop_admitted";
      }
    } catch (...) {
      publish_zero_manual_velocity(
        publications);

      publish_external_stop(
        true,
        publications);

      publish_mode(
        config_.stop_mode,
        publications);

      {
        std::lock_guard<std::mutex> lock(mutex_);

        command_active_ = false;
        teleop_active_ = false;
        teleop_cancel_requested_ = false;

        active_command_id_.clear();
        active_command_type_.clear();

        last_terminal_command_id_ =
          command.command_id;

        ++rejected_command_count_;

        last_reason_ =
          "bridge_teleop_worker_start_failed";
      }

      return rejection(
        "bridge_teleop_worker_start_failed",
        true,
        publications);
    }

    return acceptance(
      "bridge_teleop_admitted",
      publications);
  }

  void run_teleop(
    std::string command_id,
    TeleopNudgeCommandPayload payload,
    const TimePoint requested_at) noexcept
  {
    std::size_t publications = 0U;

    std::string terminal_reason;
    bool fail_safe_stop = false;

    try {
      const auto admission_deadline =
        requested_at +
        std::chrono::milliseconds(
        config_.mode_transition_timeout_ms);

      bool downstream_ready = false;

      {
        std::unique_lock<std::mutex> lock(mutex_);

        downstream_ready =
          observation_changed_.wait_until(
          lock,
          admission_deadline,
          [this, requested_at]()
          {
            const TimePoint now =
              SteadyClock::now();

            if (
              shutdown_requested_ ||
              teleop_cancel_requested_)
            {
              return true;
            }

            const bool external_stop_released =
              external_stop_observed_ &&
              !external_stop_active_ &&
              external_stop_observed_at_ >=
              requested_at &&
              observation_fresh(
                external_stop_observed_at_,
                now,
                config_.observed_state_timeout_ms);

            const bool manual_mode_confirmed =
              mode_state_observed_ &&
              mode_state_ ==
              normalize_upper(
                config_.manual_mode) &&
              mode_state_observed_at_ >=
              requested_at &&
              observation_fresh(
                mode_state_observed_at_,
                now,
                config_.observed_state_timeout_ms);

            const bool safety_clear =
              safety_stop_observed_ &&
              !safety_stop_active_ &&
              observation_fresh(
                safety_stop_observed_at_,
                now,
                config_.observed_state_timeout_ms);

            const bool safe_velocity_fresh =
              safe_velocity_observed_ &&
              observation_fresh(
                safe_velocity_observed_at_,
                now,
                config_.observed_state_timeout_ms);

            return
              external_stop_released &&
              manual_mode_confirmed &&
              safety_clear &&
              safe_velocity_fresh;
          });

        if (shutdown_requested_) {
          terminal_reason =
            "bridge_teleop_shutdown";

          fail_safe_stop = true;
        } else if (teleop_cancel_requested_) {
          terminal_reason =
            "bridge_teleop_canceled";

          fail_safe_stop = true;
        } else if (!downstream_ready) {
          terminal_reason =
            "bridge_teleop_manual_mode_timeout";

          fail_safe_stop = true;
        } else if (
          !runtime_allows_teleop_locked(
            SteadyClock::now()))
        {
          terminal_reason =
            "bridge_teleop_runtime_not_ready";

          fail_safe_stop = true;
        }
      }

      if (terminal_reason.empty()) {
        geometry_msgs::msg::Twist command_message;

        command_message.linear.x =
          payload.linear_x_mps;

        command_message.linear.y =
          payload.linear_y_mps;

        command_message.angular.z =
          payload.angular_z_radps;

        const auto execution_deadline =
          SteadyClock::now() +
          std::chrono::milliseconds(
          payload.duration_ms);

        while (
          SteadyClock::now() <
          execution_deadline)
        {
          bool runtime_allowed = false;

          {
            std::lock_guard<std::mutex> lock(mutex_);

            const TimePoint now =
              SteadyClock::now();

            if (shutdown_requested_) {
              terminal_reason =
                "bridge_teleop_shutdown";

              fail_safe_stop = true;
              break;
            }

            if (teleop_cancel_requested_) {
              terminal_reason =
                "bridge_teleop_canceled";

              fail_safe_stop = true;
              break;
            }

            runtime_allowed =
              runtime_allows_teleop_locked(now);

            if (!runtime_allowed) {
              terminal_reason =
                "bridge_teleop_runtime_state_lost";

              fail_safe_stop = true;
              break;
            }
          }

          publish_manual_velocity(
            command_message,
            publications);

          const auto wake_at = std::min(
            execution_deadline,
            SteadyClock::now() +
            std::chrono::milliseconds(
            config_.teleop_publish_period_ms));

          std::unique_lock<std::mutex> lock(mutex_);

          observation_changed_.wait_until(
            lock,
            wake_at,
            [this]()
            {
              return
                shutdown_requested_ ||
                teleop_cancel_requested_;
            });
        }

        if (terminal_reason.empty()) {
          terminal_reason =
            "bridge_teleop_completed";
        }
      }

      publish_zero_manual_velocity(
        publications);

      if (fail_safe_stop) {
        publish_external_stop(
          true,
          publications);
      }

      publish_mode(
        config_.stop_mode,
        publications);
    } catch (...) {
      terminal_reason =
        "bridge_teleop_internal_error";

      try {
        publish_zero_manual_velocity(
          publications);

        publish_external_stop(
          true,
          publications);

        publish_mode(
          config_.stop_mode,
          publications);
      } catch (...) {
        // Preserve noexcept worker behavior.
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (active_command_id_ == command_id) {
        command_active_ = false;
        teleop_active_ = false;
        teleop_cancel_requested_ = false;

        active_command_id_.clear();
        active_command_type_.clear();
      }

      last_terminal_command_id_ =
        std::move(command_id);

      last_reason_ =
        terminal_reason.empty() ?
        "bridge_teleop_internal_error" :
        terminal_reason;
    }

    observation_changed_.notify_all();
  }

  [[nodiscard]] bool cancel_action_authorized(
    const ValidatedCommand & command) const
  {
    if (!command.origin_agent.has_value()) {
      return false;
    }

    const std::string & agent =
      command.origin_agent.value();

    return
      agent == "teleop_agent" ||
      agent == "safety_agent";
  }

  [[nodiscard]] CommandDispatchResult
  dispatch_cancel_action(
    const ValidatedCommand & command)
  {
    join_finished_teleop_thread();

    if (!cancel_action_authorized(command)) {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        ++rejected_command_count_;

        last_reason_ =
          "bridge_cancel_authority_rejected";
      }

      return rejection(
        "bridge_cancel_authority_rejected",
        false,
        0U);
    }

    const auto * payload =
      std::get_if<CancelActionCommandPayload>(
      &command.payload);

    if (payload == nullptr) {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        ++rejected_command_count_;

        last_reason_ =
          "bridge_cancel_payload_type_mismatch";
      }

      return rejection(
        "bridge_cancel_payload_type_mismatch",
        false,
        0U);
    }

    const std::string target_command_id =
      payload->target_command_id;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (shutdown_requested_) {
        ++rejected_command_count_;

        last_reason_ =
          "ros_command_dispatcher_shutdown";

        return rejection(
          "ros_command_dispatcher_shutdown",
          false,
          0U);
      }

      if (
        !command_active_ ||
        !teleop_active_)
      {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_cancel_target_not_active";

        return rejection(
          "bridge_cancel_target_not_active",
          false,
          0U);
      }

      if (
        active_command_id_ !=
        target_command_id)
      {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_cancel_target_mismatch";

        return rejection(
          "bridge_cancel_target_mismatch",
          false,
          0U);
      }

      teleop_cancel_requested_ = true;

      last_reason_ =
        "bridge_cancel_requested";
    }

    observation_changed_.notify_all();

    const auto deadline =
      SteadyClock::now() +
      std::chrono::milliseconds(
      config_.teleop_cancel_timeout_ms);

    bool target_terminal = false;
    std::string target_terminal_reason;
    std::string acknowledgement_reason;
    std::thread completed_thread;

    {
      std::unique_lock<std::mutex> lock(mutex_);

      target_terminal =
        observation_changed_.wait_until(
        lock,
        deadline,
        [this, &target_command_id]()
        {
          return
            !teleop_active_ &&
            active_command_id_ !=
            target_command_id;
        });

      if (target_terminal) {
        target_terminal_reason = last_reason_;

        if (teleop_thread_.joinable()) {
          completed_thread =
            std::move(teleop_thread_);
        }

        acknowledgement_reason =
          target_terminal_reason ==
          "bridge_teleop_canceled" ?
          "bridge_cancel_confirmed" :
          "bridge_cancel_target_terminal";

        ++accepted_command_count_;

        last_reason_ =
          acknowledgement_reason;
      } else {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_cancel_confirmation_timeout";
      }
    }

    if (completed_thread.joinable()) {
      completed_thread.join();
    }

    if (!target_terminal) {
      std::size_t publications = 0U;

      // The worker failed to acknowledge cancellation within
      // the bound. Apply an independent fail-safe STOP path.
      publish_zero_manual_velocity(
        publications);

      publish_external_stop(
        true,
        publications);

      publish_mode(
        config_.stop_mode,
        publications);

      return rejection(
        "bridge_cancel_confirmation_timeout",
        true,
        publications);
    }

    return acceptance(
      acknowledgement_reason,
      0U);
  }

  [[nodiscard]] bool stop_authorized(
    const ValidatedCommand & command) const
  {
    if (
      command.priority !=
      CommandPriority::Emergency)
    {
      return false;
    }

    if (!command.origin_agent.has_value()) {
      return false;
    }

    return
      command.origin_agent.value() ==
      "safety_agent";
  }

  [[nodiscard]] CommandDispatchResult dispatch_stop(
    const ValidatedCommand & command)
  {
    if (!stop_authorized(command)) {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        ++rejected_command_count_;

        last_reason_ =
          "bridge_stop_authority_rejected";
      }

      return rejection(
        "bridge_stop_authority_rejected",
        false,
        0U);
    }

    const TimePoint requested_at =
      SteadyClock::now();

    std::size_t publications = 0U;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      teleop_cancel_requested_ = true;
      navigation_cancel_requested_ = true;

      last_reason_ =
        "bridge_stop_dispatching";
    }

    // Cancellation flags are set before any mode or velocity
    // publications.
    publish_zero_manual_velocity(publications);
    publish_external_stop(true, publications);
    publish_mode(config_.stop_mode, publications);

    const auto deadline =
      requested_at +
      std::chrono::milliseconds(
      config_.stop_confirmation_timeout_ms);

    bool confirmed = false;

    {
      std::unique_lock<std::mutex> lock(mutex_);

      confirmed = observation_changed_.wait_until(
        lock,
        deadline,
        [this, requested_at]()
        {
          const TimePoint now = SteadyClock::now();

          const bool external_stop_confirmed =
            external_stop_observed_ &&
            external_stop_active_ &&
            external_stop_observed_at_ >= requested_at &&
            observation_fresh(
              external_stop_observed_at_,
              now,
              config_.observed_state_timeout_ms);

          const bool stop_mode_confirmed =
            mode_state_observed_ &&
            mode_state_ ==
            normalize_upper(config_.stop_mode) &&
            mode_state_observed_at_ >= requested_at &&
            observation_fresh(
              mode_state_observed_at_,
              now,
              config_.observed_state_timeout_ms);

          const bool safe_zero_confirmed =
            safe_velocity_observed_ &&
            safe_velocity_zero_ &&
            safe_velocity_observed_at_ >= requested_at &&
            observation_fresh(
              safe_velocity_observed_at_,
              now,
              config_.observed_state_timeout_ms);

          return
            external_stop_confirmed &&
            stop_mode_confirmed &&
            safe_zero_confirmed;
        });

      last_terminal_command_id_ =
        command.command_id;

      if (confirmed) {
        ++accepted_command_count_;

        last_reason_ =
          "bridge_stop_confirmed";
      } else {
        ++rejected_command_count_;

        last_reason_ =
          "bridge_stop_confirmation_timeout";
      }
    }

    if (!confirmed) {
      return rejection(
        "bridge_stop_confirmation_timeout",
        true,
        publications);
    }

    return acceptance(
      "bridge_stop_confirmed",
      publications);
  }

  void publish_mode(
    const std::string & mode,
    std::size_t & publications)
  {
    std_msgs::msg::String message;
    message.data = normalize_upper(mode);

    mode_command_publisher_->publish(message);

    ++publications;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++ros_publication_count_;
    }
  }

  void publish_external_stop(
    const bool active,
    std::size_t & publications)
  {
    std_msgs::msg::Bool message;
    message.data = active;

    external_stop_publisher_->publish(message);

    ++publications;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++ros_publication_count_;
    }
  }

  void publish_manual_velocity(
    const geometry_msgs::msg::Twist & message,
    std::size_t & publications)
  {
    manual_velocity_publisher_->publish(message);

    ++publications;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++ros_publication_count_;
    }
  }

  void publish_zero_manual_velocity(
    std::size_t & publications)
  {
    geometry_msgs::msg::Twist zero;

    for (
      std::size_t index = 0U;
      index < config_.final_zero_publication_count;
      ++index)
    {
      manual_velocity_publisher_->publish(zero);

      ++publications;

      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++ros_publication_count_;
      }
    }
  }

  rclcpp::Node & node_;
  RosCommandDispatcherConfig config_;

  mutable std::mutex mutex_;
  std::condition_variable observation_changed_;

  bool shutdown_requested_{false};

  bool command_active_{false};
  bool teleop_active_{false};
  bool teleop_cancel_requested_{false};
  bool navigation_goal_active_{false};
  bool navigation_cancel_requested_{false};

  std::string active_command_id_;
  std::string active_command_type_;
  std::string last_terminal_command_id_;

  bool mode_state_observed_{false};
  std::string mode_state_;
  TimePoint mode_state_observed_at_{};

  bool external_stop_observed_{false};
  bool external_stop_active_{false};
  TimePoint external_stop_observed_at_{};

  bool safety_stop_observed_{false};
  bool safety_stop_active_{false};
  TimePoint safety_stop_observed_at_{};

  bool safe_velocity_observed_{false};
  bool safe_velocity_zero_{false};
  TimePoint safe_velocity_observed_at_{};

  bool navigation_readiness_observed_{false};
  std::string navigation_readiness_;
  TimePoint navigation_readiness_observed_at_{};

  std::uint64_t accepted_command_count_{0U};
  std::uint64_t rejected_command_count_{0U};
  std::uint64_t ros_publication_count_{0U};

  std::string last_reason_{
    "ros_command_dispatcher_not_started"};

  std::thread teleop_thread_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    mode_command_publisher_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    external_stop_publisher_;

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr
    manual_velocity_publisher_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    mode_state_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    external_stop_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    safety_stop_subscription_;

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr
    safe_velocity_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    navigation_readiness_subscription_;
};

RosCommandDispatcher::RosCommandDispatcher(
  rclcpp::Node & node,
  RosCommandDispatcherConfig config)
: impl_(std::make_unique<Impl>(
    node,
    std::move(config)))
{
}

RosCommandDispatcher::~RosCommandDispatcher() = default;

CommandDispatchResult RosCommandDispatcher::dispatch(
  const ValidatedCommand & command)
{
  return impl_->dispatch(command);
}

RosCommandDispatcherSnapshot
RosCommandDispatcher::snapshot() const
{
  return impl_->snapshot();
}

void RosCommandDispatcher::shutdown() noexcept
{
  impl_->shutdown();
}

}  // namespace savo_bridge
