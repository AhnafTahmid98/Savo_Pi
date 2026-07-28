#include "savo_mapping/parameter_utils.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/scan360_controller.hpp"
#include "savo_mapping/scan360_orchestrator.hpp"
#include "savo_mapping/scan360_planner.hpp"
#include "savo_mapping/scan360_rotate_action_binding.hpp"
#include "savo_mapping/scan360_rotate_action_client.hpp"
#include "savo_mapping/topic_names.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace savo_mapping
{
namespace
{

using SteadyClock = std::chrono::steady_clock;
using TimePoint = SteadyClock::time_point;

constexpr std::size_t kMaximumDecisionSteps = 16U;
constexpr std::size_t kMaximumEventRounds = 16U;
constexpr double kMinimumQuaternionNorm = 1.0e-9;
constexpr double kMinimumTickPeriodSec = 0.01;
constexpr double kMaximumTickPeriodSec = 1.0;

std::atomic<bool> shutdown_requested{false};

void request_shutdown([[maybe_unused]] int signal_number)
{
  shutdown_requested.store(true);
}

std::string require_non_empty(
  std::string_view name,
  std::string value)
{
  if (value.empty()) {
    throw std::invalid_argument(
            "parameter '" + std::string{name} + "' must not be empty");
  }

  return value;
}

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

std::string json_escape(const std::string & input)
{
  std::ostringstream output;

  for (const char character : input) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;

      case '"':
        output << "\\\"";
        break;

      case '\n':
        output << "\\n";
        break;

      case '\r':
        output << "\\r";
        break;

      case '\t':
        output << "\\t";
        break;

      default:
        output << character;
        break;
    }
  }

  return output.str();
}

std::chrono::nanoseconds seconds_to_period(
  const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

scan360::RotationDirection parse_direction(
  const std::string & direction)
{
  if (
    direction ==
    scan360::to_string(
      scan360::RotationDirection::CounterClockwise))
  {
    return scan360::RotationDirection::CounterClockwise;
  }

  if (
    direction ==
    scan360::to_string(
      scan360::RotationDirection::Clockwise))
  {
    return scan360::RotationDirection::Clockwise;
  }

  throw std::invalid_argument(
          "direction must be counter_clockwise or clockwise");
}

}  // namespace

class Scan360MapperNode final : public rclcpp::Node
{
public:
  using NativeClient = Scan360RotateActionClient;
  using StringMessage = std_msgs::msg::String;

  Scan360MapperNode()
  : Node("scan360_mapper_node")
  {
    declare_and_validate_parameters();
    create_observability();
    create_odom_subscription();

    set_state(
      enabled_ ? "waiting_for_odom" : "disabled",
      enabled_ ? "waiting_for_valid_odom_yaw" : "disabled_by_configuration");

    RCLCPP_INFO(
      get_logger(),
      "Scan360 mapper foundation ready: enabled=%s auto_start=%s "
      "odom_topic=%s odom_frame=%s",
      bool_text(enabled_),
      bool_text(auto_start_),
      odom_topic_.c_str(),
      odom_frame_.c_str());
  }

  void initialize_runtime()
  {
    NativeClient::Options options;
    options.action_name = action_name_;
    options.server_wait_timeout_sec = server_wait_timeout_sec_;
    options.goal_response_timeout_sec = goal_response_timeout_sec_;
    options.feedback_stale_timeout_sec = feedback_stale_timeout_sec_;
    options.cancel_timeout_sec = cancel_timeout_sec_;
    options.execution_grace_sec = execution_grace_timeout_sec_;

    native_client_ = NativeClient::create(
      std::static_pointer_cast<rclcpp::Node>(
        shared_from_this()),
      std::move(options));

    orchestrator_ =
      std::make_unique<scan360::Scan360Orchestrator>(
      scan360::make_scan360_rotate_action_callbacks(
        native_client_),
      rotation_max_duration_sec_);

    tick_timer_ = create_wall_timer(
      seconds_to_period(tick_period_sec_),
      std::bind(&Scan360MapperNode::tick, this));

    publish_status();
  }

  void begin_bounded_shutdown()
  {
    if (shutdown_started_) {
      return;
    }

    shutdown_started_ = true;

    if (!scan_active_ || terminal_) {
      shutdown_complete_ = true;
      return;
    }

    const auto decision =
      controller_.handle(
      scan360::ControllerEvent::OperatorCancel);

    process_decisions({decision});

    if (
      controller_.state() ==
      scan360::ControllerState::Canceled ||
      controller_.state() ==
      scan360::ControllerState::Failed)
    {
      shutdown_complete_ = true;
      return;
    }

    shutdown_deadline_ =
      SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(
        cancel_timeout_sec_ + tick_period_sec_ * 2.0));
  }

  void shutdown_tick()
  {
    if (!shutdown_started_ || shutdown_complete_) {
      return;
    }

    pump_orchestrator();

    if (
      controller_.state() ==
      scan360::ControllerState::Canceled ||
      controller_.state() ==
      scan360::ControllerState::Failed)
    {
      shutdown_complete_ = true;
      return;
    }

    if (SteadyClock::now() >= shutdown_deadline_) {
      mark_terminal(
        "failed",
        "shutdown_cancel_timeout");

      shutdown_complete_ = true;
    }
  }

  [[nodiscard]] bool shutdown_complete() const
  {
    return shutdown_complete_;
  }

  [[nodiscard]] double shutdown_bound_sec() const
  {
    return cancel_timeout_sec_ + tick_period_sec_ * 4.0 + 0.25;
  }

private:
  void declare_and_validate_parameters()
  {
    enabled_ = params::declare_or_get<bool>(
      *this, "enabled", true);

    auto_start_ = params::declare_or_get<bool>(
      *this, "auto_start", false);

    action_name_ = require_non_empty(
      "action_name",
      params::declare_or_get<std::string>(
        *this,
        "action_name",
        "/savo_control/rotate_to_heading"));

    odom_topic_ = require_non_empty(
      "odom_topic",
      params::declare_or_get<std::string>(
        *this,
        "odom_topic",
        std::string{topics::ODOM_FILTERED}));

    odom_frame_ = require_non_empty(
      "odom_frame",
      params::declare_or_get<std::string>(
        *this,
        "odom_frame",
        "odom"));

    status_topic_ = require_non_empty(
      "status_topic",
      params::declare_or_get<std::string>(
        *this,
        "status_topic",
        "/savo_mapping/scan360/status"));

    state_topic_ = require_non_empty(
      "state_topic",
      params::declare_or_get<std::string>(
        *this,
        "state_topic",
        "/savo_mapping/scan360/state"));

    yaw_stale_timeout_sec_ =
      params::require_positive_parameter(
      "yaw_stale_timeout_sec",
      params::declare_or_get<double>(
        *this,
        "yaw_stale_timeout_sec",
        1.0));

    plan_options_.sweep_angle_rad =
      params::require_positive_parameter(
      "sweep_angle_rad",
      params::declare_or_get<double>(
        *this,
        "sweep_angle_rad",
        scan360::kTwoPi));

    plan_options_.step_angle_rad =
      params::require_positive_parameter(
      "step_angle_rad",
      params::declare_or_get<double>(
        *this,
        "step_angle_rad",
        scan360::kPi / 6.0));

    const auto direction =
      params::declare_or_get<std::string>(
      *this,
      "direction",
      "counter_clockwise");

    plan_options_.direction =
      parse_direction(direction);

    settle_duration_sec_ =
      params::require_non_negative_parameter(
      "settle_duration_sec",
      params::declare_or_get<double>(
        *this,
        "settle_duration_sec",
        0.5));

    rotation_max_duration_sec_ =
      params::require_positive_parameter(
      "rotation_max_duration_sec",
      params::declare_or_get<double>(
        *this,
        "rotation_max_duration_sec",
        20.0));

    tick_period_sec_ =
      params::require_parameter_in_closed_range(
      "tick_period_sec",
      params::declare_or_get<double>(
        *this,
        "tick_period_sec",
        0.05),
      kMinimumTickPeriodSec,
      kMaximumTickPeriodSec);

    server_wait_timeout_sec_ =
      params::require_positive_parameter(
      "server_wait_timeout_sec",
      params::declare_or_get<double>(
        *this,
        "server_wait_timeout_sec",
        1.0));

    goal_response_timeout_sec_ =
      params::require_positive_parameter(
      "goal_response_timeout_sec",
      params::declare_or_get<double>(
        *this,
        "goal_response_timeout_sec",
        1.5));

    feedback_stale_timeout_sec_ =
      params::require_positive_parameter(
      "feedback_stale_timeout_sec",
      params::declare_or_get<double>(
        *this,
        "feedback_stale_timeout_sec",
        3.0));

    cancel_timeout_sec_ =
      params::require_positive_parameter(
      "cancel_timeout_sec",
      params::declare_or_get<double>(
        *this,
        "cancel_timeout_sec",
        2.0));

    execution_grace_timeout_sec_ =
      params::require_non_negative_parameter(
      "execution_grace_timeout_sec",
      params::declare_or_get<double>(
        *this,
        "execution_grace_timeout_sec",
        1.0));

    const auto validation =
      scan360::make_plan(0.0, plan_options_);

    if (!validation.accepted) {
      throw std::invalid_argument(
              "invalid Scan360 plan parameters: " +
              validation.reason);
    }
  }

  void create_observability()
  {
    state_publisher_ =
      create_publisher<StringMessage>(
      state_topic_,
      qos::state_qos());

    status_publisher_ =
      create_publisher<StringMessage>(
      status_topic_,
      qos::state_qos());
  }

  void create_odom_subscription()
  {
    odom_subscription_ =
      create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile(),
      std::bind(
        &Scan360MapperNode::handle_odom,
        this,
        std::placeholders::_1));
  }

  void handle_odom(
    const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    odom_valid_ = false;

    if (message->header.frame_id != odom_frame_) {
      odom_reason_ = "odom_frame_mismatch";
      publish_status();
      return;
    }

    const auto & orientation =
      message->pose.pose.orientation;

    if (
      !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      odom_reason_ = "odom_quaternion_not_finite";
      publish_status();
      return;
    }

    const double norm_squared =
      orientation.x * orientation.x +
      orientation.y * orientation.y +
      orientation.z * orientation.z +
      orientation.w * orientation.w;

    if (!std::isfinite(norm_squared)) {
      odom_reason_ = "odom_quaternion_norm_not_finite";
      publish_status();
      return;
    }

    const double norm = std::sqrt(norm_squared);

    if (
      !std::isfinite(norm) ||
      norm <= kMinimumQuaternionNorm)
    {
      odom_reason_ = "odom_quaternion_norm_too_small";
      publish_status();
      return;
    }

    const double x = orientation.x / norm;
    const double y = orientation.y / norm;
    const double z = orientation.z / norm;
    const double w = orientation.w / norm;

    const double yaw_rad = std::atan2(
      2.0 * (w * z + x * y),
      1.0 - 2.0 * (y * y + z * z));

    if (!std::isfinite(yaw_rad)) {
      odom_reason_ = "odom_yaw_not_finite";
      publish_status();
      return;
    }

    current_yaw_rad_ =
      scan360::normalize_yaw(yaw_rad);

    if (!std::isfinite(current_yaw_rad_)) {
      odom_reason_ = "normalized_odom_yaw_not_finite";
      publish_status();
      return;
    }

    odom_received_at_ = SteadyClock::now();
    odom_valid_ = true;
    odom_reason_ = "odom_yaw_valid";
    publish_status();
  }

  [[nodiscard]] bool odom_is_fresh() const
  {
    if (!odom_valid_ || !odom_received_at_.has_value()) {
      return false;
    }

    const double age_sec =
      std::chrono::duration<double>(
      SteadyClock::now() -
      odom_received_at_.value()).count();

    return
      std::isfinite(age_sec) &&
      age_sec >= 0.0 &&
      age_sec <= yaw_stale_timeout_sec_;
  }

  void tick()
  {
    if (shutdown_started_) {
      return;
    }

    const bool odom_fresh = odom_is_fresh();

    if (!enabled_) {
      set_state(
        "disabled",
        "disabled_by_configuration");

      return;
    }

    if (
      auto_start_ &&
      !scan_started_ &&
      !terminal_)
    {
      if (odom_fresh) {
        start_scan();
      } else {
        set_state(
          "waiting_for_odom",
          odom_valid_ ?
          "odom_yaw_stale" :
          odom_reason_);
      }
    }

    if (scan_started_ && !terminal_) {
      pump_orchestrator();
      complete_settle_if_due();
      pump_orchestrator();
    }

    publish_status();
  }

  void start_scan()
  {
    const auto result =
      scan360::make_plan(
      current_yaw_rad_,
      plan_options_);

    if (!result.accepted) {
      mark_terminal(
        "failed",
        "plan_rejected:" + result.reason);

      return;
    }

    target_count_ = result.plan.targets.size();

    const auto loaded =
      controller_.load_plan(result.plan);

    if (
      loaded.state ==
      scan360::ControllerState::Failed)
    {
      mark_terminal(
        "failed",
        loaded.reason);

      return;
    }

    orchestrator_->reset();
    scan_started_ = true;

    set_state(
      "ready",
      loaded.reason);

    process_decisions(
      {controller_.handle(
          scan360::ControllerEvent::Start)});
  }

  void pump_orchestrator()
  {
    if (!orchestrator_ || terminal_) {
      return;
    }

    for (
      std::size_t round = 0;
      round < kMaximumEventRounds;
      ++round)
    {
      orchestrator_->tick();

      auto events =
        orchestrator_->take_events();

      if (events.empty()) {
        return;
      }

      std::vector<scan360::ControllerDecision>
      decisions;

      decisions.reserve(events.size());

      for (const auto event : events) {
        decisions.push_back(
          controller_.handle(event));
      }

      process_decisions(
        std::move(decisions));

      if (terminal_) {
        return;
      }
    }

    mark_terminal(
      "failed",
      "orchestrator_event_round_limit_exceeded");
  }

  void process_decisions(
    std::vector<scan360::ControllerDecision>
    decisions)
  {
    std::size_t next = 0U;
    std::size_t processed = 0U;

    while (next < decisions.size()) {
      if (++processed > kMaximumDecisionSteps) {
        mark_terminal(
          "failed",
          "controller_decision_limit_exceeded");

        return;
      }

      const auto decision =
        decisions[next++];

      last_reason_ = decision.reason;

      if (decision.target.has_value()) {
        target_yaw_rad_ =
          decision.target->normalized_yaw_rad;
      }

      switch (decision.action) {
        case scan360::ControllerAction::None:
          break;

        case scan360::ControllerAction::IssueRotationRequest:
          if (terminal_ || cancellation_started_) {
            mark_terminal(
              "failed",
              "rotation_request_after_terminal_or_cancel");

            return;
          }

          settling_ = false;
          orchestrator_->dispatch(decision);
          set_state(
            std::string{
            scan360::to_string(decision.state)},
            decision.reason);
          break;

        case scan360::ControllerAction::RequestCancel:
          cancellation_started_ = true;
          settling_ = false;
          orchestrator_->dispatch(decision);
          set_state(
            "canceling",
            decision.reason);
          break;

        case scan360::ControllerAction::StartSettleTimer:
          settling_ = true;

          settle_deadline_ =
            SteadyClock::now() +
            std::chrono::duration_cast<
            SteadyClock::duration>(
            std::chrono::duration<double>(
              settle_duration_sec_));

          set_state(
            "settling",
            decision.reason);

          if (settle_duration_sec_ == 0.0) {
            settling_ = false;

            decisions.push_back(
              controller_.handle(
                scan360::ControllerEvent::
                SettleComplete));
          }
          break;

        case scan360::ControllerAction::ScanComplete:
          mark_terminal(
            "complete",
            decision.reason);
          return;
      }

      const auto state = controller_.state();

      if (state == scan360::ControllerState::Canceled) {
        mark_terminal(
          "canceled",
          decision.reason);
        return;
      }

      if (state == scan360::ControllerState::Failed) {
        mark_terminal(
          "failed",
          decision.reason);
        return;
      }
    }
  }

  void complete_settle_if_due()
  {
    if (
      !settling_ ||
      SteadyClock::now() < settle_deadline_)
    {
      return;
    }

    settling_ = false;

    process_decisions(
      {controller_.handle(
          scan360::ControllerEvent::
          SettleComplete)});
  }

  void mark_terminal(
    std::string state,
    std::string reason)
  {
    if (terminal_) {
      return;
    }

    terminal_ = true;
    scan_active_ = false;
    settling_ = false;
    last_reason_ = std::move(reason);

    set_state(
      std::move(state),
      last_reason_);
  }

  void set_state(
    std::string state,
    std::string reason)
  {
    const bool changed =
      state != state_ ||
      reason != last_reason_;

    state_ = std::move(state);
    last_reason_ = std::move(reason);

    const auto controller_state =
      controller_.state();

    scan_active_ =
      controller_state ==
      scan360::ControllerState::CommandPending ||
      controller_state ==
      scan360::ControllerState::Rotating ||
      controller_state ==
      scan360::ControllerState::Settling ||
      controller_state ==
      scan360::ControllerState::Canceling;

    if (changed && state_publisher_) {
      StringMessage message;
      message.data = state_;
      state_publisher_->publish(message);
    }

    publish_status();
  }

  void publish_status()
  {
    if (!status_publisher_) {
      return;
    }

    NativeClient::Snapshot native_snapshot;

    if (native_client_) {
      native_snapshot =
        native_client_->snapshot();
    }

    const bool odom_fresh = odom_is_fresh();

    std::ostringstream output;

    output
      << std::boolalpha
      << "{\"enabled\":"
      << enabled_
      << ",\"ready\":"
      << (
      enabled_ &&
      odom_fresh &&
      !terminal_)
      << ",\"state\":\""
      << json_escape(state_)
      << "\",\"reason\":\""
      << json_escape(last_reason_)
      << "\",\"current_target_index\":"
      << controller_.current_target_index()
      << ",\"target_count\":"
      << target_count_
      << ",\"current_yaw_rad\":"
      << current_yaw_rad_
      << ",\"target_yaw_rad\":"
      << target_yaw_rad_
      << ",\"error_rad\":"
      << native_snapshot.error_rad
      << ",\"action_client_state\":\""
      << NativeClient::to_string(
      native_snapshot.state)
      << "\",\"action_reason\":\""
      << json_escape(
      native_snapshot.reason)
      << "\",\"settling\":"
      << settling_
      << ",\"terminal\":"
      << terminal_
      << ",\"odom_valid\":"
      << odom_valid_
      << ",\"odom_fresh\":"
      << odom_fresh
      << "}";

    StringMessage message;
    message.data = output.str();
    status_publisher_->publish(message);
  }

  bool enabled_{true};
  bool auto_start_{false};

  std::string action_name_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string status_topic_;
  std::string state_topic_;

  double yaw_stale_timeout_sec_{1.0};
  double settle_duration_sec_{0.5};
  double rotation_max_duration_sec_{20.0};
  double tick_period_sec_{0.05};
  double server_wait_timeout_sec_{1.0};
  double goal_response_timeout_sec_{1.5};
  double feedback_stale_timeout_sec_{3.0};
  double cancel_timeout_sec_{2.0};
  double execution_grace_timeout_sec_{1.0};

  scan360::Scan360PlanOptions plan_options_;
  scan360::Scan360Controller controller_;

  NativeClient::SharedPtr native_client_;

  std::unique_ptr<
    scan360::Scan360Orchestrator>
  orchestrator_;

  bool odom_valid_{false};
  std::string odom_reason_{"odom_not_received"};

  std::optional<TimePoint>
  odom_received_at_;

  double current_yaw_rad_{0.0};
  double target_yaw_rad_{0.0};

  std::size_t target_count_{0U};

  bool scan_started_{false};
  bool scan_active_{false};
  bool cancellation_started_{false};
  bool settling_{false};
  bool terminal_{false};

  bool shutdown_started_{false};
  bool shutdown_complete_{false};
  TimePoint shutdown_deadline_{};
  TimePoint settle_deadline_{};

  std::string state_{"initializing"};
  std::string last_reason_{"startup"};

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::SharedPtr
    odom_subscription_;

  rclcpp::Publisher<
    StringMessage>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<
    StringMessage>::SharedPtr
    status_publisher_;

  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;

  rclcpp::init(
    argc,
    argv,
    init_options,
    rclcpp::SignalHandlerOptions::None);

  std::signal(SIGINT, savo_mapping::request_shutdown);
  std::signal(SIGTERM, savo_mapping::request_shutdown);

  try {
    auto node =
      std::make_shared<
      savo_mapping::Scan360MapperNode>();

    node->initialize_runtime();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (
      rclcpp::ok() &&
      !savo_mapping::shutdown_requested.load())
    {
      executor.spin_some();

      std::this_thread::sleep_for(
        std::chrono::milliseconds(5));
    }

    node->begin_bounded_shutdown();

    const auto deadline =
      savo_mapping::SteadyClock::now() +
      std::chrono::duration_cast<
      savo_mapping::SteadyClock::duration>(
      std::chrono::duration<double>(
        node->shutdown_bound_sec()));

    while (
      rclcpp::ok() &&
      !node->shutdown_complete() &&
      savo_mapping::SteadyClock::now() <
      deadline)
    {
      executor.spin_some();
      node->shutdown_tick();

      std::this_thread::sleep_for(
        std::chrono::milliseconds(5));
    }

    executor.remove_node(node);
    node.reset();
  } catch (const std::exception & error) {
    std::cerr
      << "scan360_mapper_node failed: "
      << error.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
