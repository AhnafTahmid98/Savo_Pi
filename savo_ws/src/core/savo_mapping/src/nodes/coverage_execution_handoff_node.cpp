#include "savo_mapping/coverage_execution_handoff.hpp"
#include "savo_mapping/coverage_mission.hpp"
#include "savo_mapping/qos_profiles.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <savo_msgs/action/execute_coverage_path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_mapping
{
namespace
{

constexpr double kQuaternionTolerance = 1.0e-3;
constexpr double kPlanarTolerance = 1.0e-6;
constexpr std::int64_t kMinimumWatchdogPeriodMs = 20;
constexpr std::int64_t kMaximumWatchdogPeriodMs = 1000;
constexpr std::uint32_t kMaximumConfiguredWaypointLimit = 100000U;

bool text_is_blank(const std::string & value)
{
  return value.empty() ||
         std::all_of(
    value.begin(),
    value.end(),
    [](const unsigned char character) {
      return std::isspace(character) != 0;
    });
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

builtin_interfaces::msg::Duration duration_from_seconds(
  const double seconds)
{
  builtin_interfaces::msg::Duration result;
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    return result;
  }

  const double capped = std::min(
    seconds,
    static_cast<double>(
      std::numeric_limits<std::int32_t>::max()));
  result.sec = static_cast<std::int32_t>(std::floor(capped));
  const double fraction =
    capped - static_cast<double>(result.sec);
  auto nanoseconds = static_cast<std::uint64_t>(
    std::llround(fraction * 1.0e9));
  if (nanoseconds >= 1000000000ULL) {
    if (result.sec <
      std::numeric_limits<std::int32_t>::max())
    {
      ++result.sec;
      nanoseconds = 0U;
    } else {
      nanoseconds = 999999999ULL;
    }
  }
  result.nanosec = static_cast<std::uint32_t>(nanoseconds);
  return result;
}

struct CandidateValidation
{
  bool valid{false};
  std::string reason;
  double total_distance_m{0.0};
};

CandidateValidation validate_candidate_path(
  const nav_msgs::msg::Path & path,
  const std::string & expected_frame,
  const std::uint32_t maximum_waypoints)
{
  if (path.header.frame_id != expected_frame) {
    return {
      false,
      path.header.frame_id.empty() ?
      "coverage_handoff_path_frame_empty" :
      "coverage_handoff_path_frame_mismatch",
      0.0};
  }

  if (path.poses.empty()) {
    return {
      false,
      "coverage_handoff_path_empty",
      0.0};
  }

  if (path.poses.size() > maximum_waypoints) {
    return {
      false,
      "coverage_handoff_waypoint_limit_exceeded",
      0.0};
  }

  double total_distance_m = 0.0;
  for (std::size_t index = 0;
    index < path.poses.size(); ++index)
  {
    const auto & pose = path.poses[index];
    if (pose.header.frame_id != expected_frame) {
      return {
        false,
        "coverage_handoff_waypoint_frame_mismatch",
        0.0};
    }

    const auto & position = pose.pose.position;
    const auto & orientation = pose.pose.orientation;
    if (!std::isfinite(position.x) ||
      !std::isfinite(position.y) ||
      !std::isfinite(position.z))
    {
      return {
        false,
        "coverage_handoff_waypoint_position_invalid",
        0.0};
    }

    if (std::abs(position.z) > kPlanarTolerance) {
      return {
        false,
        "coverage_handoff_waypoint_not_planar",
        0.0};
    }

    if (!std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      return {
        false,
        "coverage_handoff_waypoint_orientation_invalid",
        0.0};
    }

    if (std::abs(orientation.x) > kPlanarTolerance ||
      std::abs(orientation.y) > kPlanarTolerance)
    {
      return {
        false,
        "coverage_handoff_waypoint_orientation_not_planar",
        0.0};
    }

    const double norm = std::sqrt(
      orientation.x * orientation.x +
      orientation.y * orientation.y +
      orientation.z * orientation.z +
      orientation.w * orientation.w);
    if (!std::isfinite(norm) ||
      std::abs(norm - 1.0) > kQuaternionTolerance)
    {
      return {
        false,
        "coverage_handoff_waypoint_orientation_not_normalized",
        0.0};
    }

    if (index > 0U) {
      const auto & previous =
        path.poses[index - 1U].pose.position;
      const double segment = std::hypot(
        position.x - previous.x,
        position.y - previous.y);
      if (!std::isfinite(segment)) {
        return {
          false,
          "coverage_handoff_path_distance_invalid",
          0.0};
      }
      total_distance_m += segment;
      if (!std::isfinite(total_distance_m)) {
        return {
          false,
          "coverage_handoff_path_distance_invalid",
          0.0};
      }
    }
  }

  return {
    true,
    "coverage_handoff_plan_valid",
    total_distance_m};
}

}  // namespace

class CoverageExecutionHandoffNode final : public rclcpp::Node
{
public:
  using ExecuteCoveragePath =
    savo_msgs::action::ExecuteCoveragePath;
  using ActionClient =
    rclcpp_action::Client<ExecuteCoveragePath>;
  using GoalHandle = ActionClient::GoalHandle;
  using CancelResponse = ActionClient::CancelResponse;

  CoverageExecutionHandoffNode()
  : Node("coverage_execution_handoff_node")
  {
    declare_and_validate_parameters();
    create_interfaces();
    publish_state();
    publish_status();

    RCLCPP_INFO(
      get_logger(),
      "Coverage execution handoff ready: action=%s",
      action_name_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Publishing %s never dispatches motion; approval service=%s",
      plan_topic_.c_str(),
      approve_service_name_.c_str());
  }

private:
  void declare_and_validate_parameters()
  {
    enabled_ = declare_parameter<bool>(
      "enabled", true);
    expected_frame_ = declare_parameter<std::string>(
      "expected_frame", "map");
    mission_id_prefix_ = declare_parameter<std::string>(
      "mission_id_prefix", "coverage");

    plan_topic_ = declare_parameter<std::string>(
      "plan_topic", coverage::kCoveragePlanTopic);
    action_name_ = declare_parameter<std::string>(
      "action_name", coverage::kCoverageExecutionActionName);
    state_topic_ = declare_parameter<std::string>(
      "state_topic", coverage::kCoverageExecutionStateTopic);
    status_topic_ = declare_parameter<std::string>(
      "status_topic", coverage::kCoverageExecutionStatusTopic);
    feedback_topic_ = declare_parameter<std::string>(
      "feedback_topic", coverage::kCoverageExecutionFeedbackTopic);
    approve_service_name_ = declare_parameter<std::string>(
      "approve_service", coverage::kCoverageExecutionApproveService);
    cancel_service_name_ = declare_parameter<std::string>(
      "cancel_service", coverage::kCoverageExecutionCancelService);
    reset_service_name_ = declare_parameter<std::string>(
      "reset_service", coverage::kCoverageExecutionResetService);

    policy_.server_wait_timeout_sec =
      declare_parameter<double>(
      "server_wait_timeout_sec", 3.0);
    policy_.goal_response_timeout_sec =
      declare_parameter<double>(
      "goal_response_timeout_sec", 3.0);
    policy_.cancel_timeout_sec =
      declare_parameter<double>(
      "cancel_timeout_sec", 5.0);
    policy_.execution_timeout_sec =
      declare_parameter<double>(
      "execution_timeout_sec", 0.0);

    const auto maximum_waypoints =
      declare_parameter<std::int64_t>(
      "maximum_waypoints", 10000);
    const auto watchdog_period_ms =
      declare_parameter<std::int64_t>(
      "watchdog_period_ms", 100);

    const std::array<std::pair<const char *, const std::string *>, 10>
    text_parameters{{
      {"expected_frame", &expected_frame_},
      {"mission_id_prefix", &mission_id_prefix_},
      {"plan_topic", &plan_topic_},
      {"action_name", &action_name_},
      {"state_topic", &state_topic_},
      {"status_topic", &status_topic_},
      {"feedback_topic", &feedback_topic_},
      {"approve_service", &approve_service_name_},
      {"cancel_service", &cancel_service_name_},
      {"reset_service", &reset_service_name_},
    }};
    for (const auto & [name, value] : text_parameters) {
      if (text_is_blank(*value)) {
        throw std::invalid_argument(
                std::string{"coverage_handoff_parameter_empty:"} +
                name);
      }
    }

    if (maximum_waypoints <= 0 ||
      maximum_waypoints >
      static_cast<std::int64_t>(
        kMaximumConfiguredWaypointLimit))
    {
      throw std::invalid_argument(
              "coverage_handoff_maximum_waypoints_invalid");
    }
    policy_.maximum_waypoints =
      static_cast<std::uint32_t>(maximum_waypoints);

    const auto policy_error =
      coverage::validate_coverage_execution_handoff_policy(
      policy_);
    if (!policy_error.empty()) {
      throw std::invalid_argument(policy_error);
    }

    if (coverage::make_coverage_mission_id(
        mission_id_prefix_, 1U, 0).empty())
    {
      throw std::invalid_argument(
              "coverage_handoff_mission_id_prefix_invalid");
    }

    if (watchdog_period_ms < kMinimumWatchdogPeriodMs ||
      watchdog_period_ms > kMaximumWatchdogPeriodMs)
    {
      throw std::invalid_argument(
              "coverage_handoff_watchdog_period_invalid");
    }
    watchdog_period_ms_ = watchdog_period_ms;
  }

  void create_interfaces()
  {
    state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      state_topic_, qos::state_qos());
    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      status_topic_, qos::status_qos());
    feedback_publisher_ =
      create_publisher<std_msgs::msg::String>(
      feedback_topic_, qos::status_qos());

    plan_subscription_ =
      create_subscription<nav_msgs::msg::Path>(
      plan_topic_,
      qos::state_qos(),
      std::bind(
        &CoverageExecutionHandoffNode::handle_plan,
        this,
        std::placeholders::_1));

    approve_service_ =
      create_service<std_srvs::srv::Trigger>(
      approve_service_name_,
      std::bind(
        &CoverageExecutionHandoffNode::handle_approve,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    cancel_service_ =
      create_service<std_srvs::srv::Trigger>(
      cancel_service_name_,
      std::bind(
        &CoverageExecutionHandoffNode::handle_cancel,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    reset_service_ =
      create_service<std_srvs::srv::Trigger>(
      reset_service_name_,
      std::bind(
        &CoverageExecutionHandoffNode::handle_reset,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    action_client_ =
      rclcpp_action::create_client<ExecuteCoveragePath>(
      this, action_name_);

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(watchdog_period_ms_),
      std::bind(
        &CoverageExecutionHandoffNode::watchdog_tick,
        this));
  }

  std::int64_t next_event_time_ns()
  {
    auto value = now().nanoseconds();
    if (value <= last_event_time_ns_) {
      value = last_event_time_ns_ + 1;
    }
    last_event_time_ns_ = value;
    return value;
  }

  void handle_plan(
    const nav_msgs::msg::Path::SharedPtr message)
  {
    const auto validation = validate_candidate_path(
      *message,
      expected_frame_,
      policy_.maximum_waypoints);

    ++candidate_generation_;
    candidate_received_at_ =
      std::chrono::steady_clock::now();
    candidate_reason_ = validation.reason;
    candidate_valid_ = validation.valid;
    candidate_total_distance_m_ =
      validation.total_distance_m;

    if (validation.valid) {
      candidate_path_ = *message;
      RCLCPP_INFO(
        get_logger(),
        "Coverage plan staged: generation=%llu waypoints=%zu distance=%.3f",
        static_cast<unsigned long long>(candidate_generation_),
        candidate_path_->poses.size(),
        candidate_total_distance_m_);
    } else {
      candidate_path_.reset();
      RCLCPP_WARN(
        get_logger(),
        "Coverage plan rejected: generation=%llu reason=%s",
        static_cast<unsigned long long>(candidate_generation_),
        validation.reason.c_str());
    }

    publish_state();
    publish_status();
  }

  void handle_approve(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!enabled_) {
      response->success = false;
      response->message = "coverage_handoff_disabled";
      return;
    }

    if (!candidate_valid_ || !candidate_path_) {
      response->success = false;
      response->message = "coverage_handoff_no_valid_plan";
      return;
    }

    if (coverage::is_active(mission_.state())) {
      response->success = false;
      response->message = "coverage_handoff_mission_active";
      return;
    }

    if (coverage::is_terminal(mission_.state())) {
      const auto reset = mission_.reset();
      if (!reset.accepted) {
        response->success = false;
        response->message = reset.reason;
        return;
      }
      clear_runtime_dispatch();
    }

    if (mission_.state() !=
      coverage::CoverageMissionState::kIdle)
    {
      response->success = false;
      response->message = "coverage_handoff_not_idle";
      return;
    }

    const auto created_at_ns = next_event_time_ns();
    const auto mission_id =
      coverage::make_coverage_mission_id(
      mission_id_prefix_,
      candidate_generation_,
      created_at_ns);
    if (mission_id.empty()) {
      response->success = false;
      response->message = "coverage_handoff_mission_id_invalid";
      return;
    }

    active_path_ = *candidate_path_;
    active_candidate_generation_ = candidate_generation_;
    active_result_code_.reset();
    active_terminal_state_.clear();
    active_result_reason_.clear();

    const auto loaded = mission_.load_plan(
      coverage::CoverageMissionPlan{
        mission_id,
        static_cast<std::uint32_t>(
          active_path_->poses.size()),
        candidate_total_distance_m_,
        created_at_ns});
    if (!loaded.accepted) {
      active_path_.reset();
      response->success = false;
      response->message = loaded.reason;
      return;
    }

    const auto dispatch = mission_.request_dispatch(
      next_event_time_ns());
    if (!dispatch.accepted) {
      static_cast<void>(
        mission_.mark_failed(
          next_event_time_ns(),
          dispatch.reason));
      response->success = false;
      response->message = dispatch.reason;
      publish_state();
      publish_status();
      return;
    }

    ++dispatch_generation_;
    pending_started_at_ =
      std::chrono::steady_clock::now();
    goal_sent_at_.reset();
    cancel_started_at_.reset();
    current_goal_handle_.reset();
    goal_sent_ = false;
    goal_accepted_ = false;
    cancel_requested_ = false;
    cancel_acknowledged_ = false;
    cancel_timeout_reported_ = false;
    response_timeout_reported_ = false;
    cancellation_reason_.clear();

    response->success = true;
    response->message = mission_id;

    publish_state();
    publish_status();
  }

  void handle_cancel(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (!coverage::is_active(mission_.state())) {
      response->success = false;
      response->message = "coverage_handoff_no_active_mission";
      return;
    }

    if (cancel_requested_) {
      response->success = true;
      response->message = "coverage_handoff_cancel_already_requested";
      return;
    }

    cancel_requested_ = true;
    cancellation_reason_ = "operator_cancel_requested";

    if (!goal_sent_) {
      const auto canceled = mission_.request_cancel(
        next_event_time_ns(),
        cancellation_reason_);
      if (!canceled.accepted) {
        cancel_requested_ = false;
        response->success = false;
        response->message = canceled.reason;
        return;
      }
      clear_runtime_dispatch();
      response->success = true;
      response->message = "coverage_handoff_canceled_before_send";
      publish_state();
      publish_status();
      return;
    }

    if (!current_goal_handle_) {
      response->success = true;
      response->message =
        "coverage_handoff_cancel_waiting_for_goal_response";
      publish_status();
      return;
    }

    const auto transition = mission_.request_cancel(
      next_event_time_ns(),
      cancellation_reason_);
    if (!transition.accepted) {
      cancel_requested_ = false;
      response->success = false;
      response->message = transition.reason;
      return;
    }

    send_cancel_request();
    response->success = true;
    response->message = "coverage_handoff_cancel_sent";
    publish_state();
    publish_status();
  }

  void handle_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (coverage::is_active(mission_.state())) {
      response->success = false;
      response->message = "coverage_handoff_cannot_reset_active_mission";
      return;
    }

    if (mission_.state() ==
      coverage::CoverageMissionState::kIdle)
    {
      response->success = true;
      response->message = "coverage_handoff_already_idle";
      return;
    }

    const auto reset = mission_.reset();
    response->success = reset.accepted;
    response->message = reset.reason;
    if (reset.accepted) {
      clear_runtime_dispatch();
      active_path_.reset();
      active_candidate_generation_ = 0U;
      active_result_code_.reset();
      active_terminal_state_.clear();
      active_result_reason_.clear();
    }
    publish_state();
    publish_status();
  }

  void watchdog_tick()
  {
    if (!enabled_ ||
      mission_.state() !=
      coverage::CoverageMissionState::kAwaitingDispatch)
    {
      check_cancel_timeout();
      return;
    }

    if (!goal_sent_) {
      if (action_client_->action_server_is_ready()) {
        send_active_goal();
        return;
      }

      if (pending_started_at_.has_value() &&
        elapsed_seconds(*pending_started_at_) >=
        policy_.server_wait_timeout_sec)
      {
        static_cast<void>(
          mission_.mark_timed_out(
            next_event_time_ns(),
            "savo_nav_coverage_action_unavailable"));
        clear_runtime_dispatch();
        publish_state();
        publish_status();
      }
      return;
    }

    if (!goal_accepted_ &&
      goal_sent_at_.has_value() &&
      elapsed_seconds(*goal_sent_at_) >=
      policy_.goal_response_timeout_sec &&
      !response_timeout_reported_)
    {
      response_timeout_reported_ = true;
      cancel_requested_ = true;
      cancellation_reason_ =
        "savo_nav_coverage_goal_response_timeout";
      RCLCPP_ERROR(
        get_logger(),
        "Coverage goal response timeout; retaining mission "
        "ownership until a response arrives: mission_id=%s",
        mission_.plan().mission_id.c_str());
      publish_status();
    }

    check_cancel_timeout();
  }

  double elapsed_seconds(
    const std::chrono::steady_clock::time_point & start) const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count();
  }


  double candidate_age_seconds() const
  {
    if (!candidate_received_at_.has_value()) {
      return 0.0;
    }
    return std::max(
      0.0,
      elapsed_seconds(*candidate_received_at_));
  }

  void check_cancel_timeout()
  {
    if (mission_.state() !=
      coverage::CoverageMissionState::kCanceling ||
      !cancel_started_at_.has_value() ||
      cancel_timeout_reported_)
    {
      return;
    }

    const double elapsed =
      elapsed_seconds(*cancel_started_at_);
    if (elapsed < policy_.cancel_timeout_sec) {
      return;
    }

    cancel_timeout_reported_ = true;
    RCLCPP_ERROR(
      get_logger(),
      "Coverage cancellation has not reached a terminal result "
      "after %.2f seconds; retaining mission ownership: "
      "mission_id=%s acknowledged=%s",
      elapsed,
      mission_.plan().mission_id.c_str(),
      bool_text(cancel_acknowledged_));
    publish_status();
  }

  void send_active_goal()
  {
    if (!active_path_ || goal_sent_ ||
      mission_.state() !=
      coverage::CoverageMissionState::kAwaitingDispatch)
    {
      return;
    }

    ExecuteCoveragePath::Goal goal;
    goal.contract_version =
      ExecuteCoveragePath::Goal::CONTRACT_VERSION;
    goal.mission_id = mission_.plan().mission_id;
    goal.path = *active_path_;
    goal.execution_timeout = duration_from_seconds(
      policy_.execution_timeout_sec);

    const auto generation = dispatch_generation_;
    const auto mission_id = mission_.plan().mission_id;
    ActionClient::SendGoalOptions options;
    options.goal_response_callback =
      [this, generation, mission_id](
      const GoalHandle::SharedPtr & goal_handle)
      {
        handle_goal_response(
          generation,
          mission_id,
          goal_handle);
      };
    options.feedback_callback =
      [this, generation, mission_id](
      GoalHandle::SharedPtr,
      const std::shared_ptr<
        const ExecuteCoveragePath::Feedback> feedback)
      {
        handle_feedback(
          generation,
          mission_id,
          feedback);
      };
    options.result_callback =
      [this, generation, mission_id](
      const GoalHandle::WrappedResult & result)
      {
        handle_result(
          generation,
          mission_id,
          result);
      };

    goal_sent_ = true;
    goal_sent_at_ = std::chrono::steady_clock::now();
    try {
      static_cast<void>(
        action_client_->async_send_goal(goal, options));
      RCLCPP_INFO(
        get_logger(),
        "Approved Coverage path sent to savo_nav: mission_id=%s waypoints=%zu",
        mission_id.c_str(),
        active_path_->poses.size());
    } catch (const std::exception & error) {
      goal_sent_ = false;
      static_cast<void>(
        mission_.mark_failed(
          next_event_time_ns(),
          std::string{"coverage_goal_send_exception:"} +
          error.what()));
      clear_runtime_dispatch();
      publish_state();
      publish_status();
    }
  }

  void handle_goal_response(
    const std::uint64_t generation,
    const std::string & mission_id,
    const GoalHandle::SharedPtr & goal_handle)
  {
    if (!callback_matches(generation, mission_id) ||
      coverage::is_terminal(mission_.state()))
    {
      return;
    }

    if (!goal_handle) {
      if (cancel_requested_ &&
        cancellation_reason_ ==
        "savo_nav_coverage_goal_response_timeout")
      {
        static_cast<void>(
          mission_.mark_timed_out(
            next_event_time_ns(),
            cancellation_reason_));
      } else if (cancel_requested_) {
        static_cast<void>(
          mission_.request_cancel(
            next_event_time_ns(),
            cancellation_reason_));
      } else {
        static_cast<void>(
          mission_.mark_rejected(
            next_event_time_ns(),
            "savo_nav_coverage_goal_rejected"));
      }
      clear_runtime_dispatch();
      publish_state();
      publish_status();
      return;
    }

    current_goal_handle_ = goal_handle;
    goal_accepted_ = true;
    const auto accepted = mission_.mark_accepted(
      next_event_time_ns());
    if (!accepted.accepted) {
      RCLCPP_ERROR(
        get_logger(),
        "Coverage acceptance transition rejected: %s",
        accepted.reason.c_str());
      cancel_requested_ = true;
      cancellation_reason_ =
        "coverage_handoff_acceptance_state_error";
      send_cancel_request_without_transition();
      publish_status();
      return;
    }

    if (cancel_requested_) {
      const auto canceling = mission_.request_cancel(
        next_event_time_ns(),
        cancellation_reason_);
      if (canceling.accepted) {
        send_cancel_request();
      }
    }
    publish_state();
    publish_status();
  }

  void handle_feedback(
    const std::uint64_t generation,
    const std::string & mission_id,
    const std::shared_ptr<
      const ExecuteCoveragePath::Feedback> & feedback)
  {
    if (!feedback ||
      !callback_matches(generation, mission_id) ||
      (mission_.state() !=
      coverage::CoverageMissionState::kExecuting &&
      mission_.state() !=
      coverage::CoverageMissionState::kCanceling))
    {
      return;
    }

    const double completed_distance_m = std::clamp(
      mission_.plan().total_distance_m -
      feedback->remaining_distance_m,
      0.0,
      mission_.plan().total_distance_m);
    static_cast<void>(
      mission_.update_progress(
        mission_id,
        feedback->current_waypoint,
        feedback->completed_waypoints,
        completed_distance_m,
        next_event_time_ns()));

    std::ostringstream output;
    output.precision(17);
    output
      << "{\"mission_id\":\""
      << json_escape(mission_id)
      << "\",\"state\":\""
      << json_escape(feedback->state_text)
      << "\",\"reason\":\""
      << json_escape(feedback->reason)
      << "\",\"current_waypoint\":"
      << feedback->current_waypoint
      << ",\"completed_waypoints\":"
      << feedback->completed_waypoints
      << ",\"total_waypoints\":"
      << feedback->total_waypoints
      << ",\"completion_ratio\":"
      << feedback->completion_ratio
      << ",\"remaining_distance_m\":"
      << feedback->remaining_distance_m
      << "}";
    std_msgs::msg::String message;
    message.data = output.str();
    feedback_publisher_->publish(message);
    publish_status();
  }

  void handle_result(
    const std::uint64_t generation,
    const std::string & mission_id,
    const GoalHandle::WrappedResult & wrapped)
  {
    if (!callback_matches(generation, mission_id) ||
      coverage::is_terminal(mission_.state()))
    {
      return;
    }

    if (!wrapped.result) {
      static_cast<void>(
        mission_.mark_failed(
          next_event_time_ns(),
          "savo_nav_coverage_result_missing"));
      clear_runtime_dispatch();
      publish_state();
      publish_status();
      return;
    }

    active_result_code_ = wrapped.result->result_code;
    active_terminal_state_ =
      wrapped.result->terminal_state;
    active_result_reason_ =
      wrapped.result->reason;

    const auto result_code =
      wrapped.result->result_code;
    const auto event_time_ns = next_event_time_ns();
    if (wrapped.result->success &&
      result_code ==
      ExecuteCoveragePath::Result::RESULT_SUCCEEDED)
    {
      static_cast<void>(
        mission_.mark_succeeded(event_time_ns));
    } else if (result_code ==
      ExecuteCoveragePath::Result::RESULT_CANCELED ||
      wrapped.code == rclcpp_action::ResultCode::CANCELED)
    {
      static_cast<void>(
        mission_.mark_canceled(
          event_time_ns,
          result_reason_or(
            "savo_nav_coverage_canceled")));
    } else if (result_code ==
      ExecuteCoveragePath::Result::RESULT_TIMED_OUT ||
      result_code ==
      ExecuteCoveragePath::Result::RESULT_FEEDBACK_STALE)
    {
      static_cast<void>(
        mission_.mark_timed_out(
          event_time_ns,
          result_reason_or(
            "savo_nav_coverage_timed_out")));
    } else {
      static_cast<void>(
        mission_.mark_failed(
          event_time_ns,
          result_reason_or(
            "savo_nav_coverage_failed")));
    }

    clear_runtime_dispatch();
    publish_state();
    publish_status();
  }

  std::string result_reason_or(
    const std::string & fallback) const
  {
    return active_result_reason_.empty() ?
           fallback :
           active_result_reason_;
  }

  bool callback_matches(
    const std::uint64_t generation,
    const std::string & mission_id) const
  {
    return generation == dispatch_generation_ &&
           mission_id == mission_.plan().mission_id;
  }

  void send_cancel_request()
  {
    if (!current_goal_handle_) {
      return;
    }
    cancel_started_at_ =
      std::chrono::steady_clock::now();
    cancel_acknowledged_ = false;
    cancel_timeout_reported_ = false;
    send_cancel_request_without_transition();
  }

  void send_cancel_request_without_transition()
  {
    if (!current_goal_handle_) {
      return;
    }

    const auto generation = dispatch_generation_;
    const auto mission_id = mission_.plan().mission_id;
    try {
      action_client_->async_cancel_goal(
        current_goal_handle_,
        [this, generation, mission_id](
          const CancelResponse::SharedPtr response)
        {
          handle_cancel_response(
            generation,
            mission_id,
            response);
        });
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(),
        "Coverage cancel request failed: %s",
        error.what());
    }
  }

  void handle_cancel_response(
    const std::uint64_t generation,
    const std::string & mission_id,
    const CancelResponse::SharedPtr response)
  {
    if (!callback_matches(generation, mission_id) ||
      coverage::is_terminal(mission_.state()))
    {
      return;
    }

    cancel_acknowledged_ =
      response &&
      response->return_code ==
      CancelResponse::ERROR_NONE &&
      !response->goals_canceling.empty();
    if (!cancel_acknowledged_) {
      RCLCPP_ERROR(
        get_logger(),
        "savo_nav did not acknowledge Coverage cancellation; "
        "retaining mission ownership: mission_id=%s",
        mission_id.c_str());
    }
    publish_status();
  }

  void clear_runtime_dispatch()
  {
    pending_started_at_.reset();
    goal_sent_at_.reset();
    cancel_started_at_.reset();
    current_goal_handle_.reset();
    goal_sent_ = false;
    goal_accepted_ = false;
    cancel_requested_ = false;
    cancel_acknowledged_ = false;
    cancel_timeout_reported_ = false;
    response_timeout_reported_ = false;
    cancellation_reason_.clear();
  }

  std::string effective_state() const
  {
    if (!enabled_) {
      return "disabled";
    }
    if (mission_.state() !=
      coverage::CoverageMissionState::kIdle)
    {
      return coverage::to_string(mission_.state());
    }
    if (candidate_valid_ && candidate_path_) {
      return "plan_available";
    }
    if (candidate_generation_ > 0U) {
      return "plan_invalid";
    }
    return "waiting_for_plan";
  }

  void publish_state()
  {
    const auto state = effective_state();
    if (state == last_published_state_) {
      return;
    }
    last_published_state_ = state;
    std_msgs::msg::String message;
    message.data = state;
    state_publisher_->publish(message);
  }

  void publish_status()
  {
    std::ostringstream output;
    output.precision(17);
    output
      << "{\"enabled\":" << bool_text(enabled_)
      << ",\"state\":\""
      << json_escape(effective_state())
      << "\",\"reason\":\""
      << json_escape(mission_.reason())
      << "\",\"candidate_valid\":"
      << bool_text(candidate_valid_)
      << ",\"candidate_reason\":\""
      << json_escape(candidate_reason_)
      << "\",\"candidate_generation\":"
      << candidate_generation_
      << ",\"candidate_waypoints\":"
      << (candidate_path_ ?
      candidate_path_->poses.size() : 0U)
      << ",\"candidate_distance_m\":"
      << candidate_total_distance_m_
      << ",\"candidate_age_sec\":"
      << candidate_age_seconds()
      << ",\"active_candidate_generation\":"
      << active_candidate_generation_
      << ",\"mission_id\":\""
      << json_escape(mission_.plan().mission_id)
      << "\",\"mission_sequence\":"
      << mission_.sequence()
      << ",\"goal_sent\":"
      << bool_text(goal_sent_)
      << ",\"goal_accepted\":"
      << bool_text(goal_accepted_)
      << ",\"cancel_requested\":"
      << bool_text(cancel_requested_)
      << ",\"cancel_acknowledged\":"
      << bool_text(cancel_acknowledged_)
      << ",\"response_timeout_reported\":"
      << bool_text(response_timeout_reported_)
      << ",\"cancel_timeout_reported\":"
      << bool_text(cancel_timeout_reported_)
      << ",\"current_waypoint\":"
      << mission_.progress().current_waypoint
      << ",\"completed_waypoints\":"
      << mission_.progress().completed_waypoints
      << ",\"total_waypoints\":"
      << mission_.plan().total_waypoints
      << ",\"completion_ratio\":"
      << mission_.progress().completion_ratio
      << ",\"remaining_distance_m\":"
      << mission_.progress().remaining_distance_m
      << ",\"result_code\":";
    if (active_result_code_.has_value()) {
      output << static_cast<unsigned int>(
        *active_result_code_);
    } else {
      output << "null";
    }
    output
      << ",\"terminal_state\":\""
      << json_escape(active_terminal_state_)
      << "\",\"result_reason\":\""
      << json_escape(active_result_reason_)
      << "\"}";

    std_msgs::msg::String message;
    message.data = output.str();
    status_publisher_->publish(message);
  }

  bool enabled_{true};
  std::string expected_frame_;
  std::string mission_id_prefix_;
  std::string plan_topic_;
  std::string action_name_;
  std::string state_topic_;
  std::string status_topic_;
  std::string feedback_topic_;
  std::string approve_service_name_;
  std::string cancel_service_name_;
  std::string reset_service_name_;
  coverage::CoverageExecutionHandoffPolicy policy_;
  std::int64_t watchdog_period_ms_{100};

  coverage::CoverageMission mission_;
  std::int64_t last_event_time_ns_{-1};
  std::uint64_t candidate_generation_{0};
  std::uint64_t active_candidate_generation_{0};
  std::uint64_t dispatch_generation_{0};
  bool candidate_valid_{false};
  std::string candidate_reason_{"waiting_for_plan"};
  double candidate_total_distance_m_{0.0};
  std::optional<nav_msgs::msg::Path> candidate_path_;
  std::optional<nav_msgs::msg::Path> active_path_;
  std::optional<std::chrono::steady_clock::time_point>
    candidate_received_at_;
  std::optional<std::chrono::steady_clock::time_point>
    pending_started_at_;
  std::optional<std::chrono::steady_clock::time_point>
    goal_sent_at_;
  std::optional<std::chrono::steady_clock::time_point>
    cancel_started_at_;
  bool goal_sent_{false};
  bool goal_accepted_{false};
  bool cancel_requested_{false};
  bool cancel_acknowledged_{false};
  bool cancel_timeout_reported_{false};
  bool response_timeout_reported_{false};
  std::string cancellation_reason_;
  std::optional<std::uint8_t> active_result_code_;
  std::string active_terminal_state_;
  std::string active_result_reason_;
  std::string last_published_state_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    feedback_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
    plan_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    approve_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    cancel_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    reset_service_;
  ActionClient::SharedPtr action_client_;
  GoalHandle::SharedPtr current_goal_handle_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<
        savo_mapping::CoverageExecutionHandoffNode>());
  } catch (const std::exception & error) {
    std::cerr
      << "coverage_execution_handoff_node failed: "
      << error.what()
      << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
