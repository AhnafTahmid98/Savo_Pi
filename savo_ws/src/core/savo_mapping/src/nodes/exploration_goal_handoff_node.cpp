#include "savo_mapping/exploration_goal_handoff.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace savo_mapping
{

class ExplorationGoalHandoffNode final
  : public rclcpp::Node
{
public:
  using NavigateToPose =
    nav2_msgs::action::NavigateToPose;

  using ActionClient =
    rclcpp_action::Client<
      NavigateToPose>;

  using GoalHandle =
    ActionClient::GoalHandle;

  using CancelResponse =
    ActionClient::CancelResponse;

  ExplorationGoalHandoffNode()
  : Node("exploration_goal_handoff_node")
  {
    expected_frame_ =
      declare_parameter<std::string>(
      "expected_frame",
      "map");

    server_wait_timeout_sec_ =
      declare_parameter<double>(
      "server_wait_timeout_sec",
      3.0);

    goal_response_timeout_sec_ =
      declare_parameter<double>(
      "goal_response_timeout_sec",
      3.0);

    execution_timeout_sec_ =
      declare_parameter<double>(
      "execution_timeout_sec",
      300.0);

    feedback_stale_timeout_sec_ =
      declare_parameter<double>(
      "feedback_stale_timeout_sec",
      10.0);

    cancel_timeout_sec_ =
      declare_parameter<double>(
      "cancel_timeout_sec",
      5.0);

    restamp_goal_ =
      declare_parameter<bool>(
      "restamp_goal",
      true);

    const std::int64_t poll_period_ms =
      declare_parameter<std::int64_t>(
      "server_poll_period_ms",
      100);

    if (expected_frame_.empty()) {
      throw std::invalid_argument(
              "expected_frame_empty");
    }

    if (!std::isfinite(
        server_wait_timeout_sec_) ||
        server_wait_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "server_wait_timeout_must_be_positive");
    }

    if (!std::isfinite(
        goal_response_timeout_sec_) ||
        goal_response_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "goal_response_timeout_must_be_positive");
    }

    if (!std::isfinite(
        execution_timeout_sec_) ||
        execution_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "execution_timeout_must_be_positive");
    }

    if (!std::isfinite(
        feedback_stale_timeout_sec_) ||
        feedback_stale_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "feedback_stale_timeout_must_be_positive");
    }

    if (!std::isfinite(
        cancel_timeout_sec_) ||
        cancel_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "cancel_timeout_must_be_positive");
    }

    if (poll_period_ms < 20 ||
        poll_period_ms > 1000)
    {
      throw std::invalid_argument(
              "server_poll_period_out_of_range");
    }

    const auto state_qos =
      rclcpp::QoS(
      rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

    state_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        exploration::kGoalStateTopic,
        state_qos);

    status_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        exploration::kGoalStatusTopic,
        state_qos);

    feedback_publisher_ =
      create_publisher<
        std_msgs::msg::String>(
        exploration::kGoalFeedbackTopic,
        rclcpp::QoS(10).reliable());

    selected_goal_subscription_ =
      create_subscription<
        geometry_msgs::msg::PoseStamped>(
        exploration::kSelectedGoalTopic,
        rclcpp::QoS(1).reliable(),
        std::bind(
          &ExplorationGoalHandoffNode::
          handle_selected_goal,
          this,
          std::placeholders::_1));

    cancel_service_ =
      create_service<
        std_srvs::srv::Trigger>(
        exploration::kGoalCancelService,
        std::bind(
          &ExplorationGoalHandoffNode::
          handle_cancel,
          this,
          std::placeholders::_1,
          std::placeholders::_2));

    action_client_ =
      rclcpp_action::create_client<
        NavigateToPose>(
        this,
        exploration::kExplorationActionName);

    watchdog_timer_ =
      create_wall_timer(
        std::chrono::milliseconds(
          poll_period_ms),
        std::bind(
          &ExplorationGoalHandoffNode::
          check_handoff_watchdog,
          this));

    publish_state();
    publish_status(true);

    RCLCPP_INFO(
      get_logger(),
      "exploration goal handoff ready");

    RCLCPP_INFO(
      get_logger(),
      "selected goals are forwarded only to %s",
      exploration::kExplorationActionName);

    RCLCPP_INFO(
      get_logger(),
      "node does not publish cmd_vel or call "
      "Nav2 /navigate_to_pose directly");
  }

private:
  static std::string escape_json(
    const std::string & value)
  {
    std::ostringstream output;

    for (const char character : value) {
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

  static bool finite(
    const double value)
  {
    return std::isfinite(value);
  }

  bool prepare_goal(
    geometry_msgs::msg::PoseStamped & pose,
    std::string & reason)
  {
    if (pose.header.frame_id.empty()) {
      reason = "goal_frame_empty";
      return false;
    }

    if (pose.header.frame_id !=
        expected_frame_)
    {
      reason =
        "goal_frame_mismatch:" +
        pose.header.frame_id;

      return false;
    }

    const auto & position =
      pose.pose.position;

    auto & orientation =
      pose.pose.orientation;

    if (!finite(position.x) ||
        !finite(position.y) ||
        !finite(position.z))
    {
      reason =
        "goal_position_not_finite";

      return false;
    }

    if (!finite(orientation.x) ||
        !finite(orientation.y) ||
        !finite(orientation.z) ||
        !finite(orientation.w))
    {
      reason =
        "goal_orientation_not_finite";

      return false;
    }

    const double norm =
      std::sqrt(
        orientation.x * orientation.x +
        orientation.y * orientation.y +
        orientation.z * orientation.z +
        orientation.w * orientation.w);

    if (norm < 1.0e-9) {
      reason =
        "goal_orientation_zero_norm";

      return false;
    }

    orientation.x /= norm;
    orientation.y /= norm;
    orientation.z /= norm;
    orientation.w /= norm;

    if (restamp_goal_) {
      pose.header.stamp =
        now();
    }

    reason = "goal_valid";
    return true;
  }

  void handle_selected_goal(
    const geometry_msgs::msg::
    PoseStamped::SharedPtr message)
  {
    geometry_msgs::msg::PoseStamped pose =
      *message;

    std::string validation_reason;

    if (!prepare_goal(
        pose,
        validation_reason))
    {
      publish_rejection(
        validation_reason);

      RCLCPP_WARN(
        get_logger(),
        "selected exploration goal rejected: %s",
        validation_reason.c_str());

      return;
    }

    const std::string request_id =
      "frontier-" +
      std::to_string(
        machine_.sequence() + 1U);

    const auto transition =
      machine_.begin(request_id);

    if (!transition.accepted) {
      publish_rejection(
        transition.reason);

      RCLCPP_WARN(
        get_logger(),
        "selected exploration goal rejected: %s",
        transition.reason.c_str());

      return;
    }

    clear_runtime_goal();

    pending_goal_ = pose;
    pending_started_ = now();

    publish_transition(
      transition);

    RCLCPP_INFO(
      get_logger(),
      "exploration goal queued: "
      "request_id=%s x=%.3f y=%.3f",
      request_id.c_str(),
      pose.pose.position.x,
      pose.pose.position.y);
  }

  void check_handoff_watchdog()
  {
    const auto state =
      machine_.state();

    const auto current_time =
      now();

    if (state ==
        exploration::
        GoalHandoffState::
        kWaitingForServer)
    {
      if (!pending_goal_.has_value() ||
          !pending_started_.has_value())
      {
        publish_transition(
          machine_.mark_error(
            "pending_goal_missing"));

        clear_runtime_goal();
        return;
      }

      if (action_client_->
          action_server_is_ready())
      {
        send_pending_goal();
        return;
      }

      const double elapsed =
        (
          current_time -
          pending_started_.value()
        ).seconds();

      if (elapsed >=
          server_wait_timeout_sec_)
      {
        publish_transition(
          machine_.mark_timed_out(
            "savo_nav_action_server_unavailable"));

        clear_runtime_goal();

        RCLCPP_WARN(
          get_logger(),
          "savo_nav exploration action server "
          "was unavailable for %.2f seconds",
          elapsed);
      }

      return;
    }

    if (state ==
        exploration::
        GoalHandoffState::kSending)
    {
      if (!goal_sent_at_.has_value()) {
        publish_transition(
          machine_.mark_error(
            "goal_send_timestamp_missing"));

        clear_runtime_goal();
        return;
      }

      const double elapsed =
        (
          current_time -
          goal_sent_at_.value()
        ).seconds();

      if (elapsed >=
          goal_response_timeout_sec_)
      {
        publish_transition(
          machine_.mark_timed_out(
            "savo_nav_goal_response_timeout"));

        clear_runtime_goal();

        RCLCPP_WARN(
          get_logger(),
          "savo_nav goal response timed out "
          "after %.2f seconds",
          elapsed);
      }

      return;
    }

    if (state ==
        exploration::
        GoalHandoffState::kAccepted ||
        state ==
        exploration::
        GoalHandoffState::kExecuting)
    {
      if (!goal_accepted_at_.has_value() ||
          !last_feedback_at_.has_value())
      {
        publish_transition(
          machine_.mark_error(
            "active_goal_timestamp_missing"));

        clear_runtime_goal();
        return;
      }

      const double execution_elapsed =
        (
          current_time -
          goal_accepted_at_.value()
        ).seconds();

      if (execution_elapsed >=
          execution_timeout_sec_)
      {
        timeout_active_goal(
          "savo_nav_execution_timeout",
          execution_elapsed);

        return;
      }

      const double feedback_elapsed =
        (
          current_time -
          last_feedback_at_.value()
        ).seconds();

      if (feedback_elapsed >=
          feedback_stale_timeout_sec_)
      {
        timeout_active_goal(
          "savo_nav_feedback_stale",
          feedback_elapsed);
      }

      return;
    }

    if (state ==
        exploration::
        GoalHandoffState::kCanceling)
    {
      if (!cancel_started_at_.has_value()) {
        publish_transition(
          machine_.mark_error(
            "cancel_timestamp_missing"));

        clear_runtime_goal();
        return;
      }

      const double elapsed =
        (
          current_time -
          cancel_started_at_.value()
        ).seconds();

      if (elapsed >=
          cancel_timeout_sec_ &&
          !cancel_timeout_reported_)
      {
        cancel_timeout_reported_ = true;

        RCLCPP_ERROR(
          get_logger(),
          "savo_nav cancellation has not reached "
          "a terminal result after %.2f seconds; "
          "keeping the exploration goal active "
          "until terminal confirmation arrives; "
          "cancel_acknowledged=%s",
          elapsed,
          cancel_acknowledged_ ?
          "true" :
          "false");
      }
    }
  }
  static std::string cancel_response_reason(
    const std::int8_t return_code,
    const bool has_canceling_goal)
  {
    if (return_code ==
        CancelResponse::ERROR_NONE)
    {
      return has_canceling_goal ?
        "" :
        "savo_nav_cancel_ack_missing_goal";
    }

    if (return_code ==
        CancelResponse::ERROR_REJECTED)
    {
      return "savo_nav_cancel_rejected";
    }

    if (return_code ==
        CancelResponse::ERROR_UNKNOWN_GOAL_ID)
    {
      return "savo_nav_cancel_unknown_goal";
    }

    if (return_code ==
        CancelResponse::ERROR_GOAL_TERMINATED)
    {
      return "savo_nav_cancel_goal_already_terminal";
    }

    return
      std::string{
      "savo_nav_cancel_response_code_"} +
      std::to_string(
        static_cast<int>(
          return_code));
  }

  void handle_active_cancel_response(
    const std::string & request_id,
    const CancelResponse::SharedPtr response)
  {
    if (request_id !=
        machine_.request_id())
    {
      RCLCPP_WARN(
        get_logger(),
        "ignoring stale cancel acknowledgement: "
        "request_id=%s current_request_id=%s",
        request_id.c_str(),
        machine_.request_id().c_str());

      return;
    }

    if (exploration::is_terminal(
        machine_.state()))
    {
      RCLCPP_INFO(
        get_logger(),
        "cancel acknowledgement arrived after "
        "terminal result: request_id=%s",
        request_id.c_str());

      return;
    }

    if (machine_.state() !=
        exploration::
        GoalHandoffState::kCanceling)
    {
      RCLCPP_WARN(
        get_logger(),
        "cancel acknowledgement arrived in "
        "unexpected state: request_id=%s state=%s",
        request_id.c_str(),
        exploration::to_string(
          machine_.state()).c_str());

      return;
    }

    if (!response) {
      cancel_acknowledged_ = false;

      publish_status(
        false,
        "savo_nav_cancel_response_missing");

      RCLCPP_ERROR(
        get_logger(),
        "savo_nav returned no cancel response; "
        "keeping exploration goal active: "
        "request_id=%s",
        request_id.c_str());

      return;
    }

    const bool has_canceling_goal =
      !response->goals_canceling.empty();

    const std::string reason =
      cancel_response_reason(
        response->return_code,
        has_canceling_goal);

    if (!reason.empty()) {
      cancel_acknowledged_ = false;

      publish_status(
        false,
        reason);

      RCLCPP_ERROR(
        get_logger(),
        "savo_nav did not acknowledge cancellation: "
        "request_id=%s return_code=%d "
        "goals_canceling=%zu reason=%s; "
        "keeping exploration goal active",
        request_id.c_str(),
        static_cast<int>(
          response->return_code),
        response->goals_canceling.size(),
        reason.c_str());

      return;
    }

    cancel_acknowledged_ = true;

    publish_status(true);

    RCLCPP_INFO(
      get_logger(),
      "savo_nav acknowledged cancellation: "
      "request_id=%s goals_canceling=%zu",
      request_id.c_str(),
      response->goals_canceling.size());
  }

  void send_active_cancel_request(
    const std::string & request_id)
  {
    const auto goal_handle =
      current_goal_handle_;

    if (!goal_handle) {
      throw std::runtime_error(
              "accepted_goal_handle_missing");
    }

    action_client_->async_cancel_goal(
      goal_handle,
      [this, request_id](
        const CancelResponse::SharedPtr response)
      {
        handle_active_cancel_response(
          request_id,
          response);
      });
  }

  void timeout_active_goal(
    const std::string & reason,
    const double elapsed)
  {
    if (timeout_reason_.has_value()) {
      return;
    }

    if (!current_goal_handle_) {
      RCLCPP_ERROR(
        get_logger(),
        "cannot cancel timed-out exploration goal: "
        "goal handle is missing");

      return;
    }

    const auto transition =
      machine_.request_cancel(
      reason);

    if (!transition.accepted) {
      RCLCPP_ERROR(
        get_logger(),
        "timeout cancellation transition rejected: %s",
        transition.reason.c_str());

      return;
    }

    timeout_reason_ =
      reason;

    cancel_started_at_ =
      now();

    cancel_timeout_reported_ = false;
    cancel_acknowledged_ = false;

    publish_transition(
      transition);

    try {
      send_active_cancel_request(
        machine_.request_id());
    } catch (const std::exception & exception) {
      const std::string cancel_reason =
        std::string{
        "savo_nav_cancel_request_exception:"} +
        exception.what();

      publish_status(
        false,
        cancel_reason);

      RCLCPP_ERROR(
        get_logger(),
        "failed to request cancellation for "
        "timed-out exploration goal: %s; "
        "keeping exploration goal active",
        exception.what());
    }

    RCLCPP_WARN(
      get_logger(),
      "exploration goal timeout detected: "
      "reason=%s elapsed=%.2f; "
      "waiting for savo_nav terminal result",
      reason.c_str(),
      elapsed);
  }

  void send_pending_goal()
  {
    const auto server_transition =
      machine_.mark_server_available();

    if (!server_transition.accepted) {
      publish_rejection(
        server_transition.reason);

      return;
    }

    publish_transition(
      server_transition);

    NavigateToPose::Goal goal;

    goal.pose =
      pending_goal_.value();

    goal.behavior_tree = "";

    const std::string request_id =
      machine_.request_id();

    typename rclcpp_action::Client<
      NavigateToPose>::
      SendGoalOptions options;

    options.goal_response_callback =
      [this, request_id](
        const GoalHandle::SharedPtr &
        goal_handle)
      {
        handle_goal_response(
          request_id,
          goal_handle);
      };

    options.feedback_callback =
      [this, request_id](
        GoalHandle::SharedPtr,
        const std::shared_ptr<
          const NavigateToPose::Feedback>
          feedback)
      {
        handle_feedback(
          request_id,
          feedback);
      };

    options.result_callback =
      [this, request_id](
        const GoalHandle::WrappedResult &
        result)
      {
        handle_result(
          request_id,
          result);
      };

    goal_sent_at_ =
      now();

    try {
      action_client_->async_send_goal(
        goal,
        options);

      pending_goal_.reset();
      pending_started_.reset();

      RCLCPP_INFO(
        get_logger(),
        "exploration goal sent to savo_nav: %s",
        request_id.c_str());
    } catch (const std::exception & exception) {
      publish_transition(
        machine_.mark_error(
          std::string{
          "goal_send_exception:"} +
          exception.what()));

      clear_runtime_goal();
    }
  }

  void handle_goal_response(
    const std::string & request_id,
    const GoalHandle::SharedPtr &
    goal_handle)
  {
    if (request_id !=
        machine_.request_id())
    {
      if (goal_handle) {
        action_client_->
        async_cancel_goal(
          goal_handle);
      }

      return;
    }

    if (exploration::is_terminal(
        machine_.state()))
    {
      if (goal_handle) {
        action_client_->
        async_cancel_goal(
          goal_handle);
      }

      return;
    }

    if (!goal_handle) {
      publish_transition(
        machine_.mark_rejected(
          "savo_nav_rejected_goal"));

      clear_runtime_goal();

      RCLCPP_WARN(
        get_logger(),
        "savo_nav rejected exploration goal: %s",
        request_id.c_str());

      return;
    }

    current_goal_handle_ =
      goal_handle;

    const auto accepted_time =
      now();

    goal_sent_at_.reset();

    goal_accepted_at_ =
      accepted_time;

    last_feedback_at_ =
      accepted_time;

    cancel_started_at_.reset();

    publish_transition(
      machine_.mark_accepted());

    RCLCPP_INFO(
      get_logger(),
      "savo_nav accepted exploration goal: %s",
      request_id.c_str());
  }

  void handle_feedback(
    const std::string & request_id,
    const std::shared_ptr<
      const NavigateToPose::Feedback>
      feedback)
  {
    if (request_id !=
        machine_.request_id())
    {
      return;
    }

    if (machine_.state() ==
        exploration::
        GoalHandoffState::kAccepted ||
        machine_.state() ==
        exploration::
        GoalHandoffState::kExecuting ||
        machine_.state() ==
        exploration::
        GoalHandoffState::kCanceling)
    {
      last_feedback_at_ =
        now();
    }

    if (machine_.state() ==
        exploration::
        GoalHandoffState::kAccepted)
    {
      publish_transition(
        machine_.mark_executing());
    }

    if (machine_.state() !=
        exploration::
        GoalHandoffState::kExecuting &&
        machine_.state() !=
        exploration::
        GoalHandoffState::kCanceling)
    {
      return;
    }

    std::ostringstream output;

    output
      << "{"
      << "\"request_id\":\""
      << escape_json(request_id)
      << "\",\"state\":\""
      << exploration::to_string(
           machine_.state())
      << "\",\"distance_remaining\":"
      << feedback->distance_remaining
      << ",\"number_of_recoveries\":"
      << feedback->number_of_recoveries
      << ",\"current_x\":"
      << feedback->
         current_pose.pose.position.x
      << ",\"current_y\":"
      << feedback->
         current_pose.pose.position.y
      << ",\"navigation_time_sec\":"
      << feedback->navigation_time.sec
      << ",\"navigation_time_nanosec\":"
      << feedback->
         navigation_time.nanosec
      << "}";

    std_msgs::msg::String message;
    message.data = output.str();

    feedback_publisher_->publish(message);
  }

  void handle_result(
    const std::string & request_id,
    const GoalHandle::WrappedResult &
    wrapped_result)
  {
    if (request_id !=
        machine_.request_id())
    {
      return;
    }

    if (exploration::is_terminal(
        machine_.state()))
    {
      current_goal_handle_.reset();
      return;
    }

    if (timeout_reason_.has_value()) {
      const std::string timeout_reason =
        timeout_reason_.value();

      publish_transition(
        machine_.mark_timed_out(
          timeout_reason));

      RCLCPP_INFO(
        get_logger(),
        "timed-out exploration goal reached "
        "terminal savo_nav result: "
        "request_id=%s reason=%s result_code=%d",
        request_id.c_str(),
        timeout_reason.c_str(),
        static_cast<int>(
          wrapped_result.code));

      clear_runtime_goal();
      return;
    }

    switch (wrapped_result.code) {
      case rclcpp_action::
        ResultCode::SUCCEEDED:
      {
        if (wrapped_result.result &&
            wrapped_result.result->
            error_code != 0U)
        {
          publish_transition(
            machine_.mark_aborted(
              result_reason(
                "savo_nav_result_error",
                wrapped_result.result)));
        } else {
          publish_transition(
            machine_.mark_succeeded());
        }

        break;
      }

      case rclcpp_action::
        ResultCode::ABORTED:
      {
        publish_transition(
          machine_.mark_aborted(
            result_reason(
              "savo_nav_aborted",
              wrapped_result.result)));

        break;
      }

      case rclcpp_action::
        ResultCode::CANCELED:
      {
        publish_transition(
          machine_.mark_canceled(
            "savo_nav_canceled"));

        break;
      }

      case rclcpp_action::
        ResultCode::UNKNOWN:
      default:
      {
        publish_transition(
          machine_.mark_error(
            "savo_nav_result_unknown"));

        break;
      }
    }

    if (!exploration::is_terminal(
        machine_.state()))
    {
      const std::string failure_reason =
        std::string{
        "terminal_result_transition_failed:"} +
        exploration::to_string(
          machine_.state()) +
        ":result_code_" +
        std::to_string(
          static_cast<int>(
            wrapped_result.code));

      const auto error_transition =
        machine_.mark_error(
        failure_reason);

      if (error_transition.accepted) {
        publish_transition(
          error_transition);

        RCLCPP_ERROR(
          get_logger(),
          "terminal savo_nav result could not be "
          "applied to the handoff state machine: "
          "request_id=%s reason=%s; "
          "finalizing as error because the action "
          "is confirmed terminal",
          request_id.c_str(),
          failure_reason.c_str());

        clear_runtime_goal();
      } else {
        publish_rejection(
          error_transition.reason);

        RCLCPP_ERROR(
          get_logger(),
          "terminal savo_nav result could not be "
          "applied and error finalization was "
          "rejected: request_id=%s reason=%s; "
          "retaining runtime tracking",
          request_id.c_str(),
          error_transition.reason.c_str());
      }

      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "exploration goal finished: "
      "request_id=%s state=%s reason=%s",
      request_id.c_str(),
      exploration::to_string(
        machine_.state()).c_str(),
      machine_.reason().c_str());

    clear_runtime_goal();
  }

  static std::string result_reason(
    const std::string & prefix,
    const std::shared_ptr<
      const NavigateToPose::Result> &
      result)
  {
    if (!result) {
      return prefix +
        ":result_missing";
    }

    std::string reason =
      prefix +
      ":code=" +
      std::to_string(
        result->error_code);

    if (!result->error_msg.empty()) {
      reason +=
        ":" +
        result->error_msg;
    }

    return reason;
  }

  void handle_cancel(
    const std::shared_ptr<
      std_srvs::srv::Trigger::Request>,
    std::shared_ptr<
      std_srvs::srv::Trigger::Response>
      response)
  {
    if (!exploration::is_active(
        machine_.state()))
    {
      response->success = false;
      response->message =
        "no_active_exploration_goal";

      return;
    }

    if (machine_.state() ==
        exploration::
        GoalHandoffState::kCanceling)
    {
      if (!current_goal_handle_) {
        response->success = false;
        response->message =
          "accepted_goal_handle_missing";

        publish_status(
          false,
          response->message);

        return;
      }

      cancel_started_at_ =
        now();

      cancel_timeout_reported_ = false;
      cancel_acknowledged_ = false;

      try {
        send_active_cancel_request(
          machine_.request_id());

        response->success = true;
        response->message =
          "exploration_goal_cancel_request_retried";
      } catch (const std::exception & exception) {
        const std::string reason =
          std::string{
          "cancel_request_exception:"} +
          exception.what();

        publish_status(
          false,
          reason);

        response->success = false;
        response->message = reason;

        RCLCPP_ERROR(
          get_logger(),
          "cancel retry failed: %s; "
          "keeping exploration goal active",
          exception.what());
      }

      return;
    }

    const auto transition =
      machine_.request_cancel();

    if (!transition.accepted) {
      response->success = false;
      response->message =
        transition.reason;

      publish_rejection(
        transition.reason);

      return;
    }

    publish_transition(
      transition);

    if (transition.current ==
        exploration::
        GoalHandoffState::kCanceled)
    {
      clear_runtime_goal();

      response->success = true;
      response->message =
        "exploration_goal_canceled_before_acceptance";

      return;
    }

    if (!current_goal_handle_) {
      response->success = false;
      response->message =
        "accepted_goal_handle_missing";

      publish_status(
        false,
        response->message);

      RCLCPP_ERROR(
        get_logger(),
        "manual cancellation cannot access the "
        "accepted goal handle; keeping the goal active");

      return;
    }

    cancel_started_at_ =
      now();

    cancel_timeout_reported_ = false;
    cancel_acknowledged_ = false;

    try {
      send_active_cancel_request(
        machine_.request_id());

      response->success = true;
      response->message =
        "exploration_goal_cancel_request_sent";
    } catch (const std::exception & exception) {
      const std::string reason =
        std::string{
        "cancel_request_exception:"} +
        exception.what();

      publish_status(
        false,
        reason);

      response->success = false;
      response->message = reason;

      RCLCPP_ERROR(
        get_logger(),
        "manual cancel request failed: %s; "
        "keeping exploration goal active",
        exception.what());
    }
  }
  void clear_runtime_goal()
  {
    pending_goal_.reset();
    pending_started_.reset();

    goal_sent_at_.reset();
    goal_accepted_at_.reset();
    last_feedback_at_.reset();
    cancel_started_at_.reset();
    timeout_reason_.reset();
    cancel_timeout_reported_ = false;
    cancel_acknowledged_ = false;

    current_goal_handle_.reset();
  }

  void publish_transition(
    const exploration::GoalTransition &
    transition)
  {
    if (!transition.accepted) {
      publish_rejection(
        transition.reason);

      return;
    }

    publish_state();
    publish_status(true);
  }

  void publish_state()
  {
    std_msgs::msg::String message;

    message.data =
      exploration::to_string(
      machine_.state());

    state_publisher_->publish(message);
  }

  void publish_status(
    const bool accepted,
    const std::string & override_reason = "")
  {
    const std::string reason =
      override_reason.empty() ?
      machine_.reason() :
      override_reason;

    std::ostringstream output;

    output
      << std::boolalpha
      << "{"
      << "\"accepted\":"
      << accepted
      << ",\"sequence\":"
      << machine_.sequence()
      << ",\"request_id\":\""
      << escape_json(
           machine_.request_id())
      << "\",\"state\":\""
      << exploration::to_string(
           machine_.state())
      << "\",\"active\":"
      << exploration::is_active(
           machine_.state())
      << ",\"terminal\":"
      << exploration::is_terminal(
           machine_.state())
      << ",\"reason\":\""
      << escape_json(reason)
      << "\",\"action_name\":\""
      << escape_json(
           exploration::
           kExplorationActionName)
      << "\"}";

    std_msgs::msg::String message;
    message.data = output.str();

    status_publisher_->publish(message);
  }

  void publish_rejection(
    const std::string & reason)
  {
    publish_status(
      false,
      reason);
  }

  std::string expected_frame_;

  double server_wait_timeout_sec_{3.0};
  double goal_response_timeout_sec_{3.0};
  double execution_timeout_sec_{300.0};
  double feedback_stale_timeout_sec_{10.0};
  double cancel_timeout_sec_{5.0};

  bool restamp_goal_{true};

  exploration::GoalHandoffMachine
    machine_;

  std::optional<
    geometry_msgs::msg::PoseStamped>
    pending_goal_;

  std::optional<rclcpp::Time>
    pending_started_;

  std::optional<rclcpp::Time>
    goal_sent_at_;

  std::optional<rclcpp::Time>
    goal_accepted_at_;

  std::optional<rclcpp::Time>
    last_feedback_at_;

  std::optional<rclcpp::Time>
    cancel_started_at_;

  bool cancel_timeout_reported_{false};
  bool cancel_acknowledged_{false};

  std::optional<std::string>
    timeout_reason_;

  GoalHandle::SharedPtr
    current_goal_handle_;

  rclcpp_action::Client<
    NavigateToPose>::SharedPtr
    action_client_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::
    SharedPtr
    selected_goal_subscription_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    status_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    feedback_publisher_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr
    cancel_service_;

  rclcpp::TimerBase::SharedPtr
    watchdog_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      savo_mapping::
      ExplorationGoalHandoffNode>());

  rclcpp::shutdown();
  return 0;
}
