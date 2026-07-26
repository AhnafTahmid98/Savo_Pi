#include "savo_mapping/exploration_goal_handoff.hpp"
#include "savo_mapping/exploration_runtime.hpp"
#include "savo_mapping/topic_names.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace savo_mapping
{
namespace
{

bool is_known_handoff_state(
  std::string_view state)
{
  return
    state == "idle" ||
    state == "waiting_for_savo_nav" ||
    state == "sending" ||
    state == "accepted" ||
    state == "executing" ||
    state == "canceling" ||
    state == "succeeded" ||
    state == "rejected" ||
    state == "aborted" ||
    state == "canceled" ||
    state == "timed_out" ||
    state == "error";
}

bool is_active_handoff_state(
  std::string_view state)
{
  return
    state == "waiting_for_savo_nav" ||
    state == "sending" ||
    state == "accepted" ||
    state == "executing" ||
    state == "canceling";
}

std::string json_escape(
  const std::string & input)
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

const char * bool_text(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

class ExplorationManagerNode final : public rclcpp::Node
{
public:
  using StringMessage = std_msgs::msg::String;
  using BoolMessage = std_msgs::msg::Bool;
  using CancelService = std_srvs::srv::Trigger;
  using CancelFuture =
    rclcpp::Client<CancelService>::SharedFuture;

  ExplorationManagerNode()
  : Node("exploration_manager_node")
  {
    mode_topic_ =
      declare_parameter<std::string>(
      "mode_topic",
      std::string{topics::MODE});

    exploration_mode_topic_ =
      declare_parameter<std::string>(
      "exploration_mode_topic",
      std::string{topics::EXPLORATION_MODE});

    workflow_phase_topic_ =
      declare_parameter<std::string>(
      "workflow_phase_topic",
      std::string{topics::WORKFLOW_PHASE});

    session_state_topic_ =
      declare_parameter<std::string>(
      "session_state_topic",
      std::string{topics::SESSION_STATE});

    readiness_topic_ =
      declare_parameter<std::string>(
      "readiness_topic",
      std::string{topics::READINESS});

    safety_stop_topic_ =
      declare_parameter<std::string>(
      "safety_stop_topic",
      std::string{topics::SAFETY_STOP});

    handoff_state_topic_ =
      declare_parameter<std::string>(
      "handoff_state_topic",
      exploration::kGoalStateTopic);

    runtime_enabled_topic_ =
      declare_parameter<std::string>(
      "runtime_enabled_topic",
      std::string{
        topics::EXPLORATION_RUNTIME_ENABLED});

    status_topic_ =
      declare_parameter<std::string>(
      "status_topic",
      std::string{
        topics::EXPLORATION_MANAGER_STATUS});

    cancel_service_name_ =
      declare_parameter<std::string>(
      "cancel_service",
      exploration::kGoalCancelService);

    const std::int64_t evaluation_period_ms =
      declare_parameter<std::int64_t>(
      "evaluation_period_ms",
      250);

    const std::int64_t cancel_retry_period_ms =
      declare_parameter<std::int64_t>(
      "cancel_retry_period_ms",
      1000);

    validate_parameters(
      evaluation_period_ms,
      cancel_retry_period_ms);

    cancel_retry_period_ =
      std::chrono::milliseconds(
      cancel_retry_period_ms);

    const auto retained_qos =
      rclcpp::QoS(
      rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

    const auto volatile_state_qos =
      rclcpp::QoS(
      rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();

    runtime_enabled_publisher_ =
      create_publisher<BoolMessage>(
      runtime_enabled_topic_,
      retained_qos);

    status_publisher_ =
      create_publisher<StringMessage>(
      status_topic_,
      retained_qos);

    mode_subscription_ =
      create_subscription<StringMessage>(
      mode_topic_,
      retained_qos,
      std::bind(
        &ExplorationManagerNode::handle_mode,
        this,
        std::placeholders::_1));

    exploration_mode_subscription_ =
      create_subscription<StringMessage>(
      exploration_mode_topic_,
      retained_qos,
      std::bind(
        &ExplorationManagerNode::
        handle_exploration_mode,
        this,
        std::placeholders::_1));

    workflow_phase_subscription_ =
      create_subscription<StringMessage>(
      workflow_phase_topic_,
      retained_qos,
      std::bind(
        &ExplorationManagerNode::
        handle_workflow_phase,
        this,
        std::placeholders::_1));

    session_state_subscription_ =
      create_subscription<StringMessage>(
      session_state_topic_,
      retained_qos,
      std::bind(
        &ExplorationManagerNode::
        handle_session_state,
        this,
        std::placeholders::_1));

    readiness_subscription_ =
      create_subscription<StringMessage>(
      readiness_topic_,
      retained_qos,
      std::bind(
        &ExplorationManagerNode::
        handle_readiness,
        this,
        std::placeholders::_1));

    safety_stop_subscription_ =
      create_subscription<BoolMessage>(
      safety_stop_topic_,
      volatile_state_qos,
      std::bind(
        &ExplorationManagerNode::
        handle_safety_stop,
        this,
        std::placeholders::_1));

    handoff_state_subscription_ =
      create_subscription<StringMessage>(
      handoff_state_topic_,
      retained_qos,
      std::bind(
        &ExplorationManagerNode::
        handle_handoff_state,
        this,
        std::placeholders::_1));

    cancel_client_ =
      create_client<CancelService>(
      cancel_service_name_);

    evaluation_timer_ =
      create_wall_timer(
      std::chrono::milliseconds(
        evaluation_period_ms),
      std::bind(
        &ExplorationManagerNode::
        evaluate_and_apply,
        this));

    evaluate_and_apply();

    RCLCPP_INFO(
      get_logger(),
      "exploration manager ready; "
      "runtime authority publishes to %s",
      runtime_enabled_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "exploration cancellation uses only %s",
      cancel_service_name_.c_str());
  }

private:
  void validate_parameters(
    std::int64_t evaluation_period_ms,
    std::int64_t cancel_retry_period_ms) const
  {
    const bool topic_missing =
      mode_topic_.empty() ||
      exploration_mode_topic_.empty() ||
      workflow_phase_topic_.empty() ||
      session_state_topic_.empty() ||
      readiness_topic_.empty() ||
      safety_stop_topic_.empty() ||
      handoff_state_topic_.empty() ||
      runtime_enabled_topic_.empty() ||
      status_topic_.empty() ||
      cancel_service_name_.empty();

    if (topic_missing) {
      throw std::invalid_argument(
              "topic_and_service_parameters_"
              "must_not_be_empty");
    }

    if (
      evaluation_period_ms < 50 ||
      evaluation_period_ms > 10000)
    {
      throw std::invalid_argument(
              "evaluation_period_ms_out_of_range");
    }

    if (
      cancel_retry_period_ms < 100 ||
      cancel_retry_period_ms > 60000)
    {
      throw std::invalid_argument(
              "cancel_retry_period_ms_out_of_range");
    }
  }

  void handle_mode(
    const StringMessage::ConstSharedPtr message)
  {
    const auto parsed =
      mapping_mode_from_string(
      message->data);

    if (!parsed.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "ignored invalid mapping mode: %s",
        message->data.c_str());

      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.mode = parsed.value();
    }

    evaluate_and_apply();
  }

  void handle_exploration_mode(
    const StringMessage::ConstSharedPtr message)
  {
    const auto parsed =
      exploration_mode_from_string(
      message->data);

    if (!parsed.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "ignored invalid exploration mode: %s",
        message->data.c_str());

      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.exploration_mode = parsed.value();
    }

    evaluate_and_apply();
  }

  void handle_workflow_phase(
    const StringMessage::ConstSharedPtr message)
  {
    const auto parsed =
      workflow_phase_from_string(
      message->data);

    if (!parsed.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "ignored invalid workflow phase: %s",
        message->data.c_str());

      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.workflow_phase = parsed.value();
    }

    evaluate_and_apply();
  }

  void handle_session_state(
    const StringMessage::ConstSharedPtr message)
  {
    const auto parsed =
      session_state_from_string(
      message->data);

    if (!parsed.has_value()) {
      RCLCPP_WARN(
        get_logger(),
        "ignored invalid session state: %s",
        message->data.c_str());

      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.session_state = parsed.value();
    }

    evaluate_and_apply();
  }

  void handle_readiness(
    const StringMessage::ConstSharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      inputs_.readiness_received = true;
      inputs_.mapping_ready =
        message->data == "ready";

      readiness_text_ = message->data;
    }

    evaluate_and_apply();
  }

  void handle_safety_stop(
    const BoolMessage::ConstSharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      inputs_.safety_stop_received = true;
      inputs_.safety_stop_active =
        message->data;
    }

    evaluate_and_apply();
  }

  void handle_handoff_state(
    const StringMessage::ConstSharedPtr message)
  {
    if (!is_known_handoff_state(
        message->data))
    {
      RCLCPP_WARN(
        get_logger(),
        "ignored invalid handoff state: %s",
        message->data.c_str());

      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      handoff_state_text_ =
        message->data;

      inputs_.handoff_state_received = true;
      inputs_.handoff_active =
        is_active_handoff_state(
        message->data);

      if (!inputs_.handoff_active) {
        cancel_request_in_flight_ = false;
        cancel_requested_for_active_goal_ =
          false;
      }
    }

    evaluate_and_apply();
  }

  bool cancel_retry_elapsed_locked(
    std::chrono::steady_clock::time_point
    current_time) const
  {
    if (!last_cancel_attempt_.has_value()) {
      return true;
    }

    return
      current_time -
      last_cancel_attempt_.value() >=
      cancel_retry_period_;
  }

  void evaluate_and_apply()
  {
    exploration_runtime::RuntimeDecision
      decision;

    bool publish_runtime_state = false;
    bool runtime_enabled = false;
    bool dispatch_cancel_request = false;
    std::string status_json;

    const auto current_time =
      std::chrono::steady_clock::now();

    {
      std::lock_guard<std::mutex> lock(mutex_);

      decision =
        exploration_runtime::evaluate(
        inputs_);

      runtime_enabled =
        decision.frontier_enabled;

      if (
        !runtime_state_published_ ||
        runtime_enabled !=
        last_runtime_enabled_)
      {
        runtime_state_published_ = true;
        last_runtime_enabled_ =
          runtime_enabled;

        publish_runtime_state = true;
      }

      const bool service_ready =
        cancel_client_->service_is_ready();

      if (
        decision.cancel_active_goal &&
        inputs_.handoff_active &&
        !cancel_request_in_flight_ &&
        !cancel_requested_for_active_goal_ &&
        service_ready &&
        cancel_retry_elapsed_locked(
          current_time))
      {
        cancel_request_in_flight_ = true;
        last_cancel_attempt_ =
          current_time;

        dispatch_cancel_request = true;
      }

      status_json =
        make_status_json_locked(
        decision,
        service_ready);
    }

    if (publish_runtime_state) {
      BoolMessage message;
      message.data = runtime_enabled;

      runtime_enabled_publisher_->publish(
        message);

      RCLCPP_INFO(
        get_logger(),
        "frontier runtime authority changed: %s",
        runtime_enabled ? "enabled" : "disabled");
    }

    publish_status(status_json);

    if (dispatch_cancel_request) {
      dispatch_cancel();
    }
  }

  void dispatch_cancel()
  {
    const auto request =
      std::make_shared<
      CancelService::Request>();

    try {
      cancel_client_->async_send_request(
        request,
        [this](CancelFuture future) {
          handle_cancel_response(
            std::move(future));
        });
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        cancel_request_in_flight_ = false;
        last_cancel_response_ =
          std::string{
          "request_exception: "} +
        error.what();
      }

      RCLCPP_ERROR(
        get_logger(),
        "exploration cancel request failed: %s",
        error.what());

      evaluate_and_apply();
    }
  }

  void handle_cancel_response(
    CancelFuture future)
  {
    bool success = false;
    std::string response_text;

    try {
      const auto response =
        future.get();

      success = response->success;
      response_text =
        response->message;
    } catch (const std::exception & error) {
      response_text =
        std::string{
        "response_exception: "} +
      error.what();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      cancel_request_in_flight_ = false;

      if (
        success &&
        inputs_.handoff_active)
      {
        cancel_requested_for_active_goal_ =
          true;
      }

      last_cancel_response_ =
        response_text;
    }

    if (success) {
      RCLCPP_INFO(
        get_logger(),
        "exploration cancel request accepted: %s",
        response_text.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "exploration cancel request rejected: %s",
        response_text.c_str());
    }

    evaluate_and_apply();
  }

  std::string make_status_json_locked(
    const exploration_runtime::
    RuntimeDecision & decision,
    bool cancel_service_ready) const
  {
    std::ostringstream output;

    output
      << "{\"disposition\":\""
      << exploration_runtime::to_string(
      decision.disposition)
      << "\",\"frontier_enabled\":"
      << bool_text(
      decision.frontier_enabled)
      << ",\"cancel_required\":"
      << bool_text(
      decision.cancel_active_goal)
      << ",\"cancel_request_in_flight\":"
      << bool_text(
      cancel_request_in_flight_)
      << ",\"cancel_requested\":"
      << bool_text(
      cancel_requested_for_active_goal_)
      << ",\"cancel_service_ready\":"
      << bool_text(
      cancel_service_ready)
      << ",\"mapping_ready\":"
      << bool_text(
      inputs_.mapping_ready)
      << ",\"safety_stop_active\":"
      << bool_text(
      inputs_.safety_stop_active)
      << ",\"handoff_active\":"
      << bool_text(
      inputs_.handoff_active)
      << ",\"readiness\":\""
      << json_escape(
      readiness_text_)
      << "\",\"handoff_state\":\""
      << json_escape(
      handoff_state_text_)
      << "\",\"reason\":\""
      << json_escape(
      decision.reason)
      << "\",\"cancel_response\":\""
      << json_escape(
      last_cancel_response_)
      << "\"}";

    return output.str();
  }

  void publish_status(
    const std::string & status_json)
  {
    StringMessage message;
    message.data = status_json;

    status_publisher_->publish(message);
  }

  std::mutex mutex_;

  exploration_runtime::RuntimeInputs inputs_;

  std::string mode_topic_;
  std::string exploration_mode_topic_;
  std::string workflow_phase_topic_;
  std::string session_state_topic_;
  std::string readiness_topic_;
  std::string safety_stop_topic_;
  std::string handoff_state_topic_;
  std::string runtime_enabled_topic_;
  std::string status_topic_;
  std::string cancel_service_name_;

  std::string readiness_text_{
    "not_received"};

  std::string handoff_state_text_{
    "not_received"};

  std::string last_cancel_response_{
    "none"};

  bool runtime_state_published_{false};
  bool last_runtime_enabled_{false};

  bool cancel_request_in_flight_{false};

  bool cancel_requested_for_active_goal_{
    false};

  std::optional<
    std::chrono::steady_clock::time_point>
  last_cancel_attempt_;

  std::chrono::milliseconds
    cancel_retry_period_{1000};

  rclcpp::Publisher<
    BoolMessage>::SharedPtr
    runtime_enabled_publisher_;

  rclcpp::Publisher<
    StringMessage>::SharedPtr
    status_publisher_;

  rclcpp::Subscription<
    StringMessage>::SharedPtr
    mode_subscription_;

  rclcpp::Subscription<
    StringMessage>::SharedPtr
    exploration_mode_subscription_;

  rclcpp::Subscription<
    StringMessage>::SharedPtr
    workflow_phase_subscription_;

  rclcpp::Subscription<
    StringMessage>::SharedPtr
    session_state_subscription_;

  rclcpp::Subscription<
    StringMessage>::SharedPtr
    readiness_subscription_;

  rclcpp::Subscription<
    BoolMessage>::SharedPtr
    safety_stop_subscription_;

  rclcpp::Subscription<
    StringMessage>::SharedPtr
    handoff_state_subscription_;

  rclcpp::Client<
    CancelService>::SharedPtr
    cancel_client_;

  rclcpp::TimerBase::SharedPtr
    evaluation_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        savo_mapping::
        ExplorationManagerNode>());
  } catch (const std::exception & error) {
    std::cerr
      << "exploration_manager_node failed: "
      << error.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
