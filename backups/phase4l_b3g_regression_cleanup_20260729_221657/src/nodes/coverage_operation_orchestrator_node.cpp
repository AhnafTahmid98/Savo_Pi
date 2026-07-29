// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "savo_mapping/coverage_execution_handoff.hpp"
#include "savo_mapping/coverage_operation_orchestrator.hpp"
#include "savo_mapping/qos_profiles.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace coverage = savo_mapping::coverage;
namespace qos = savo_mapping::qos;

namespace
{

constexpr std::int64_t kMinimumWatchdogPeriodMs = 20;
constexpr std::int64_t kMaximumWatchdogPeriodMs = 5000;
constexpr std::int64_t kMinimumPublishPeriodMs = 100;
constexpr std::int64_t kMaximumPublishPeriodMs = 10000;

std::string json_escape(const std::string & value)
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

const char * bool_text(const bool value)
{
  return value ? "true" : "false";
}

}  // namespace

class CoverageOperationOrchestratorNode : public rclcpp::Node
{
public:
  using Trigger = std_srvs::srv::Trigger;

  CoverageOperationOrchestratorNode()
  : Node("coverage_operation_orchestrator_node")
  {
    callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    load_parameters();
    validate_parameters();
    create_interfaces();

    publish_state();
    publish_status();

    RCLCPP_INFO(
      get_logger(),
      "Coverage operation orchestrator started | public_approve=%s "
      "internal_approve=%s supervisor=%s",
      public_approve_service_.c_str(),
      internal_approve_service_.c_str(),
      supervisor_state_topic_.c_str());
  }

private:
  void load_parameters()
  {
    enabled_ = declare_parameter<bool>("enabled", true);

    supervisor_state_topic_ = declare_parameter<std::string>(
      "supervisor_state_topic", coverage::kSupervisorStateTopic);
    handoff_state_topic_ = declare_parameter<std::string>(
      "handoff_state_topic", coverage::kCoverageExecutionStateTopic);
    handoff_status_topic_ = declare_parameter<std::string>(
      "handoff_status_topic", coverage::kCoverageExecutionStatusTopic);
    handoff_feedback_topic_ = declare_parameter<std::string>(
      "handoff_feedback_topic", coverage::kCoverageExecutionFeedbackTopic);

    state_topic_ = declare_parameter<std::string>(
      "state_topic", coverage::kCoverageOperationStateTopic);
    status_topic_ = declare_parameter<std::string>(
      "status_topic", coverage::kCoverageOperationStatusTopic);
    events_topic_ = declare_parameter<std::string>(
      "events_topic", coverage::kCoverageOperationEventsTopic);

    public_approve_service_ = declare_parameter<std::string>(
      "approve_service", coverage::kCoverageOperationApproveService);
    public_cancel_service_ = declare_parameter<std::string>(
      "cancel_service", coverage::kCoverageOperationCancelService);
    public_reset_service_ = declare_parameter<std::string>(
      "reset_service", coverage::kCoverageOperationResetService);

    internal_approve_service_ = declare_parameter<std::string>(
      "internal_approve_service",
      coverage::kInternalCoverageApproveService);
    internal_cancel_service_ = declare_parameter<std::string>(
      "internal_cancel_service",
      coverage::kInternalCoverageCancelService);
    internal_reset_service_ = declare_parameter<std::string>(
      "internal_reset_service",
      coverage::kInternalCoverageResetService);

    policy_.supervisor_timeout_sec = declare_parameter<double>(
      "supervisor_timeout_sec", 1.5);
    policy_.maximum_candidate_age_sec = declare_parameter<double>(
      "maximum_candidate_age_sec", 300.0);
    policy_.internal_service_timeout_sec = declare_parameter<double>(
      "internal_service_timeout_sec", 2.0);
    policy_.supervisor_loss_cancel_delay_sec =
      declare_parameter<double>(
      "supervisor_loss_cancel_delay_sec", 0.5);
    policy_.allow_degraded_supervisor = declare_parameter<bool>(
      "allow_degraded_supervisor", false);
    policy_.cancel_on_supervisor_loss = declare_parameter<bool>(
      "cancel_on_supervisor_loss", true);

    watchdog_period_ms_ = declare_parameter<std::int64_t>(
      "watchdog_period_ms", 100);
    publish_period_ms_ = declare_parameter<std::int64_t>(
      "publish_period_ms", 500);
  }

  void validate_parameters() const
  {
    const auto policy_error =
      coverage::validate_coverage_operation_policy(policy_);
    if (!policy_error.empty()) {
      throw std::runtime_error(policy_error);
    }

    const std::string * names[] = {
      &supervisor_state_topic_,
      &handoff_state_topic_,
      &handoff_status_topic_,
      &handoff_feedback_topic_,
      &state_topic_,
      &status_topic_,
      &events_topic_,
      &public_approve_service_,
      &public_cancel_service_,
      &public_reset_service_,
      &internal_approve_service_,
      &internal_cancel_service_,
      &internal_reset_service_};
    for (const auto * name : names) {
      if (name->empty()) {
        throw std::runtime_error(
                "coverage_operation_endpoint_empty");
      }
    }

    if (public_approve_service_ == internal_approve_service_ ||
      public_cancel_service_ == internal_cancel_service_ ||
      public_reset_service_ == internal_reset_service_)
    {
      throw std::runtime_error(
              "coverage_operation_public_internal_endpoint_collision");
    }

    if (watchdog_period_ms_ < kMinimumWatchdogPeriodMs ||
      watchdog_period_ms_ > kMaximumWatchdogPeriodMs)
    {
      throw std::runtime_error(
              "coverage_operation_watchdog_period_invalid");
    }
    if (publish_period_ms_ < kMinimumPublishPeriodMs ||
      publish_period_ms_ > kMaximumPublishPeriodMs)
    {
      throw std::runtime_error(
              "coverage_operation_publish_period_invalid");
    }
  }

  void create_interfaces()
  {
    state_publisher_ = create_publisher<std_msgs::msg::String>(
      state_topic_, qos::state_qos());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      status_topic_, qos::status_qos());
    events_publisher_ = create_publisher<std_msgs::msg::String>(
      events_topic_, qos::event_qos());

    const auto latched_qos =
      rclcpp::QoS(1).reliable().transient_local();

    supervisor_subscription_ =
      create_subscription<std_msgs::msg::String>(
      supervisor_state_topic_,
      latched_qos,
      std::bind(
        &CoverageOperationOrchestratorNode::handle_supervisor,
        this,
        std::placeholders::_1));

    handoff_state_subscription_ =
      create_subscription<std_msgs::msg::String>(
      handoff_state_topic_,
      latched_qos,
      std::bind(
        &CoverageOperationOrchestratorNode::handle_handoff_state,
        this,
        std::placeholders::_1));

    handoff_status_subscription_ =
      create_subscription<std_msgs::msg::String>(
      handoff_status_topic_,
      latched_qos,
      std::bind(
        &CoverageOperationOrchestratorNode::handle_handoff_status,
        this,
        std::placeholders::_1));

    handoff_feedback_subscription_ =
      create_subscription<std_msgs::msg::String>(
      handoff_feedback_topic_,
      qos::status_qos(),
      std::bind(
        &CoverageOperationOrchestratorNode::handle_handoff_feedback,
        this,
        std::placeholders::_1));

    internal_approve_client_ = create_client<Trigger>(
      internal_approve_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    internal_cancel_client_ = create_client<Trigger>(
      internal_cancel_service_,
      rclcpp::ServicesQoS(),
      callback_group_);
    internal_reset_client_ = create_client<Trigger>(
      internal_reset_service_,
      rclcpp::ServicesQoS(),
      callback_group_);

    public_approve_server_ = create_service<Trigger>(
      public_approve_service_,
      std::bind(
        &CoverageOperationOrchestratorNode::handle_public_approve,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(),
      callback_group_);
    public_cancel_server_ = create_service<Trigger>(
      public_cancel_service_,
      std::bind(
        &CoverageOperationOrchestratorNode::handle_public_cancel,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(),
      callback_group_);
    public_reset_server_ = create_service<Trigger>(
      public_reset_service_,
      std::bind(
        &CoverageOperationOrchestratorNode::handle_public_reset,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(),
      callback_group_);

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(watchdog_period_ms_),
      std::bind(
        &CoverageOperationOrchestratorNode::watchdog_tick,
        this));
    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(
        &CoverageOperationOrchestratorNode::publish_tick,
        this));
  }

  void handle_supervisor(
    const std_msgs::msg::String::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      supervisor_snapshot_ =
        coverage::parse_supervisor_authorization(message->data);
      supervisor_received_at_ =
        std::chrono::steady_clock::now();
    }
    publish_state();
    publish_status();
  }

  void handle_handoff_state(
    const std_msgs::msg::String::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handoff_state_text_ = message->data;
      if (handoff_snapshot_.valid) {
        handoff_snapshot_.state = message->data;
      }
    }
    publish_state();
    publish_status();
  }

  void handle_handoff_status(
    const std_msgs::msg::String::SharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto parsed =
        coverage::parse_coverage_handoff_snapshot(message->data);
      if (parsed.valid) {
        if (!handoff_state_text_.empty()) {
          parsed.state = handoff_state_text_;
        }
        handoff_snapshot_ = parsed;
        handoff_status_received_at_ =
          std::chrono::steady_clock::now();
      }
    }
    publish_state();
    publish_status();
  }

  void handle_handoff_feedback(
    const std_msgs::msg::String::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_feedback_ = message->data;
  }

  double age_seconds(
    const std::optional<
      std::chrono::steady_clock::time_point> & received_at) const
  {
    if (!received_at.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(
      0.0,
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() -
        *received_at).count());
  }

  coverage::CoverageApprovalDecision approval_decision() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double supervisor_age =
      age_seconds(supervisor_received_at_);
    const double status_age =
      age_seconds(handoff_status_received_at_);
    const double candidate_age =
      handoff_snapshot_.candidate_age_sec + status_age;
    return coverage::evaluate_coverage_approval(
      supervisor_snapshot_,
      supervisor_age,
      handoff_snapshot_,
      candidate_age,
      policy_);
  }

  bool call_internal_service(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    std::string & message)
  {
    const auto timeout = std::chrono::duration<double>(
      policy_.internal_service_timeout_sec);
    if (!client->wait_for_service(timeout)) {
      message = "coverage_operation_internal_service_unavailable";
      return false;
    }

    auto request = std::make_shared<Trigger::Request>();
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      message = "coverage_operation_internal_service_timeout";
      return false;
    }

    const auto response = future.get();
    message = response->message;
    return response->success;
  }

  void handle_public_approve(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response)
  {
    if (!enabled_) {
      response->success = false;
      response->message = "coverage_operation_disabled";
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (approval_pending_) {
        response->success = false;
        response->message =
          "coverage_operation_approval_already_pending";
        return;
      }
    }

    const auto decision = approval_decision();
    if (!decision.accepted) {
      response->success = false;
      response->message = decision.reason;
      publish_event("approval_rejected", decision.reason);
      publish_state();
      publish_status();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      approval_pending_ = true;
      pending_candidate_generation_ =
        decision.candidate_generation;
    }
    publish_state();
    publish_status();

    std::string internal_message;
    const bool accepted = call_internal_service(
      internal_approve_client_, internal_message);

    bool generation_matches = false;
    if (accepted) {
      generation_matches =
        coverage::mission_id_matches_candidate_generation(
        internal_message,
        decision.candidate_generation);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      approval_pending_ = false;
      pending_candidate_generation_ = 0U;
      if (accepted && generation_matches) {
        approved_candidate_generation_ =
          decision.candidate_generation;
        approved_mission_id_ = internal_message;
      }
    }

    if (accepted && !generation_matches) {
      std::string cancel_message;
      static_cast<void>(call_internal_service(
        internal_cancel_client_, cancel_message));
      response->success = false;
      response->message =
        "coverage_operation_candidate_changed_during_approval";
      publish_event(
        "approval_revoked",
        response->message);
    } else {
      response->success = accepted;
      response->message = internal_message;
      publish_event(
        accepted ? "approval_accepted" : "approval_failed",
        internal_message);
    }

    publish_state();
    publish_status();
  }

  void handle_public_cancel(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response)
  {
    std::string internal_message;
    response->success = call_internal_service(
      internal_cancel_client_, internal_message);
    response->message = internal_message;
    publish_event(
      response->success ? "cancel_requested" : "cancel_rejected",
      internal_message);
    publish_state();
    publish_status();
  }

  void handle_public_reset(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response)
  {
    std::string internal_message;
    response->success = call_internal_service(
      internal_reset_client_, internal_message);
    response->message = internal_message;
    if (response->success) {
      std::lock_guard<std::mutex> lock(mutex_);
      approved_candidate_generation_ = 0U;
      approved_mission_id_.clear();
      supervisor_cancel_requested_ = false;
    }
    publish_event(
      response->success ? "operation_reset" : "reset_rejected",
      internal_message);
    publish_state();
    publish_status();
  }

  void watchdog_tick()
  {
    if (!enabled_ || !policy_.cancel_on_supervisor_loss) {
      return;
    }

    bool should_start_loss_timer = false;
    bool should_cancel = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool active =
        coverage::coverage_handoff_state_active(
        handoff_snapshot_.state);
      const bool authorized =
        age_seconds(supervisor_received_at_) <=
        policy_.supervisor_timeout_sec &&
        coverage::supervisor_authorized(
        supervisor_snapshot_, policy_);

      if (!active || authorized ||
        handoff_snapshot_.state == "canceling")
      {
        supervisor_loss_started_at_.reset();
        if (!active) {
          supervisor_cancel_requested_ = false;
        }
        return;
      }

      if (!supervisor_loss_started_at_.has_value()) {
        should_start_loss_timer = true;
        supervisor_loss_started_at_ =
          std::chrono::steady_clock::now();
      } else if (!supervisor_cancel_requested_ &&
        age_seconds(supervisor_loss_started_at_) >=
        policy_.supervisor_loss_cancel_delay_sec)
      {
        supervisor_cancel_requested_ = true;
        should_cancel = true;
      }
    }

    if (should_start_loss_timer) {
      publish_event(
        "supervisor_authorization_lost",
        "coverage_operation_supervisor_not_authorized");
      publish_state();
      publish_status();
    }

    if (should_cancel) {
      std::string internal_message;
      const bool accepted = call_internal_service(
        internal_cancel_client_, internal_message);
      publish_event(
        accepted ?
        "supervisor_loss_cancel_requested" :
        "supervisor_loss_cancel_failed",
        internal_message);
      publish_state();
      publish_status();
    }
  }

  void publish_tick()
  {
    publish_state();
    publish_status();
  }

  std::string effective_state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool authorized =
      age_seconds(supervisor_received_at_) <=
      policy_.supervisor_timeout_sec &&
      coverage::supervisor_authorized(
      supervisor_snapshot_, policy_);
    return coverage::coverage_operation_effective_state(
      enabled_,
      approval_pending_,
      authorized,
      handoff_snapshot_);
  }

  void publish_state()
  {
    const auto state = effective_state();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state == last_state_) {
        return;
      }
      last_state_ = state;
    }
    std_msgs::msg::String message;
    message.data = state;
    state_publisher_->publish(message);
  }

  void publish_status()
  {
    coverage::SupervisorAuthorizationSnapshot supervisor;
    coverage::CoverageHandoffSnapshot handoff;
    std::string feedback;
    std::string approved_mission;
    std::uint64_t approved_generation = 0U;
    std::uint64_t pending_generation = 0U;
    bool pending = false;
    bool supervisor_cancel_requested = false;
    double supervisor_age = 0.0;
    double candidate_age = 0.0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      supervisor = supervisor_snapshot_;
      handoff = handoff_snapshot_;
      feedback = latest_feedback_;
      approved_mission = approved_mission_id_;
      approved_generation = approved_candidate_generation_;
      pending_generation = pending_candidate_generation_;
      pending = approval_pending_;
      supervisor_cancel_requested =
        supervisor_cancel_requested_;
      supervisor_age = age_seconds(supervisor_received_at_);
      candidate_age = handoff.candidate_age_sec +
        age_seconds(handoff_status_received_at_);
    }

    const bool authorized =
      supervisor_age <= policy_.supervisor_timeout_sec &&
      coverage::supervisor_authorized(supervisor, policy_);
    const double supervisor_age_json =
      std::isfinite(supervisor_age) ? supervisor_age : -1.0;
    const double candidate_age_json =
      std::isfinite(candidate_age) ? candidate_age : -1.0;
    const auto state =
      coverage::coverage_operation_effective_state(
      enabled_, pending, authorized, handoff);

    std::ostringstream output;
    output.precision(17);
    output
      << "{\"schema_version\":1"
      << ",\"node\":\"coverage_operation_orchestrator\""
      << ",\"enabled\":" << bool_text(enabled_)
      << ",\"state\":\"" << json_escape(state) << "\""
      << ",\"supervisor_authorized\":"
      << bool_text(authorized)
      << ",\"supervisor_age_sec\":" << supervisor_age_json
      << ",\"supervisor_lifecycle\":\""
      << json_escape(supervisor.lifecycle) << "\""
      << ",\"supervisor_health\":\""
      << json_escape(supervisor.health) << "\""
      << ",\"supervisor_reason\":\""
      << json_escape(supervisor.reason) << "\""
      << ",\"handoff_state\":\""
      << json_escape(handoff.state) << "\""
      << ",\"handoff_reason\":\""
      << json_escape(handoff.reason) << "\""
      << ",\"candidate_valid\":"
      << bool_text(handoff.candidate_valid)
      << ",\"candidate_generation\":"
      << handoff.candidate_generation
      << ",\"candidate_age_sec\":" << candidate_age_json
      << ",\"approval_pending\":" << bool_text(pending)
      << ",\"pending_candidate_generation\":"
      << pending_generation
      << ",\"approved_candidate_generation\":"
      << approved_generation
      << ",\"mission_id\":\""
      << json_escape(
      handoff.mission_id.empty() ?
      approved_mission : handoff.mission_id)
      << "\""
      << ",\"terminal_state\":\""
      << json_escape(handoff.terminal_state) << "\""
      << ",\"result_reason\":\""
      << json_escape(handoff.result_reason) << "\""
      << ",\"supervisor_cancel_requested\":"
      << bool_text(supervisor_cancel_requested)
      << ",\"latest_feedback\":\""
      << json_escape(feedback) << "\"}";

    std_msgs::msg::String message;
    message.data = output.str();
    status_publisher_->publish(message);
  }

  void publish_event(
    const std::string & event,
    const std::string & reason)
  {
    std_msgs::msg::String message;
    std::ostringstream output;
    output
      << "{\"schema_version\":1"
      << ",\"node\":\"coverage_operation_orchestrator\""
      << ",\"event\":\"" << json_escape(event) << "\""
      << ",\"reason\":\"" << json_escape(reason) << "\""
      << ",\"stamp_s\":" << now().seconds() << "}";
    message.data = output.str();
    events_publisher_->publish(message);
  }

  bool enabled_{true};
  coverage::CoverageOperationPolicy policy_;
  std::int64_t watchdog_period_ms_{100};
  std::int64_t publish_period_ms_{500};

  std::string supervisor_state_topic_;
  std::string handoff_state_topic_;
  std::string handoff_status_topic_;
  std::string handoff_feedback_topic_;
  std::string state_topic_;
  std::string status_topic_;
  std::string events_topic_;
  std::string public_approve_service_;
  std::string public_cancel_service_;
  std::string public_reset_service_;
  std::string internal_approve_service_;
  std::string internal_cancel_service_;
  std::string internal_reset_service_;

  mutable std::mutex mutex_;
  coverage::SupervisorAuthorizationSnapshot supervisor_snapshot_;
  coverage::CoverageHandoffSnapshot handoff_snapshot_;
  std::string handoff_state_text_;
  std::string latest_feedback_;
  std::string last_state_;
  std::string approved_mission_id_;
  std::uint64_t approved_candidate_generation_{0U};
  std::uint64_t pending_candidate_generation_{0U};
  bool approval_pending_{false};
  bool supervisor_cancel_requested_{false};
  std::optional<std::chrono::steady_clock::time_point>
  supervisor_received_at_;
  std::optional<std::chrono::steady_clock::time_point>
  handoff_status_received_at_;
  std::optional<std::chrono::steady_clock::time_point>
  supervisor_loss_started_at_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    supervisor_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    handoff_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    handoff_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    handoff_feedback_subscription_;
  rclcpp::Client<Trigger>::SharedPtr internal_approve_client_;
  rclcpp::Client<Trigger>::SharedPtr internal_cancel_client_;
  rclcpp::Client<Trigger>::SharedPtr internal_reset_client_;
  rclcpp::Service<Trigger>::SharedPtr public_approve_server_;
  rclcpp::Service<Trigger>::SharedPtr public_cancel_server_;
  rclcpp::Service<Trigger>::SharedPtr public_reset_server_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<CoverageOperationOrchestratorNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 4U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
