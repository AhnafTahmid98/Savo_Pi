// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "savo_msgs/msg/location_candidate.hpp"
#include "savo_msgs/msg/location_record.hpp"
#include "savo_msgs/srv/approve_location.hpp"
#include "savo_msgs/srv/authorize_location_operation.hpp"
#include "savo_msgs/srv/get_location_candidate.hpp"
#include "savo_msgs/srv/reject_location_candidate.hpp"
#include "savo_msgs/srv/review_location_candidate.hpp"

#include "savo_mapping/qos_profiles.hpp"
#include "savo_mapping/topic_names.hpp"

namespace savo_mapping
{
namespace
{

using Json = nlohmann::json;
using ReviewService = savo_msgs::srv::ReviewLocationCandidate;
using GetCandidate = savo_msgs::srv::GetLocationCandidate;
using Authorize = savo_msgs::srv::AuthorizeLocationOperation;
using Approve = savo_msgs::srv::ApproveLocation;
using Reject = savo_msgs::srv::RejectLocationCandidate;

[[nodiscard]] std::chrono::nanoseconds timeout_duration(
  const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

template<typename FutureT>
[[nodiscard]] bool wait_until_ready(
  FutureT & future,
  const double timeout_s)
{
  return future.wait_for(timeout_duration(timeout_s)) ==
         std::future_status::ready;
}

[[nodiscard]] bool blank(const std::string_view value)
{
  for (const char character : value) {
    if (character != ' ' && character != '\t' && character != '\n' &&
      character != '\r' && character != '\f' && character != '\v')
    {
      return false;
    }
  }
  return true;
}

[[nodiscard]] std::string decision_text(const std::uint8_t decision)
{
  switch (decision) {
    case ReviewService::Request::DECISION_APPROVE:
      return "approve";
    case ReviewService::Request::DECISION_REJECT:
      return "reject";
    default:
      return "unknown";
  }
}

}  // namespace

class LocationReviewGatewayNode final : public rclcpp::Node
{
public:
  LocationReviewGatewayNode()
  : Node("location_review_gateway_node")
  {
    service_name_ = declare_parameter<std::string>(
      "service_name", "/savo_mapping/locations/review");
    candidate_lookup_service_name_ = declare_parameter<std::string>(
      "candidate_lookup_service", "/savo_locations/candidates/get");
    authorization_service_name_ = declare_parameter<std::string>(
      "authorization_service", "/savo_supervisor/authorize_location_operation");
    approval_service_name_ = declare_parameter<std::string>(
      "approval_service", "/savo_locations/candidates/approve");
    rejection_service_name_ = declare_parameter<std::string>(
      "rejection_service", "/savo_locations/candidates/reject");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", std::string{topics::LOCATION_REVIEW_STATUS});
    result_topic_ = declare_parameter<std::string>(
      "result_topic", std::string{topics::LOCATION_REVIEW_RESULTS});
    heartbeat_topic_ = declare_parameter<std::string>(
      "heartbeat_topic", std::string{topics::LOCATION_REVIEW_HEARTBEAT});
    dependency_wait_timeout_s_ = declare_parameter<double>(
      "dependency_wait_timeout_s", 2.0);
    operation_timeout_s_ = declare_parameter<double>(
      "operation_timeout_s", 5.0);
    status_publish_hz_ = declare_parameter<double>(
      "status_publish_hz", 1.0);
    heartbeat_publish_hz_ = declare_parameter<double>(
      "heartbeat_publish_hz", 2.0);

    if (service_name_.empty() || candidate_lookup_service_name_.empty() ||
      authorization_service_name_.empty() || approval_service_name_.empty() ||
      rejection_service_name_.empty() || status_topic_.empty() ||
      result_topic_.empty() || heartbeat_topic_.empty() ||
      dependency_wait_timeout_s_ <= 0.0 || operation_timeout_s_ <= 0.0 ||
      status_publish_hz_ <= 0.0 || heartbeat_publish_hz_ <= 0.0)
    {
      throw std::runtime_error("invalid location review gateway parameters");
    }

    callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

    candidate_client_ = create_client<GetCandidate>(
      candidate_lookup_service_name_,
      rclcpp::ServicesQoS(),
      callback_group_);
    authorization_client_ = create_client<Authorize>(
      authorization_service_name_,
      rclcpp::ServicesQoS(),
      callback_group_);
    approval_client_ = create_client<Approve>(
      approval_service_name_,
      rclcpp::ServicesQoS(),
      callback_group_);
    rejection_client_ = create_client<Reject>(
      rejection_service_name_,
      rclcpp::ServicesQoS(),
      callback_group_);

    status_publisher_ = create_publisher<std_msgs::msg::String>(
      status_topic_, qos::state_qos());
    result_publisher_ = create_publisher<std_msgs::msg::String>(
      result_topic_, qos::event_qos());
    heartbeat_publisher_ = create_publisher<std_msgs::msg::UInt64>(
      heartbeat_topic_, qos::status_qos());

    service_ = create_service<ReviewService>(
      service_name_,
      std::bind(
        &LocationReviewGatewayNode::handle_review,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(),
      callback_group_);

    status_timer_ = create_wall_timer(
      timeout_duration(1.0 / status_publish_hz_),
      std::bind(&LocationReviewGatewayNode::publish_status, this));
    heartbeat_timer_ = create_wall_timer(
      timeout_duration(1.0 / heartbeat_publish_hz_),
      std::bind(&LocationReviewGatewayNode::publish_heartbeat, this));

    update_state("ready", "review_gateway_ready", "", "", "none");
    publish_status();

    RCLCPP_INFO(
      get_logger(),
      "location review gateway ready service=%s",
      service_name_.c_str());
  }

private:
  void handle_review(
    const std::shared_ptr<ReviewService::Request> request,
    std::shared_ptr<ReviewService::Response> response)
  {
    std::unique_lock<std::mutex> operation_lock{
      operation_mutex_, std::try_to_lock};

    if (!operation_lock.owns_lock()) {
      response->completed = false;
      response->approved = false;
      response->rejected = false;
      response->result_code = ReviewService::Response::RESULT_BUSY;
      response->reason = "another location review is already active";
      response->completed_at = now();
      reviews_failed_.fetch_add(1U);
      publish_result(*request, *response);
      return;
    }

    reviews_started_.fetch_add(1U);
    const auto decision = decision_text(request->decision);
    update_state(
      "processing",
      "validating_review_request",
      request->request_id,
      request->candidate_id,
      decision);

    if (blank(request->request_id) || blank(request->actor_id) ||
      blank(request->candidate_id) || request->expected_candidate_revision == 0U ||
      (request->decision != ReviewService::Request::DECISION_APPROVE &&
      request->decision != ReviewService::Request::DECISION_REJECT) ||
      (request->decision == ReviewService::Request::DECISION_REJECT &&
      blank(request->rejection_reason)))
    {
      finish(
        *request,
        response,
        ReviewService::Response::RESULT_INVALID_REQUEST,
        "request ID, actor, decision, candidate ID and revision are required");
      return;
    }

    const auto candidate = lookup_candidate(*request, response);
    if (!candidate.has_value()) {
      return;
    }

    response->candidate = candidate.value();

    if (candidate->state != savo_msgs::msg::LocationCandidate::STATE_PENDING_REVIEW) {
      finish(
        *request,
        response,
        ReviewService::Response::RESULT_CANDIDATE_NOT_PENDING,
        "candidate is not pending review");
      return;
    }

    if (candidate->candidate_revision != request->expected_candidate_revision) {
      finish(
        *request,
        response,
        ReviewService::Response::RESULT_STALE_REVISION,
        "candidate revision changed before authorization");
      return;
    }

    if (!authorize_review(*request, candidate.value(), response)) {
      return;
    }

    if (request->decision == ReviewService::Request::DECISION_APPROVE) {
      forward_approval(*request, response);
    } else {
      forward_rejection(*request, response);
    }
  }

  std::optional<savo_msgs::msg::LocationCandidate> lookup_candidate(
    const ReviewService::Request & request,
    const std::shared_ptr<ReviewService::Response> & response)
  {
    update_state(
      "processing",
      "loading_authoritative_candidate",
      request.request_id,
      request.candidate_id,
      decision_text(request.decision));

    if (!candidate_client_->wait_for_service(
        timeout_duration(dependency_wait_timeout_s_)))
    {
      finish(
        request,
        response,
        ReviewService::Response::RESULT_REGISTRY_UNAVAILABLE,
        "candidate lookup service unavailable");
      return std::nullopt;
    }

    auto lookup_request = std::make_shared<GetCandidate::Request>();
    lookup_request->candidate_id = request.candidate_id;
    auto future = candidate_client_->async_send_request(lookup_request);

    if (!wait_until_ready(future, operation_timeout_s_)) {
      finish(
        request,
        response,
        ReviewService::Response::RESULT_TIMED_OUT,
        "candidate lookup timed out");
      return std::nullopt;
    }

    const auto lookup = future.get();
    if (!lookup->found) {
      const auto code =
        lookup->result_code == GetCandidate::Response::RESULT_NOT_FOUND ?
        ReviewService::Response::RESULT_CANDIDATE_NOT_FOUND :
        ReviewService::Response::RESULT_REGISTRY_UNAVAILABLE;
      finish(request, response, code, lookup->reason);
      return std::nullopt;
    }

    return lookup->candidate;
  }

  bool authorize_review(
    const ReviewService::Request & request,
    const savo_msgs::msg::LocationCandidate & candidate,
    const std::shared_ptr<ReviewService::Response> & response)
  {
    update_state(
      "processing",
      "requesting_supervisor_authorization",
      request.request_id,
      request.candidate_id,
      decision_text(request.decision));

    if (!authorization_client_->wait_for_service(
        timeout_duration(dependency_wait_timeout_s_)))
    {
      finish(
        request,
        response,
        ReviewService::Response::RESULT_SUPERVISOR_UNAVAILABLE,
        "supervisor authorization service unavailable");
      return false;
    }

    auto authorization = std::make_shared<Authorize::Request>();
    authorization->operation =
      request.decision == ReviewService::Request::DECISION_APPROVE ?
      Authorize::Request::OP_APPROVE_LOCATION :
      Authorize::Request::OP_REJECT_LOCATION_CANDIDATE;
    authorization->request_id = request.request_id;
    authorization->actor_id = request.actor_id;
    authorization->candidate_id = candidate.candidate_id;
    authorization->location_id =
      request.decision == ReviewService::Request::DECISION_APPROVE ?
      request.location_id : "";
    authorization->map_id = candidate.map_id;
    authorization->map_revision = candidate.map_revision;
    authorization->motion_required = false;

    auto future = authorization_client_->async_send_request(authorization);
    if (!wait_until_ready(future, operation_timeout_s_)) {
      finish(
        request,
        response,
        ReviewService::Response::RESULT_TIMED_OUT,
        "supervisor authorization timed out");
      return false;
    }

    const auto decision = future.get();
    if (!decision->authorized) {
      authorization_denials_.fetch_add(1U);
      finish(
        request,
        response,
        ReviewService::Response::RESULT_SUPERVISOR_DENIED,
        decision->reason);
      return false;
    }

    return true;
  }

  void forward_approval(
    const ReviewService::Request & request,
    const std::shared_ptr<ReviewService::Response> & response)
  {
    update_state(
      "processing",
      "forwarding_candidate_approval",
      request.request_id,
      request.candidate_id,
      "approve");

    if (!approval_client_->wait_for_service(
        timeout_duration(dependency_wait_timeout_s_)))
    {
      finish(
        request,
        response,
        ReviewService::Response::RESULT_REGISTRY_UNAVAILABLE,
        "candidate approval service unavailable");
      return;
    }

    auto approval = std::make_shared<Approve::Request>();
    approval->candidate_id = request.candidate_id;
    approval->expected_candidate_revision = request.expected_candidate_revision;
    approval->actor_id = request.actor_id;
    approval->location_id = request.location_id;
    approval->display_name = request.display_name;
    approval->aliases = request.aliases;
    approval->semantic_type = request.semantic_type;
    approval->approach_pose = request.approach_pose;
    approval->confirmation_pose_valid = request.confirmation_pose_valid;
    approval->confirmation_pose = request.confirmation_pose;
    approval->arrival_confirmation_required = request.arrival_confirmation_required;
    approval->building = request.building;
    approval->floor = request.floor;
    approval->area = request.area;
    approval->notes = request.notes;

    auto future = approval_client_->async_send_request(approval);
    if (!wait_until_ready(future, operation_timeout_s_)) {
      finish(
        request,
        response,
        ReviewService::Response::RESULT_TIMED_OUT,
        "candidate approval timed out");
      return;
    }

    const auto result = future.get();
    if (!result->approved) {
      finish(
        request,
        response,
        approval_failure_code(result->result_code),
        result->reason);
      return;
    }

    response->location = result->location;
    finish(
      request,
      response,
      ReviewService::Response::RESULT_APPROVED,
      result->reason,
      true,
      false);
  }

  void forward_rejection(
    const ReviewService::Request & request,
    const std::shared_ptr<ReviewService::Response> & response)
  {
    update_state(
      "processing",
      "forwarding_candidate_rejection",
      request.request_id,
      request.candidate_id,
      "reject");

    if (!rejection_client_->wait_for_service(
        timeout_duration(dependency_wait_timeout_s_)))
    {
      finish(
        request,
        response,
        ReviewService::Response::RESULT_REGISTRY_UNAVAILABLE,
        "candidate rejection service unavailable");
      return;
    }

    auto rejection = std::make_shared<Reject::Request>();
    rejection->candidate_id = request.candidate_id;
    rejection->expected_candidate_revision = request.expected_candidate_revision;
    rejection->actor_id = request.actor_id;
    rejection->rejection_reason = request.rejection_reason;

    auto future = rejection_client_->async_send_request(rejection);
    if (!wait_until_ready(future, operation_timeout_s_)) {
      finish(
        request,
        response,
        ReviewService::Response::RESULT_TIMED_OUT,
        "candidate rejection timed out");
      return;
    }

    const auto result = future.get();
    if (!result->rejected) {
      finish(
        request,
        response,
        rejection_failure_code(result->result_code),
        result->reason);
      return;
    }

    response->candidate = result->candidate;
    finish(
      request,
      response,
      ReviewService::Response::RESULT_REJECTED,
      result->reason,
      false,
      true);
  }

  [[nodiscard]] static std::uint8_t approval_failure_code(
    const std::uint8_t code)
  {
    if (code == Approve::Response::RESULT_CANDIDATE_NOT_FOUND) {
      return ReviewService::Response::RESULT_CANDIDATE_NOT_FOUND;
    }
    if (code == Approve::Response::RESULT_CANDIDATE_NOT_PENDING) {
      return ReviewService::Response::RESULT_CANDIDATE_NOT_PENDING;
    }
    if (code == Approve::Response::RESULT_STALE_REVISION) {
      return ReviewService::Response::RESULT_STALE_REVISION;
    }
    if (code == Approve::Response::RESULT_STORAGE_UNAVAILABLE) {
      return ReviewService::Response::RESULT_REGISTRY_UNAVAILABLE;
    }
    return ReviewService::Response::RESULT_REGISTRY_REJECTED;
  }

  [[nodiscard]] static std::uint8_t rejection_failure_code(
    const std::uint8_t code)
  {
    if (code == Reject::Response::RESULT_CANDIDATE_NOT_FOUND) {
      return ReviewService::Response::RESULT_CANDIDATE_NOT_FOUND;
    }
    if (code == Reject::Response::RESULT_CANDIDATE_NOT_PENDING) {
      return ReviewService::Response::RESULT_CANDIDATE_NOT_PENDING;
    }
    if (code == Reject::Response::RESULT_STALE_REVISION) {
      return ReviewService::Response::RESULT_STALE_REVISION;
    }
    if (code == Reject::Response::RESULT_STORAGE_UNAVAILABLE) {
      return ReviewService::Response::RESULT_REGISTRY_UNAVAILABLE;
    }
    return ReviewService::Response::RESULT_REGISTRY_REJECTED;
  }

  void finish(
    const ReviewService::Request & request,
    const std::shared_ptr<ReviewService::Response> & response,
    const std::uint8_t result_code,
    std::string reason,
    const bool approved = false,
    const bool rejected = false)
  {
    response->completed = approved || rejected;
    response->approved = approved;
    response->rejected = rejected;
    response->result_code = result_code;
    response->reason = std::move(reason);
    response->completed_at = now();

    if (response->completed) {
      reviews_committed_.fetch_add(1U);
    } else {
      reviews_failed_.fetch_add(1U);
    }

    update_state(
      "ready",
      response->reason,
      request.request_id,
      request.candidate_id,
      decision_text(request.decision));
    publish_result(request, *response);
    publish_status();
  }

  void update_state(
    std::string state,
    std::string reason,
    std::string request_id,
    std::string candidate_id,
    std::string decision)
  {
    std::lock_guard<std::mutex> lock{state_mutex_};
    state_ = std::move(state);
    reason_ = std::move(reason);
    last_request_id_ = std::move(request_id);
    last_candidate_id_ = std::move(candidate_id);
    last_decision_ = std::move(decision);
  }

  void publish_result(
    const ReviewService::Request & request,
    const ReviewService::Response & response)
  {
    Json event;
    event["schema_version"] = 1;
    event["sequence"] = ++result_sequence_;
    event["request_id"] = request.request_id;
    event["actor_id"] = request.actor_id;
    event["candidate_id"] = request.candidate_id;
    event["expected_candidate_revision"] = request.expected_candidate_revision;
    event["decision"] = decision_text(request.decision);
    event["completed"] = response.completed;
    event["approved"] = response.approved;
    event["rejected"] = response.rejected;
    event["result_code"] = response.result_code;
    event["reason"] = response.reason;
    event["location_id"] = response.location.location_id;

    std_msgs::msg::String message;
    message.data = event.dump();
    result_publisher_->publish(message);
  }

  void publish_status()
  {
    Json status;
    {
      std::lock_guard<std::mutex> lock{state_mutex_};
      status["schema_version"] = 1;
      status["lifecycle"] = "active";
      status["state"] = state_;
      status["reason"] = reason_;
      status["last_request_id"] = last_request_id_;
      status["last_candidate_id"] = last_candidate_id_;
      status["last_decision"] = last_decision_;
    }
    status["service"] = service_name_;
    status["reviews_started"] = reviews_started_.load();
    status["reviews_committed"] = reviews_committed_.load();
    status["reviews_failed"] = reviews_failed_.load();
    status["authorization_denials"] = authorization_denials_.load();

    std_msgs::msg::String message;
    message.data = status.dump();
    status_publisher_->publish(message);
  }

  void publish_heartbeat()
  {
    std_msgs::msg::UInt64 message;
    message.data = ++heartbeat_sequence_;
    heartbeat_publisher_->publish(message);
  }

  std::string service_name_{};
  std::string candidate_lookup_service_name_{};
  std::string authorization_service_name_{};
  std::string approval_service_name_{};
  std::string rejection_service_name_{};
  std::string status_topic_{};
  std::string result_topic_{};
  std::string heartbeat_topic_{};
  double dependency_wait_timeout_s_{2.0};
  double operation_timeout_s_{5.0};
  double status_publish_hz_{1.0};
  double heartbeat_publish_hz_{2.0};

  std::mutex operation_mutex_{};
  std::mutex state_mutex_{};
  std::string state_{"starting"};
  std::string reason_{"startup_pending"};
  std::string last_request_id_{};
  std::string last_candidate_id_{};
  std::string last_decision_{"none"};

  std::atomic<std::uint64_t> result_sequence_{0U};
  std::atomic<std::uint64_t> heartbeat_sequence_{0U};
  std::atomic<std::uint64_t> reviews_started_{0U};
  std::atomic<std::uint64_t> reviews_committed_{0U};
  std::atomic<std::uint64_t> reviews_failed_{0U};
  std::atomic<std::uint64_t> authorization_denials_{0U};

  rclcpp::CallbackGroup::SharedPtr callback_group_{};
  rclcpp::Client<GetCandidate>::SharedPtr candidate_client_{};
  rclcpp::Client<Authorize>::SharedPtr authorization_client_{};
  rclcpp::Client<Approve>::SharedPtr approval_client_{};
  rclcpp::Client<Reject>::SharedPtr rejection_client_{};
  rclcpp::Service<ReviewService>::SharedPtr service_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_{};
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr heartbeat_publisher_{};
  rclcpp::TimerBase::SharedPtr status_timer_{};
  rclcpp::TimerBase::SharedPtr heartbeat_timer_{};
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_mapping::LocationReviewGatewayNode>();
  rclcpp::executors::MultiThreadedExecutor executor{
    rclcpp::ExecutorOptions(), 4U};
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
