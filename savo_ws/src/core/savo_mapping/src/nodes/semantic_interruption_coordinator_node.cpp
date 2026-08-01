// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/action/register_mapped_location.hpp"
#include "savo_msgs/msg/april_tag_observation.hpp"
#include "savo_msgs/msg/autonomous_mapping_status.hpp"
#include "savo_msgs/msg/location_event.hpp"
#include "savo_msgs/msg/semantic_interruption_status.hpp"
#include "savo_msgs/srv/control_autonomous_mapping.hpp"
#include "savo_msgs/srv/list_location_candidates.hpp"
#include "savo_msgs/srv/list_locations.hpp"
#include "savo_msgs/srv/submit_semantic_location.hpp"

#include "savo_mapping/semantic_interruption.hpp"

namespace savo_mapping
{
namespace
{

using Observation = savo_msgs::msg::AprilTagObservation;
using MissionStatus = savo_msgs::msg::AutonomousMappingStatus;
using InterruptionStatus = savo_msgs::msg::SemanticInterruptionStatus;
using LocationEvent = savo_msgs::msg::LocationEvent;
using ControlMission = savo_msgs::srv::ControlAutonomousMapping;
using SubmitSemantic = savo_msgs::srv::SubmitSemanticLocation;
using ListCandidates = savo_msgs::srv::ListLocationCandidates;
using ListLocations = savo_msgs::srv::ListLocations;
using RegisterLocation = savo_msgs::action::RegisterMappedLocation;
using RegisterGoalHandle = rclcpp_action::ClientGoalHandle<RegisterLocation>;

constexpr double kNanosecondsPerSecond = 1.0e9;

[[nodiscard]] std::int64_t time_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] std::int64_t duration_ns(const double seconds)
{
  return static_cast<std::int64_t>(seconds * kNanosecondsPerSecond);
}

[[nodiscard]] double duration_seconds(
  const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) / kNanosecondsPerSecond;
}

[[nodiscard]] builtin_interfaces::msg::Duration to_duration(const double seconds)
{
  builtin_interfaces::msg::Duration output;
  const double bounded = std::max(0.0, seconds);
  output.sec = static_cast<std::int32_t>(std::floor(bounded));
  output.nanosec = static_cast<std::uint32_t>(
    (bounded - static_cast<double>(output.sec)) * kNanosecondsPerSecond);
  return output;
}

[[nodiscard]] bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) &&
         std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) &&
         std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

[[nodiscard]] bool valid_optional_map_pose(
  const bool valid,
  const geometry_msgs::msg::PoseStamped & pose)
{
  return !valid || (pose.header.frame_id == "map" && finite_pose(pose.pose));
}

[[nodiscard]] std::string sanitize_identifier(const std::string & input)
{
  std::string output;
  output.reserve(std::min<std::size_t>(input.size(), 48U));
  for (const unsigned char character : input) {
    if (output.size() >= 48U) {
      break;
    }
    if (std::isalnum(character) || character == '_') {
      output.push_back(static_cast<char>(character));
    } else {
      output.push_back('_');
    }
  }
  return output.empty() ? "mission" : output;
}

[[nodiscard]] std::string candidate_id_for(
  const std::string & mission_id,
  const std::string & family,
  const std::int32_t tag_id)
{
  return "am6_" + sanitize_identifier(mission_id) + "_" +
         sanitize_identifier(family) + "_" + std::to_string(tag_id);
}

[[nodiscard]] std::string registry_key(
  const std::string & map_id,
  const std::uint32_t map_revision,
  const std::string & family,
  const std::int32_t tag_id)
{
  return map_id + "|" + std::to_string(map_revision) + "|" + family + "|" +
         std::to_string(tag_id);
}

}  // namespace

class SemanticInterruptionCoordinatorNode final : public rclcpp::Node
{
public:
  SemanticInterruptionCoordinatorNode()
  : Node("semantic_interruption_coordinator_node"),
    core_(load_config())
  {
    registration_timeout_s_ = get_parameter("registration_timeout_s").as_double();
    observation_topic_ = declare_parameter<std::string>(
      "observation_topic", "/savo_head/apriltag/observations");
    mission_status_topic_ = declare_parameter<std::string>(
      "mission_status_topic", "/savo_mapping/autonomous/status");
    mission_control_service_ = declare_parameter<std::string>(
      "mission_control_service", "/savo_mapping/autonomous/control");
    registration_action_ = declare_parameter<std::string>(
      "registration_action", "/savo_mapping/locations/register");
    location_events_topic_ = declare_parameter<std::string>(
      "location_events_topic", "/savo_locations/events");
    candidates_list_service_ = declare_parameter<std::string>(
      "candidates_list_service", "/savo_locations/candidates/list");
    locations_list_service_ = declare_parameter<std::string>(
      "locations_list_service", "/savo_locations/list");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/savo_mapping/semantic_interruption/status");
    semantic_submit_service_ = declare_parameter<std::string>(
      "semantic_submit_service", "/savo_mapping/semantic_interruption/submit");
    actor_id_ = declare_parameter<std::string>(
      "actor_id", "savo_mapping.semantic_interruption");

    if (observation_topic_.empty() || mission_status_topic_.empty() ||
      mission_control_service_.empty() || registration_action_.empty() ||
      location_events_topic_.empty() || candidates_list_service_.empty() ||
      locations_list_service_.empty() || status_topic_.empty() ||
      semantic_submit_service_.empty() || actor_id_.empty())
    {
      throw std::invalid_argument("semantic interruption endpoint names must not be empty");
    }

    status_publisher_ = create_publisher<InterruptionStatus>(
      status_topic_, rclcpp::QoS(1).reliable().transient_local());
    observation_subscription_ = create_subscription<Observation>(
      observation_topic_, rclcpp::SensorDataQoS(),
      std::bind(
        &SemanticInterruptionCoordinatorNode::on_observation,
        this, std::placeholders::_1));
    mission_subscription_ = create_subscription<MissionStatus>(
      mission_status_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &SemanticInterruptionCoordinatorNode::on_mission_status,
        this, std::placeholders::_1));
    location_event_subscription_ = create_subscription<LocationEvent>(
      location_events_topic_, rclcpp::QoS(100).reliable().durability_volatile(),
      std::bind(
        &SemanticInterruptionCoordinatorNode::on_location_event,
        this, std::placeholders::_1));

    control_client_ = create_client<ControlMission>(mission_control_service_);
    candidates_client_ = create_client<ListCandidates>(candidates_list_service_);
    locations_client_ = create_client<ListLocations>(locations_list_service_);
    registration_client_ = rclcpp_action::create_client<RegisterLocation>(
      this, registration_action_);
    submit_service_ = create_service<SubmitSemantic>(
      semantic_submit_service_,
      std::bind(
        &SemanticInterruptionCoordinatorNode::on_semantic_submission,
        this, std::placeholders::_1, std::placeholders::_2));

    evaluation_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SemanticInterruptionCoordinatorNode::on_timer, this));

    publish_status();
    RCLCPP_INFO(
      get_logger(),
      "AM-6 semantic interruption ready observations=%s control=%s registration=%s",
      observation_topic_.c_str(), mission_control_service_.c_str(),
      registration_action_.c_str());
  }

private:
  SemanticInterruptionConfig load_config()
  {
    SemanticInterruptionConfig config;
    config.enabled = declare_parameter<bool>("enabled", true);
    config.expected_family = declare_parameter<std::string>(
      "expected_family", "tag36h11");
    config.minimum_detection_quality = declare_parameter<double>(
      "minimum_detection_quality", 0.70);
    config.observation_timeout_ns = duration_ns(declare_parameter<double>(
        "observation_timeout_s", 0.50));
    config.mission_status_timeout_ns = duration_ns(declare_parameter<double>(
        "mission_status_timeout_s", 2.0));
    config.pause_timeout_ns = duration_ns(declare_parameter<double>(
        "pause_timeout_s", 10.0));
    config.semantic_input_timeout_ns = duration_ns(declare_parameter<double>(
        "semantic_input_timeout_s", 300.0));
    registration_timeout_s_ = declare_parameter<double>(
      "registration_timeout_s", 30.0);
    config.registration_timeout_ns = duration_ns(registration_timeout_s_);
    config.resume_timeout_ns = duration_ns(declare_parameter<double>(
        "resume_timeout_s", 10.0));
    config.duplicate_cooldown_ns = duration_ns(declare_parameter<double>(
        "duplicate_cooldown_s", 30.0));
    const auto failure_policy = declare_parameter<std::string>(
      "failure_policy", "remain_paused");
    if (failure_policy == "remain_paused") {
      config.failure_policy = SemanticFailurePolicy::RemainPaused;
    } else if (failure_policy == "request_resume") {
      config.failure_policy = SemanticFailurePolicy::RequestResume;
    } else {
      throw std::invalid_argument(
              "failure_policy must be remain_paused or request_resume");
    }
    return config;
  }

  [[nodiscard]] SemanticMissionContext mission_context_locked() const
  {
    SemanticMissionContext context;
    if (!latest_mission_.has_value()) {
      return context;
    }
    context.received = true;
    context.active = latest_mission_->active;
    context.mission_id = latest_mission_->mission_id;
    context.map_id = latest_mission_->map_id;
    context.map_revision = latest_mission_->map_revision;
    context.state = latest_mission_->state;
    context.mission_start_ns = mission_start_ns_;
    context.received_ns = mission_received_ns_;
    return context;
  }

  void on_mission_status(const MissionStatus::SharedPtr message)
  {
    bool refresh_registry = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto current_ns = now().nanoseconds();
      if (!latest_mission_.has_value() ||
        latest_mission_->mission_id != message->mission_id)
      {
        const auto stamped_ns = time_ns(message->stamp);
        mission_start_ns_ = stamped_ns > 0 ? stamped_ns : current_ns;
        refresh_registry = message->active && !message->map_id.empty() &&
          message->map_revision > 0U;
      }
      latest_mission_ = *message;
      mission_received_ns_ = current_ns;
      core_.UpdateMission(mission_context_locked(), current_ns);
    }
    publish_status();
    if (refresh_registry) {
      refresh_known_tags();
    }
  }

  void on_observation(const Observation::SharedPtr message)
  {
    bool accepted = false;
    bool status_changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      SemanticObservationEvidence evidence;
      evidence.detected = true;
      evidence.pose_valid = message->pose_valid;
      evidence.pose_finite = finite_pose(message->pose.pose);
      evidence.family = message->family;
      evidence.tag_id = message->tag_id;
      evidence.detection_quality = message->detection_quality;
      evidence.observation_sequence = message->observation_sequence;
      evidence.stamp_ns = time_ns(message->header.stamp);

      const auto mission = mission_context_locked();
      const bool known = known_tags_.count(registry_key(
          mission.map_id, mission.map_revision, evidence.family, evidence.tag_id)) > 0U;
      const auto suppressed_before =
        core_.snapshot().duplicate_observations_suppressed;
      const auto result = core_.Observe(evidence, mission, known, now().nanoseconds());
      accepted = result.accepted;
      status_changed = accepted ||
        core_.snapshot().duplicate_observations_suppressed != suppressed_before;
      if (accepted) {
        detector_observation_ = *message;
      }
    }

    if (!accepted) {
      if (status_changed) {
        publish_status();
      }
      return;
    }

    publish_status();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      core_.BeginPause(now().nanoseconds());
    }
    publish_status();
    request_control(ControlMission::Request::COMMAND_PAUSE, "semantic_tag_detected");
  }

  void on_location_event(const LocationEvent::SharedPtr message)
  {
    if (message->event_type == LocationEvent::EVENT_CANDIDATE_REGISTERED ||
      message->event_type == LocationEvent::EVENT_LOCATION_APPROVED ||
      message->event_type == LocationEvent::EVENT_LOCATION_UPDATED ||
      message->event_type == LocationEvent::EVENT_LOCATION_RETIRED)
    {
      refresh_known_tags();
    }
  }

  void on_semantic_submission(
    const std::shared_ptr<SubmitSemantic::Request> request,
    std::shared_ptr<SubmitSemantic::Response> response)
  {
    if (request->contract_version != SubmitSemantic::Request::CONTRACT_VERSION) {
      response->accepted = false;
      response->result_code = SubmitSemantic::Response::RESULT_INVALID_REQUEST;
      response->reason = "unsupported_semantic_submission_contract_version";
      return;
    }

    RegisterLocation::Goal goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      SemanticSubmissionEvidence submission;
      submission.mission_id = request->mission_id;
      submission.actor_id = request->actor_id;
      submission.tag_family = request->tag_family;
      submission.tag_id = request->tag_id;
      submission.suggested_location_id = request->suggested_location_id;
      submission.suggested_display_name = request->suggested_display_name;
      submission.suggested_semantic_type = request->suggested_semantic_type;
      submission.approach_pose_valid = request->approach_pose_valid;
      submission.approach_pose_acceptable = valid_optional_map_pose(
        request->approach_pose_valid, request->approach_pose);
      submission.confirmation_pose_valid = request->confirmation_pose_valid;
      submission.confirmation_pose_acceptable = valid_optional_map_pose(
        request->confirmation_pose_valid, request->confirmation_pose);

      const auto validation = core_.ValidateSubmission(submission);
      if (!validation.accepted) {
        response->accepted = false;
        response->result_code = validation.result_code;
        response->reason = validation.reason;
        return;
      }

      const auto & snapshot = core_.snapshot();
      const auto candidate_id = candidate_id_for(
        snapshot.mission_id, snapshot.tag_family, snapshot.tag_id);
      core_.BeginRegistration(candidate_id, now().nanoseconds());

      goal.request_id = "am6_register_" + candidate_id;
      goal.actor_id = request->actor_id;
      goal.candidate_id = candidate_id;
      goal.expected_family = snapshot.tag_family;
      goal.expected_tag_id = snapshot.tag_id;
      goal.map_id = snapshot.map_id;
      goal.map_revision = snapshot.map_revision;
      goal.suggested_location_id = request->suggested_location_id;
      goal.suggested_display_name = request->suggested_display_name;
      goal.suggested_aliases = request->suggested_aliases;
      goal.suggested_semantic_type = request->suggested_semantic_type;
      goal.building = request->building;
      goal.floor = request->floor;
      goal.area = request->area;
      goal.notes = request->notes;
      goal.approach_pose_valid = request->approach_pose_valid;
      goal.approach_pose = request->approach_pose;
      goal.confirmation_pose_valid = request->confirmation_pose_valid;
      goal.confirmation_pose = request->confirmation_pose;
      goal.source_session_id = snapshot.mission_id;
      const double requested_timeout_s = duration_seconds(request->timeout);
      goal.timeout = to_duration(
        requested_timeout_s > 0.0 ?
        std::min(requested_timeout_s, registration_timeout_s_) :
        registration_timeout_s_);

      response->accepted = true;
      response->result_code = SubmitSemantic::Response::RESULT_ACCEPTED;
      response->reason = "semantic_submission_accepted";
      response->candidate_id = candidate_id;
    }

    publish_status();
    start_registration(goal);
  }

  void request_control(const std::uint8_t command, const std::string & reason)
  {
    if (!control_client_->service_is_ready()) {
      fail("autonomous_mapping_control_service_unavailable", command !=
        ControlMission::Request::COMMAND_RESUME);
      return;
    }

    auto request = std::make_shared<ControlMission::Request>();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      request->contract_version = ControlMission::Request::CONTRACT_VERSION;
      request->mission_id = core_.snapshot().mission_id;
      request->actor_id = actor_id_;
      request->command = command;
      request->reason = reason;
    }

    control_client_->async_send_request(
      request,
      [this, command](rclcpp::Client<ControlMission>::SharedFuture future)
      {
        const auto response = future.get();
        if (!response->accepted) {
          fail(
            command == ControlMission::Request::COMMAND_PAUSE ?
            "autonomous_mapping_pause_rejected:" + response->reason :
            "autonomous_mapping_resume_rejected:" + response->reason,
            command != ControlMission::Request::COMMAND_RESUME);
          return;
        }

        if (command == ControlMission::Request::COMMAND_PAUSE) {
          std::lock_guard<std::mutex> lock(mutex_);
          core_.PauseAccepted(now().nanoseconds());
        }
        publish_status();
      });
  }

  void start_registration(const RegisterLocation::Goal & goal)
  {
    if (!registration_client_->action_server_is_ready()) {
      fail("mapped_location_registration_action_unavailable", true);
      return;
    }

    rclcpp_action::Client<RegisterLocation>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const RegisterGoalHandle::SharedPtr & goal_handle)
      {
        if (!goal_handle) {
          fail("mapped_location_registration_goal_rejected", true);
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!core_.snapshot().active ||
          core_.snapshot().state != SemanticInterruptionState::Registering)
        {
          registration_client_->async_cancel_goal(goal_handle);
          return;
        }
        registration_goal_handle_ = goal_handle;
      };
    options.result_callback =
      [this](const RegisterGoalHandle::WrappedResult & wrapped)
      {
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
          !wrapped.result || !wrapped.result->registered)
        {
          const std::string reason = wrapped.result ? wrapped.result->reason :
            "mapped_location_registration_failed_without_result";
          fail("mapped_location_registration_failed:" + reason, true);
          return;
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!core_.snapshot().active ||
            core_.snapshot().state != SemanticInterruptionState::Registering)
          {
            return;
          }
          location_candidate_ = wrapped.result->candidate;
          registration_goal_handle_.reset();
          core_.RegistrationSucceeded(
            wrapped.result->candidate.candidate_id, now().nanoseconds());
          core_.BeginResume(now().nanoseconds());
        }
        publish_status();
        request_control(
          ControlMission::Request::COMMAND_RESUME,
          "semantic_location_candidate_registered");
      };
    registration_client_->async_send_goal(goal, options);
  }

  void fail(const std::string & reason, const bool allow_failure_resume)
  {
    bool request_resume = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      core_.Fail(reason, now().nanoseconds());
      registration_goal_handle_.reset();
      request_resume = allow_failure_resume &&
        core_.should_resume_after_failure() && core_.snapshot().active;
    }
    publish_status();

    if (request_resume) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        core_.BeginFailureResume(now().nanoseconds());
      }
      publish_status();
      request_control(
        ControlMission::Request::COMMAND_RESUME,
        "semantic_interruption_failure_policy");
    }
  }

  void on_timer()
  {
    bool timed_out = false;
    bool cancel_registration = false;
    bool failure_resume = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto previous_state = core_.snapshot().state;
      timed_out = core_.Tick(now().nanoseconds());
      cancel_registration = timed_out &&
        previous_state == SemanticInterruptionState::Registering &&
        registration_goal_handle_ != nullptr;
      failure_resume = timed_out && core_.should_resume_after_failure() &&
        core_.snapshot().active &&
        previous_state != SemanticInterruptionState::Resuming;
      if (cancel_registration) {
        registration_client_->async_cancel_goal(registration_goal_handle_);
        registration_goal_handle_.reset();
      }
    }

    if (timed_out) {
      publish_status();
      if (failure_resume) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          core_.BeginFailureResume(now().nanoseconds());
        }
        publish_status();
        request_control(
          ControlMission::Request::COMMAND_RESUME,
          "semantic_interruption_timeout_policy");
      }
    }
  }

  void refresh_known_tags()
  {
    std::string map_id;
    std::uint32_t map_revision = 0U;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto mission = mission_context_locked();
      map_id = mission.map_id;
      map_revision = mission.map_revision;
    }
    if (map_id.empty() || map_revision == 0U) {
      return;
    }

    if (candidates_client_->service_is_ready()) {
      auto request = std::make_shared<ListCandidates::Request>();
      request->state_filter = ListCandidates::Request::STATE_FILTER_ALL;
      request->enforce_map_context = true;
      request->map_id = map_id;
      request->map_revision = map_revision;
      candidates_client_->async_send_request(
        request,
        [this](rclcpp::Client<ListCandidates>::SharedFuture future)
        {
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          for (const auto & candidate : response->candidates) {
            if (candidate.tag_id >= 0 && !candidate.tag_family.empty()) {
              known_tags_.insert(registry_key(
                  candidate.map_id, candidate.map_revision,
                  candidate.tag_family, candidate.tag_id));
            }
          }
        });
    }

    if (locations_client_->service_is_ready()) {
      auto request = std::make_shared<ListLocations::Request>();
      request->map_id = map_id;
      request->map_revision = map_revision;
      request->enforce_map_context = true;
      request->state_filter = ListLocations::Request::STATE_ANY;
      request->enabled_only = false;
      locations_client_->async_send_request(
        request,
        [this](rclcpp::Client<ListLocations>::SharedFuture future)
        {
          const auto response = future.get();
          if (!response->success) {
            return;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          for (const auto & location : response->locations) {
            if (location.tag_id >= 0 && !location.tag_family.empty()) {
              known_tags_.insert(registry_key(
                  location.map_id, location.map_revision,
                  location.tag_family, location.tag_id));
            }
          }
        });
    }
  }

  void publish_status()
  {
    InterruptionStatus message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto & snapshot = core_.snapshot();
      message.contract_version = InterruptionStatus::CONTRACT_VERSION;
      message.header.stamp = now();
      message.header.frame_id = "map";
      message.state = static_cast<std::uint8_t>(snapshot.state);
      message.state_text = SemanticInterruptionCore::StateText(snapshot.state);
      message.reason = snapshot.reason;
      message.active = snapshot.active;
      message.mission_id = snapshot.mission_id;
      message.map_id = snapshot.map_id;
      message.map_revision = snapshot.map_revision;
      message.tag_family = snapshot.tag_family;
      message.tag_id = snapshot.tag_id;
      message.observation_sequence = snapshot.observation_sequence;
      message.duplicate_observations_suppressed =
        snapshot.duplicate_observations_suppressed;
      message.detector_observation = detector_observation_;
      message.candidate_id = snapshot.candidate_id;
      message.location_candidate = location_candidate_;
      message.semantic_submission_received =
        snapshot.semantic_submission_received;
      message.registration_started = snapshot.registration_started;
      message.registration_complete = snapshot.registration_complete;
      message.resume_requested = snapshot.resume_requested;
      message.resume_complete = snapshot.resume_complete;
    }
    status_publisher_->publish(message);
  }

  std::mutex mutex_{};
  SemanticInterruptionCore core_;
  std::optional<MissionStatus> latest_mission_{};
  std::int64_t mission_received_ns_{0};
  std::int64_t mission_start_ns_{0};
  Observation detector_observation_{};
  savo_msgs::msg::LocationCandidate location_candidate_{};
  std::set<std::string> known_tags_{};
  RegisterGoalHandle::SharedPtr registration_goal_handle_{};

  double registration_timeout_s_{30.0};
  std::string observation_topic_{};
  std::string mission_status_topic_{};
  std::string mission_control_service_{};
  std::string registration_action_{};
  std::string location_events_topic_{};
  std::string candidates_list_service_{};
  std::string locations_list_service_{};
  std::string status_topic_{};
  std::string semantic_submit_service_{};
  std::string actor_id_{};

  rclcpp::Publisher<InterruptionStatus>::SharedPtr status_publisher_{};
  rclcpp::Subscription<Observation>::SharedPtr observation_subscription_{};
  rclcpp::Subscription<MissionStatus>::SharedPtr mission_subscription_{};
  rclcpp::Subscription<LocationEvent>::SharedPtr location_event_subscription_{};
  rclcpp::Client<ControlMission>::SharedPtr control_client_{};
  rclcpp::Client<ListCandidates>::SharedPtr candidates_client_{};
  rclcpp::Client<ListLocations>::SharedPtr locations_client_{};
  rclcpp_action::Client<RegisterLocation>::SharedPtr registration_client_{};
  rclcpp::Service<SubmitSemantic>::SharedPtr submit_service_{};
  rclcpp::TimerBase::SharedPtr evaluation_timer_{};
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<savo_mapping::SemanticInterruptionCoordinatorNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
