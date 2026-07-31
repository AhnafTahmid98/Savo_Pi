// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_mapping/autonomous_mapping_mission.hpp"
#include "savo_mapping/frontier_completion_detector.hpp"
#include "savo_mapping/saved_map_contract.hpp"
#include "savo_mapping/topic_names.hpp"
#include "savo_mapping/tf_pose_reader.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <savo_msgs/action/run_autonomous_mapping.hpp>
#include <savo_msgs/msg/autonomous_mapping_status.hpp>
#include <savo_msgs/msg/frontier_exploration_status.hpp>
#include <savo_msgs/srv/control_autonomous_mapping.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace savo_mapping
{
namespace
{

using RunMission = savo_msgs::action::RunAutonomousMapping;
using GoalHandle = rclcpp_action::ServerGoalHandle<RunMission>;
using ControlMission = savo_msgs::srv::ControlAutonomousMapping;
using MissionStatus = savo_msgs::msg::AutonomousMappingStatus;
using FrontierStatus = savo_msgs::msg::FrontierExplorationStatus;
using Trigger = std_srvs::srv::Trigger;
using StringMessage = std_msgs::msg::String;
using BoolMessage = std_msgs::msg::Bool;

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;

bool duration_valid(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec >= 0 &&
         duration.nanosec <
         static_cast<std::uint32_t>(kNanosecondsPerSecond);
}

double duration_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) +
         static_cast<double>(duration.nanosec) /
         static_cast<double>(kNanosecondsPerSecond);
}

double steady_seconds(
  const std::chrono::steady_clock::time_point & time_point)
{
  return std::chrono::duration<double>(
    time_point.time_since_epoch()).count();
}

autonomous::MissionCommand command_from_message(const std::uint8_t command)
{
  switch (command) {
    case ControlMission::Request::COMMAND_PAUSE:
      return autonomous::MissionCommand::Pause;
    case ControlMission::Request::COMMAND_RESUME:
      return autonomous::MissionCommand::Resume;
    case ControlMission::Request::COMMAND_CANCEL:
      return autonomous::MissionCommand::Cancel;
    case ControlMission::Request::COMMAND_REQUEST_SCAN360:
      return autonomous::MissionCommand::RequestScan360;
    default:
      throw std::invalid_argument("unknown_autonomous_mapping_command");
  }
}

bool known_handoff_state(const std::string & state)
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

std::optional<std::string> saved_session_directory(
  const std::string & response_message)
{
  constexpr std::string_view prefix{"map_session_saved:"};

  if (response_message.rfind(prefix, 0) != 0) {
    return std::nullopt;
  }

  const std::string directory =
    response_message.substr(prefix.size());

  if (directory.empty()) {
    return std::nullopt;
  }

  return directory;
}

std::string state_field(const std::string & text)
{
  constexpr std::string_view prefix{"state="};
  const auto start = text.find(prefix);
  if (start == std::string::npos) {
    return text;
  }

  const auto value_start = start + prefix.size();
  const auto end = text.find(';', value_start);
  return text.substr(value_start, end - value_start);
}

bool scan360_state_is_active(const std::string_view state)
{
  return
    state == "ready" ||
    state == "command_pending" ||
    state == "rotating" ||
    state == "settling" ||
    state == "canceling";
}

}  // namespace

class AutonomousMappingOrchestratorNode final : public rclcpp::Node
{
public:
  AutonomousMappingOrchestratorNode()
  : Node("autonomous_mapping_orchestrator_node")
  {
    declare_and_validate_parameters();
    create_pose_reader();
    create_interfaces();

    evaluation_timer_ = create_wall_timer(
      std::chrono::milliseconds(evaluation_period_ms_),
      std::bind(
        &AutonomousMappingOrchestratorNode::evaluate_and_apply,
        this));

    evaluate_and_apply();

    RCLCPP_INFO(
      get_logger(),
      "autonomous mapping orchestrator ready action=%s control=%s",
      action_name_.c_str(),
      control_service_name_.c_str());
  }

private:
  void declare_and_validate_parameters()
  {
    action_name_ = declare_parameter<std::string>(
      "action_name",
      "/savo_mapping/autonomous/run");

    control_service_name_ = declare_parameter<std::string>(
      "control_service",
      "/savo_mapping/autonomous/control");

    status_topic_ = declare_parameter<std::string>(
      "status_topic",
      std::string{topics::AUTONOMOUS_MISSION_STATUS});

    mode_topic_ = declare_parameter<std::string>(
      "mode_topic",
      std::string{topics::MODE});

    exploration_mode_topic_ = declare_parameter<std::string>(
      "exploration_mode_topic",
      std::string{topics::EXPLORATION_MODE});

    workflow_phase_topic_ = declare_parameter<std::string>(
      "workflow_phase_topic",
      std::string{topics::WORKFLOW_PHASE});

    session_state_topic_ = declare_parameter<std::string>(
      "session_state_topic",
      std::string{topics::SESSION_STATE});

    readiness_topic_ = declare_parameter<std::string>(
      "readiness_topic",
      std::string{topics::READINESS});

    safety_stop_topic_ = declare_parameter<std::string>(
      "safety_stop_topic",
      std::string{topics::SAFETY_STOP});

    runtime_authority_topic_ = declare_parameter<std::string>(
      "runtime_authority_topic",
      std::string{topics::EXPLORATION_RUNTIME_ENABLED});

    handoff_state_topic_ = declare_parameter<std::string>(
      "handoff_state_topic",
      std::string{topics::EXPLORATION_GOAL_STATE});

    frontier_status_topic_ = declare_parameter<std::string>(
      "frontier_status_topic",
      std::string{topics::FRONTIER_EXPLORER_STATUS});

    scan360_state_topic_ = declare_parameter<std::string>(
      "sequence.scan360_state_topic",
      "/savo_mapping/scan360/state");

    head_scan_state_topic_ = declare_parameter<std::string>(
      "sequence.head_scan_state_topic",
      "/savo_head/scan_state");

    scan360_start_service_name_ = declare_parameter<std::string>(
      "sequence.scan360_start_service",
      "/savo_mapping/scan360/start");

    scan360_cancel_service_name_ = declare_parameter<std::string>(
      "sequence.scan360_cancel_service",
      "/savo_mapping/scan360/cancel");

    head_scan_start_service_name_ = declare_parameter<std::string>(
      "sequence.head_scan_start_service",
      "/savo_head/start_scan");

    head_scan_pause_service_name_ = declare_parameter<std::string>(
      "sequence.head_scan_pause_service",
      "/savo_head/pause_scan");

    head_scan_resume_service_name_ = declare_parameter<std::string>(
      "sequence.head_scan_resume_service",
      "/savo_head/resume_scan");

    inputs_.require_start_pose_capture = declare_parameter<bool>(
      "sequence.require_start_pose_capture",
      true);

    inputs_.require_initial_scan360 = declare_parameter<bool>(
      "sequence.require_initial_scan360",
      true);

    inputs_.require_initial_head_scan = declare_parameter<bool>(
      "sequence.require_initial_head_scan",
      true);

    start_pose_target_frame_ = declare_parameter<std::string>(
      "sequence.start_pose_target_frame",
      "map");

    start_pose_source_frame_ = declare_parameter<std::string>(
      "sequence.start_pose_source_frame",
      "base_link");

    start_pose_lookup_timeout_s_ = declare_parameter<double>(
      "sequence.start_pose_lookup_timeout_s",
      0.20);

    start_pose_stale_timeout_s_ = declare_parameter<double>(
      "sequence.start_pose_stale_timeout_s",
      1.0);

    start_pose_operation_timeout_s_ = declare_parameter<double>(
      "sequence.start_pose_operation_timeout_s",
      10.0);

    scan360_operation_timeout_s_ = declare_parameter<double>(
      "sequence.scan360_operation_timeout_s",
      180.0);

    head_scan_operation_timeout_s_ = declare_parameter<double>(
      "sequence.head_scan_operation_timeout_s",
      180.0);

    mode_command_topic_ = declare_parameter<std::string>(
      "mode_command_topic",
      std::string{topics::MODE_CMD});

    start_session_command_topic_ = declare_parameter<std::string>(
      "start_session_command_topic",
      std::string{topics::START_SESSION_CMD});

    cancel_session_command_topic_ = declare_parameter<std::string>(
      "cancel_session_command_topic",
      std::string{topics::CANCEL_SESSION_CMD});

    handoff_cancel_service_name_ = declare_parameter<std::string>(
      "handoff_cancel_service",
      "/savo_mapping/exploration_goal/cancel");

    map_save_service_name_ = declare_parameter<std::string>(
      "save.map_session_service",
      "/savo_mapping/map_session/save");

    map_save_operation_timeout_s_ = declare_parameter<double>(
      "save.operation_timeout_s",
      45.0);

    saved_map_expected_frame_ = declare_parameter<std::string>(
      "save.expected_frame",
      "map");

    evaluation_period_ms_ = declare_parameter<std::int64_t>(
      "evaluation_period_ms",
      250);

    command_retry_period_ms_ = declare_parameter<std::int64_t>(
      "command_retry_period_ms",
      1000);

    default_mission_timeout_s_ = declare_parameter<double>(
      "default_mission_timeout_s",
      0.0);

    autonomous::FrontierCompletionConfig completion_config;
    const std::int64_t minimum_exhaustion_observations =
      declare_parameter<std::int64_t>(
        "completion.minimum_exhaustion_observations",
        3);
    if (
      minimum_exhaustion_observations <= 0 ||
      minimum_exhaustion_observations > 1000000)
    {
      throw std::invalid_argument(
              "minimum_exhaustion_observations_out_of_range");
    }
    completion_config.minimum_observations =
      static_cast<std::uint32_t>(minimum_exhaustion_observations);
    completion_config.minimum_stable_duration_s =
      declare_parameter<double>(
        "completion.minimum_stable_duration_s",
        5.0);
    completion_config.status_timeout_s =
      declare_parameter<double>(
        "completion.frontier_status_timeout_s",
        3.0);
    completion_config.allow_no_frontiers =
      declare_parameter<bool>(
        "completion.allow_no_frontiers",
        true);
    completion_config.allow_no_reachable_frontiers =
      declare_parameter<bool>(
        "completion.allow_no_reachable_frontiers",
        true);
    completion_config.allow_no_selectable_frontier =
      declare_parameter<bool>(
        "completion.allow_no_selectable_frontier",
        false);

    const std::string completion_config_error =
      autonomous::validate_frontier_completion_config(completion_config);
    if (!completion_config_error.empty()) {
      throw std::invalid_argument(completion_config_error);
    }
    completion_detector_ =
      autonomous::FrontierCompletionDetector(completion_config);

    const std::string * endpoint_values[] = {
      &action_name_,
      &control_service_name_,
      &status_topic_,
      &mode_topic_,
      &exploration_mode_topic_,
      &workflow_phase_topic_,
      &session_state_topic_,
      &readiness_topic_,
      &safety_stop_topic_,
      &runtime_authority_topic_,
      &handoff_state_topic_,
      &frontier_status_topic_,
      &scan360_state_topic_,
      &head_scan_state_topic_,
      &scan360_start_service_name_,
      &scan360_cancel_service_name_,
      &head_scan_start_service_name_,
      &head_scan_pause_service_name_,
      &head_scan_resume_service_name_,
      &start_pose_target_frame_,
      &start_pose_source_frame_,
      &mode_command_topic_,
      &start_session_command_topic_,
      &cancel_session_command_topic_,
      &handoff_cancel_service_name_,
      &map_save_service_name_,
      &saved_map_expected_frame_};

    for (const auto * endpoint : endpoint_values) {
      if (endpoint->empty()) {
        throw std::invalid_argument(
                "autonomous_mapping_endpoint_must_not_be_empty");
      }
    }

    if (
      evaluation_period_ms_ < 50 ||
      evaluation_period_ms_ > 10000)
    {
      throw std::invalid_argument(
              "evaluation_period_ms_out_of_range");
    }

    if (
      command_retry_period_ms_ < 100 ||
      command_retry_period_ms_ > 60000)
    {
      throw std::invalid_argument(
              "command_retry_period_ms_out_of_range");
    }

    if (default_mission_timeout_s_ < 0.0) {
      throw std::invalid_argument(
              "default_mission_timeout_s_must_be_nonnegative");
    }

    if (
      map_save_operation_timeout_s_ <= 0.0 ||
      map_save_operation_timeout_s_ > 300.0)
    {
      throw std::invalid_argument(
              "save_operation_timeout_s_out_of_range");
    }

    if (
      start_pose_lookup_timeout_s_ <= 0.0 ||
      start_pose_stale_timeout_s_ < 0.0 ||
      start_pose_operation_timeout_s_ <= 0.0 ||
      scan360_operation_timeout_s_ <= 0.0 ||
      head_scan_operation_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument(
              "autonomous_sequence_timeout_out_of_range");
    }
  }

  void create_pose_reader()
  {
    if (!inputs_.require_start_pose_capture) {
      return;
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    TfPoseReaderOptions options;
    options.target_frame = start_pose_target_frame_;
    options.source_frame = start_pose_source_frame_;
    options.lookup_timeout_sec = start_pose_lookup_timeout_s_;
    options.stale_timeout_sec = start_pose_stale_timeout_s_;

    start_pose_reader_ = std::make_unique<TfPoseReader>(
      get_clock(),
      tf_buffer_,
      std::move(options));
  }

  void create_interfaces()
  {
    const auto retained_qos =
      rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .transient_local();

    const auto command_qos =
      rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();

    status_publisher_ = create_publisher<MissionStatus>(
      status_topic_,
      retained_qos);

    mode_command_publisher_ = create_publisher<StringMessage>(
      mode_command_topic_,
      command_qos);

    start_session_command_publisher_ = create_publisher<StringMessage>(
      start_session_command_topic_,
      command_qos);

    cancel_session_command_publisher_ = create_publisher<StringMessage>(
      cancel_session_command_topic_,
      command_qos);

    mode_subscription_ = create_subscription<StringMessage>(
      mode_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_mode,
        this,
        std::placeholders::_1));

    exploration_mode_subscription_ = create_subscription<StringMessage>(
      exploration_mode_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_exploration_mode,
        this,
        std::placeholders::_1));

    workflow_phase_subscription_ = create_subscription<StringMessage>(
      workflow_phase_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_workflow_phase,
        this,
        std::placeholders::_1));

    session_state_subscription_ = create_subscription<StringMessage>(
      session_state_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_session_state,
        this,
        std::placeholders::_1));

    readiness_subscription_ = create_subscription<StringMessage>(
      readiness_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_readiness,
        this,
        std::placeholders::_1));

    safety_stop_subscription_ = create_subscription<BoolMessage>(
      safety_stop_topic_,
      command_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_safety_stop,
        this,
        std::placeholders::_1));

    runtime_authority_subscription_ = create_subscription<BoolMessage>(
      runtime_authority_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_runtime_authority,
        this,
        std::placeholders::_1));

    handoff_state_subscription_ = create_subscription<StringMessage>(
      handoff_state_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_handoff_state,
        this,
        std::placeholders::_1));

    frontier_status_subscription_ = create_subscription<FrontierStatus>(
      frontier_status_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_frontier_status,
        this,
        std::placeholders::_1));

    scan360_state_subscription_ = create_subscription<StringMessage>(
      scan360_state_topic_,
      retained_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_scan360_state,
        this,
        std::placeholders::_1));

    head_scan_state_subscription_ = create_subscription<StringMessage>(
      head_scan_state_topic_,
      command_qos,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_head_scan_state,
        this,
        std::placeholders::_1));

    handoff_cancel_client_ = create_client<Trigger>(
      handoff_cancel_service_name_);

    map_save_client_ = create_client<Trigger>(
      map_save_service_name_);

    scan360_start_client_ = create_client<Trigger>(
      scan360_start_service_name_);

    scan360_cancel_client_ = create_client<Trigger>(
      scan360_cancel_service_name_);

    head_scan_start_client_ = create_client<Trigger>(
      head_scan_start_service_name_);

    head_scan_pause_client_ = create_client<Trigger>(
      head_scan_pause_service_name_);

    head_scan_resume_client_ = create_client<Trigger>(
      head_scan_resume_service_name_);

    control_service_ = create_service<ControlMission>(
      control_service_name_,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_control,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    action_server_ = rclcpp_action::create_server<RunMission>(
      this,
      action_name_,
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_action_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &AutonomousMappingOrchestratorNode::handle_accepted,
        this,
        std::placeholders::_1));
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const RunMission::Goal> goal)
  {
    const bool request_valid =
      goal->contract_version == RunMission::Goal::CONTRACT_VERSION &&
      !goal->mission_id.empty() &&
      !goal->actor_id.empty() &&
      !goal->map_id.empty() &&
      goal->map_revision > 0U &&
      goal->strategy == RunMission::Goal::STRATEGY_FRONTIER &&
      duration_valid(goal->mission_timeout);

    if (!request_valid) {
      RCLCPP_WARN(
        get_logger(),
        "rejected invalid autonomous mapping goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    if (
      goal_reserved_ ||
      goal_handle_ ||
      autonomous::is_active(mission_.snapshot().state))
    {
      RCLCPP_WARN(
        get_logger(),
        "rejected autonomous mapping goal while mission active");
      return rclcpp_action::GoalResponse::REJECT;
    }

    goal_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_action_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!goal_handle_ || goal_handle_ != goal_handle) {
        return rclcpp_action::CancelResponse::REJECT;
      }

      const auto state = mission_.snapshot().state;
      if (
        state == autonomous::MissionState::Saving ||
        state == autonomous::MissionState::Verifying)
      {
        RCLCPP_WARN(
          get_logger(),
          "rejected action cancellation during bounded save pipeline");
        return rclcpp_action::CancelResponse::REJECT;
      }

      const auto decision = mission_.control(
        autonomous::MissionCommand::Cancel,
        "action_cancel_requested",
        inputs_);

      if (!decision.accepted) {
        return rclcpp_action::CancelResponse::REJECT;
      }
    }

    evaluate_and_apply();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();

    autonomous::MissionRequest request;
    request.mission_id = goal->mission_id;
    request.actor_id = goal->actor_id;
    request.map_id = goal->map_id;
    request.map_revision = goal->map_revision;
    request.strategy = autonomous::MissionStrategy::Frontier;
    request.auto_save = goal->auto_save;
    request.require_quality_approval = goal->require_quality_approval;

    const double requested_timeout_s =
      duration_seconds(goal->mission_timeout);

    std::shared_ptr<RunMission::Result> failed_result;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      goal_reserved_ = false;
      goal_handle_ = goal_handle;
      mission_started_at_ = std::chrono::steady_clock::now();
      mission_timeout_s_ = requested_timeout_s > 0.0 ?
        requested_timeout_s : default_mission_timeout_s_;
      timeout_abort_requested_ = false;
      reset_sequence_pipeline_locked();
      reset_save_pipeline_locked();
      completion_detector_.reset("mission_started");
      update_completion_inputs_locked(
        completion_detector_.snapshot());

      const auto decision = mission_.start(request, inputs_);
      if (!decision.accepted) {
        failed_result = std::make_shared<RunMission::Result>();
        failed_result->success = false;
        failed_result->result_code = RunMission::Result::RESULT_BUSY;
        failed_result->reason = decision.reason;
        failed_result->final_status = make_status_locked();
        failed_result->map_saved = false;
        failed_result->map_release_id.clear();
        goal_handle_.reset();
      }
    }

    if (failed_result) {
      goal_handle->abort(failed_result);
      return;
    }

    evaluate_and_apply();
  }

  void handle_control(
    const ControlMission::Request::SharedPtr request,
    ControlMission::Response::SharedPtr response)
  {
    autonomous::MissionDecision decision;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        request->contract_version !=
        ControlMission::Request::CONTRACT_VERSION ||
        request->mission_id.empty() ||
        request->actor_id.empty())
      {
        response->accepted = false;
        response->result_code =
          ControlMission::Response::RESULT_INVALID_REQUEST;
        response->reason = "invalid_control_request";
        response->status = make_status_locked();
        return;
      }

      if (!autonomous::is_active(mission_.snapshot().state)) {
        response->accepted = false;
        response->result_code =
          ControlMission::Response::RESULT_NO_ACTIVE_MISSION;
        response->reason = "no_active_mission";
        response->status = make_status_locked();
        return;
      }

      if (request->mission_id != mission_.snapshot().request.mission_id) {
        response->accepted = false;
        response->result_code =
          ControlMission::Response::RESULT_MISSION_MISMATCH;
        response->reason = "mission_id_mismatch";
        response->status = make_status_locked();
        return;
      }

      try {
        decision = mission_.control(
          command_from_message(request->command),
          request->reason,
          inputs_);
      } catch (const std::invalid_argument &) {
        response->accepted = false;
        response->result_code =
          ControlMission::Response::RESULT_INVALID_REQUEST;
        response->reason = "unknown_control_command";
        response->status = make_status_locked();
        return;
      }

      response->accepted = decision.accepted;
      response->result_code = decision.accepted ?
        ControlMission::Response::RESULT_ACCEPTED :
        ControlMission::Response::RESULT_INVALID_STATE;
      response->reason = decision.reason;
      response->status = make_status_locked();
    }

    evaluate_and_apply();
  }

  void handle_mode(const StringMessage::ConstSharedPtr message)
  {
    const auto value = mapping_mode_from_string(message->data);
    if (!value.has_value()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.mode = value.value();
    }
    evaluate_and_apply();
  }

  void handle_exploration_mode(
    const StringMessage::ConstSharedPtr message)
  {
    const auto value = exploration_mode_from_string(message->data);
    if (!value.has_value()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.exploration_mode = value.value();
    }
    evaluate_and_apply();
  }

  void handle_workflow_phase(
    const StringMessage::ConstSharedPtr message)
  {
    const auto value = workflow_phase_from_string(message->data);
    if (!value.has_value()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.workflow_phase = value.value();
    }
    evaluate_and_apply();
  }

  void handle_session_state(
    const StringMessage::ConstSharedPtr message)
  {
    const auto value = session_state_from_string(message->data);
    if (!value.has_value()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.session_state = value.value();
    }
    evaluate_and_apply();
  }

  void handle_readiness(const StringMessage::ConstSharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.readiness_received = true;
      inputs_.mapping_ready = message->data == "ready";
    }
    evaluate_and_apply();
  }

  void handle_safety_stop(const BoolMessage::ConstSharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.safety_stop_received = true;
      inputs_.safety_stop_active = message->data;
    }
    evaluate_and_apply();
  }

  void handle_runtime_authority(const BoolMessage::ConstSharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.runtime_authority_received = true;
      inputs_.runtime_authorized = message->data;
    }
    evaluate_and_apply();
  }

  void handle_handoff_state(const StringMessage::ConstSharedPtr message)
  {
    if (!known_handoff_state(message->data)) {
      RCLCPP_WARN(
        get_logger(),
        "ignored unknown exploration handoff state: %s",
        message->data.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.handoff_state_received = true;
      inputs_.handoff_state = message->data;
      inputs_.handoff_active =
        autonomous::is_active_handoff_state(message->data);
    }
    evaluate_and_apply();
  }

  void handle_scan360_state(
    const StringMessage::ConstSharedPtr message)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.scan360_state = message->data;

      if (scan360_start_acknowledged_) {
        inputs_.scan360_active = scan360_state_is_active(message->data);

        if (message->data == "complete") {
          inputs_.scan360_active = false;
          inputs_.scan360_complete = true;
          inputs_.scan360_succeeded = true;
          inputs_.scan360_reason = "scan360_complete";
        }

        if (
          message->data == "failed" ||
          message->data == "canceled")
        {
          inputs_.scan360_active = false;
          inputs_.scan360_complete = true;
          inputs_.scan360_succeeded = false;
          inputs_.scan360_reason = "scan360_" + message->data;
        }
      }
    }

    evaluate_and_apply();
  }

  void handle_head_scan_state(
    const StringMessage::ConstSharedPtr message)
  {
    const std::string state = state_field(message->data);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs_.head_scan_state = state;

      if (head_scan_start_acknowledged_) {
        inputs_.head_scan_active = state == "running";
        inputs_.head_scan_paused = state == "paused";

        if (state == "done") {
          inputs_.head_scan_active = false;
          inputs_.head_scan_complete = true;
          inputs_.head_scan_succeeded = true;
          inputs_.head_scan_reason = "head_scan_complete";
        } else if (state == "error") {
          inputs_.head_scan_active = false;
          inputs_.head_scan_complete = true;
          inputs_.head_scan_succeeded = false;
          inputs_.head_scan_reason = "head_scan_error";
        }
      }
    }

    evaluate_and_apply();
  }

  void handle_frontier_status(
    const FrontierStatus::ConstSharedPtr message)
  {
    if (message->contract_version != FrontierStatus::CONTRACT_VERSION) {
      RCLCPP_WARN(
        get_logger(),
        "ignored frontier status with unsupported contract version: %u",
        message->contract_version);
      return;
    }

    const auto current_time = std::chrono::steady_clock::now();

    autonomous::FrontierPlanObservation observation;
    observation.received = true;
    observation.runtime_enabled = message->enabled;
    observation.goal_pending = message->goal_pending;
    observation.handoff_active =
      autonomous::is_active_handoff_state(message->handoff_state);
    observation.map_generation = message->map_generation;
    observation.planned_map_generation =
      message->planned_map_generation;
    observation.plan_sequence = message->plan_sequence;
    observation.planning_status = message->planning_status;
    observation.detected_frontiers = message->detected_frontiers;
    observation.reachable_frontiers = message->reachable_frontiers;
    observation.received_at_s = steady_seconds(current_time);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto completion = completion_detector_.observe(
        observation,
        steady_seconds(current_time));
      update_completion_inputs_locked(completion);
    }

    evaluate_and_apply();
  }

  void update_completion_inputs_locked(
    const autonomous::FrontierCompletionSnapshot & completion)
  {
    inputs_.frontier_status_received = completion.status_received;
    inputs_.frontier_status_fresh = completion.status_fresh;
    inputs_.frontier_planning_status = completion.planning_status;
    inputs_.frontier_plan_sequence = completion.plan_sequence;
    inputs_.frontier_map_generation = completion.map_generation;
    inputs_.detected_frontiers = completion.detected_frontiers;
    inputs_.reachable_frontiers = completion.reachable_frontiers;
    inputs_.exhaustion_observations = completion.observations;
    inputs_.exhaustion_stable_duration_s =
      completion.stable_duration_s;
    inputs_.completion_candidate = completion.candidate;
    inputs_.completion_confirmed = completion.confirmed;
    inputs_.completion_reason = completion.reason;
  }

  void reset_sequence_pipeline_locked()
  {
    inputs_.start_pose_generation = 0U;
    inputs_.start_pose_capture_started = false;
    inputs_.start_pose_capture_complete = false;
    inputs_.start_pose_valid = false;
    inputs_.start_pose_reason = "start_pose_not_requested";

    inputs_.scan360_generation = 0U;
    inputs_.scan360_started = false;
    inputs_.scan360_active = false;
    inputs_.scan360_complete = false;
    inputs_.scan360_succeeded = false;
    inputs_.scan360_state = "idle";
    inputs_.scan360_reason = "scan360_not_requested";

    inputs_.head_scan_generation = 0U;
    inputs_.head_scan_started = false;
    inputs_.head_scan_active = false;
    inputs_.head_scan_paused = false;
    inputs_.head_scan_complete = false;
    inputs_.head_scan_succeeded = false;
    inputs_.head_scan_state = "idle";
    inputs_.head_scan_reason = "head_scan_not_requested";

    start_pose_map_ = geometry_msgs::msg::PoseStamped{};
    start_pose_map_.header.frame_id = start_pose_target_frame_;
    start_pose_capture_started_at_.reset();
    scan360_started_at_.reset();
    head_scan_started_at_.reset();

    ++scan360_operation_epoch_;
    ++head_scan_operation_epoch_;
    observed_sequence_state_ = autonomous::MissionState::Idle;

    start_pose_capture_in_flight_ = false;
    scan360_start_in_flight_ = false;
    scan360_cancel_in_flight_ = false;
    scan360_start_acknowledged_ = false;
    head_scan_start_in_flight_ = false;
    head_scan_pause_in_flight_ = false;
    head_scan_resume_in_flight_ = false;
    head_scan_start_acknowledged_ = false;
  }

  void prepare_scan360_operation_locked()
  {
    ++scan360_operation_epoch_;
    inputs_.scan360_started = false;
    inputs_.scan360_active = false;
    inputs_.scan360_complete = false;
    inputs_.scan360_succeeded = false;
    inputs_.scan360_state = "idle";
    inputs_.scan360_reason = "scan360_not_requested";
    scan360_started_at_.reset();
    scan360_start_in_flight_ = false;
    scan360_cancel_in_flight_ = false;
    scan360_start_acknowledged_ = false;
  }

  void prepare_head_scan_operation_locked(const bool preserve_paused)
  {
    ++head_scan_operation_epoch_;
    inputs_.head_scan_started = preserve_paused;
    inputs_.head_scan_active = false;
    inputs_.head_scan_paused = preserve_paused;
    inputs_.head_scan_complete = false;
    inputs_.head_scan_succeeded = false;
    inputs_.head_scan_state = preserve_paused ? "paused" : "idle";
    inputs_.head_scan_reason = preserve_paused ?
      "head_scan_resume_required" : "head_scan_not_requested";
    head_scan_started_at_.reset();
    head_scan_start_in_flight_ = false;
    head_scan_pause_in_flight_ = false;
    head_scan_resume_in_flight_ = false;
    head_scan_start_acknowledged_ = false;
  }

  void refresh_sequence_stage_locked(
    const autonomous::MissionState state)
  {
    if (state == observed_sequence_state_) {
      return;
    }

    if (autonomous::is_scan_state(state)) {
      prepare_scan360_operation_locked();
    } else if (state == autonomous::MissionState::InitialHeadScan) {
      prepare_head_scan_operation_locked(inputs_.head_scan_paused);
    }

    observed_sequence_state_ = state;
  }

  void reset_save_pipeline_locked()
  {
    inputs_.map_save_started = false;
    inputs_.map_save_complete = false;
    inputs_.map_save_succeeded = false;
    inputs_.map_save_reason = "map_save_not_started";
    inputs_.saved_session_directory.clear();
    inputs_.verification_started = false;
    inputs_.verification_complete = false;
    inputs_.verification_succeeded = false;
    inputs_.verification_reason = "verification_not_started";
    map_save_in_flight_ = false;
    verification_in_flight_ = false;
    verification_dispatch_pending_ = false;
    map_save_started_at_.reset();
  }

  bool command_retry_elapsed_locked(
    const std::string & command_key,
    const std::chrono::steady_clock::time_point current_time)
  {
    if (
      command_key != last_command_key_ ||
      !last_command_attempt_.has_value() ||
      current_time - last_command_attempt_.value() >=
      std::chrono::milliseconds(command_retry_period_ms_))
    {
      last_command_key_ = command_key;
      last_command_attempt_ = current_time;
      return true;
    }

    return false;
  }

  void evaluate_and_apply()
  {
    autonomous::MissionDecision decision;
    MissionStatus status;
    std::shared_ptr<GoalHandle> feedback_handle;
    std::shared_ptr<GoalHandle> terminal_handle;
    std::optional<autonomous::MissionSnapshot> terminal_snapshot;

    bool publish_start_session = false;
    bool publish_frontier_mode = false;
    bool publish_scan360_mode = false;
    bool publish_monitor_mode = false;
    bool publish_cancel_session = false;
    bool capture_start_pose = false;
    bool dispatch_handoff_cancel = false;
    bool dispatch_scan360_start = false;
    bool dispatch_scan360_cancel = false;
    bool dispatch_head_scan_start = false;
    bool dispatch_head_scan_pause = false;
    bool dispatch_head_scan_resume = false;
    bool dispatch_map_save = false;
    bool dispatch_verification = false;
    bool verification_started_this_cycle = false;
    std::string map_save_mission_id;
    std::string verification_directory;
    std::string verification_map_id;
    std::string sequence_mission_id;
    std::uint64_t scan360_dispatch_epoch = 0U;
    std::uint64_t head_scan_dispatch_epoch = 0U;

    const auto current_time = std::chrono::steady_clock::now();

    {
      std::lock_guard<std::mutex> lock(mutex_);

      const auto completion = completion_detector_.tick(
        steady_seconds(current_time));
      update_completion_inputs_locked(completion);

      if (
        mission_.snapshot().state == autonomous::MissionState::Saving &&
        inputs_.map_save_started &&
        !inputs_.map_save_complete &&
        map_save_started_at_.has_value() &&
        std::chrono::duration<double>(
          current_time - map_save_started_at_.value()).count() >=
        map_save_operation_timeout_s_)
      {
        inputs_.map_save_complete = true;
        inputs_.map_save_succeeded = false;
        inputs_.map_save_reason = "automatic_map_save_timeout";
        map_save_in_flight_ = false;
      }

      if (
        inputs_.start_pose_capture_started &&
        !inputs_.start_pose_capture_complete &&
        start_pose_capture_started_at_.has_value() &&
        std::chrono::duration<double>(
          current_time - start_pose_capture_started_at_.value()).count() >=
        start_pose_operation_timeout_s_)
      {
        ++inputs_.start_pose_generation;
        inputs_.start_pose_capture_complete = true;
        inputs_.start_pose_valid = false;
        inputs_.start_pose_reason = "start_pose_capture_timeout";
        start_pose_capture_in_flight_ = false;
      }

      if (
        autonomous::is_scan_state(mission_.snapshot().state) &&
        inputs_.scan360_started &&
        !inputs_.scan360_complete &&
        scan360_started_at_.has_value() &&
        std::chrono::duration<double>(
          current_time - scan360_started_at_.value()).count() >=
        scan360_operation_timeout_s_)
      {
        ++inputs_.scan360_generation;
        ++scan360_operation_epoch_;
        inputs_.scan360_active = false;
        inputs_.scan360_complete = true;
        inputs_.scan360_succeeded = false;
        inputs_.scan360_state = "failed";
        inputs_.scan360_reason = "scan360_operation_timeout";
        scan360_start_in_flight_ = false;
        scan360_cancel_in_flight_ = false;
        scan360_start_acknowledged_ = false;
      }

      if (
        mission_.snapshot().state ==
        autonomous::MissionState::InitialHeadScan &&
        inputs_.head_scan_started &&
        !inputs_.head_scan_complete &&
        head_scan_started_at_.has_value() &&
        std::chrono::duration<double>(
          current_time - head_scan_started_at_.value()).count() >=
        head_scan_operation_timeout_s_)
      {
        ++inputs_.head_scan_generation;
        ++head_scan_operation_epoch_;
        inputs_.head_scan_active = false;
        inputs_.head_scan_paused = false;
        inputs_.head_scan_complete = true;
        inputs_.head_scan_succeeded = false;
        inputs_.head_scan_state = "error";
        inputs_.head_scan_reason = "head_scan_operation_timeout";
        head_scan_start_in_flight_ = false;
        head_scan_pause_in_flight_ = false;
        head_scan_resume_in_flight_ = false;
        head_scan_start_acknowledged_ = false;
      }

      const auto mission_state = mission_.snapshot().state;
      if (
        autonomous::is_active(mission_state) &&
        mission_state != autonomous::MissionState::Canceling &&
        mission_state != autonomous::MissionState::CompletionPending &&
        mission_state != autonomous::MissionState::Saving &&
        mission_state != autonomous::MissionState::Verifying &&
        mission_timeout_s_ > 0.0 &&
        mission_started_at_.has_value() &&
        !timeout_abort_requested_ &&
        std::chrono::duration<double>(
          current_time - mission_started_at_.value()).count() >=
        mission_timeout_s_)
      {
        timeout_abort_requested_ = true;
        decision = mission_.abort(
          autonomous::MissionResult::TimedOut,
          "mission_timeout",
          inputs_);
      } else {
        decision = mission_.observe(inputs_);
      }

      refresh_sequence_stage_locked(decision.snapshot.state);

      if (
        decision.request_start_session &&
        command_retry_elapsed_locked("start_session", current_time))
      {
        publish_start_session = true;
      }

      if (
        decision.request_frontier_mode &&
        command_retry_elapsed_locked("frontier_mode", current_time))
      {
        publish_frontier_mode = true;
      }

      if (
        decision.request_scan360_mode &&
        command_retry_elapsed_locked("scan360_mode", current_time))
      {
        publish_scan360_mode = true;
      }

      if (
        decision.request_start_pose_capture &&
        !start_pose_capture_in_flight_ &&
        command_retry_elapsed_locked("start_pose_capture", current_time))
      {
        inputs_.start_pose_capture_started = true;
        if (!start_pose_capture_started_at_.has_value()) {
          start_pose_capture_started_at_ = current_time;
        }
        start_pose_capture_in_flight_ = true;
        capture_start_pose = true;
      }

      if (decision.request_scan360_start) {
        if (!inputs_.scan360_started) {
          inputs_.scan360_started = true;
          inputs_.scan360_state = "waiting_for_service";
          inputs_.scan360_reason =
            "waiting_for_scan360_start_service";
          scan360_started_at_ = current_time;
        }

        if (
          !scan360_start_in_flight_ &&
          scan360_start_client_->service_is_ready() &&
          command_retry_elapsed_locked("scan360_start", current_time))
        {
          scan360_start_in_flight_ = true;
          sequence_mission_id = mission_.snapshot().request.mission_id;
          scan360_dispatch_epoch = scan360_operation_epoch_;
          dispatch_scan360_start = true;
        }
      }

      if (
        decision.request_scan360_cancel &&
        !scan360_cancel_in_flight_ &&
        scan360_cancel_client_->service_is_ready() &&
        command_retry_elapsed_locked("scan360_cancel", current_time))
      {
        scan360_cancel_in_flight_ = true;
        sequence_mission_id = mission_.snapshot().request.mission_id;
        scan360_dispatch_epoch = scan360_operation_epoch_;
        dispatch_scan360_cancel = true;
      }

      if (decision.request_head_scan_start) {
        if (!inputs_.head_scan_started) {
          inputs_.head_scan_started = true;
          inputs_.head_scan_state = "waiting_for_service";
          inputs_.head_scan_reason =
            "waiting_for_head_scan_start_service";
          head_scan_started_at_ = current_time;
        }

        if (
          !head_scan_start_in_flight_ &&
          head_scan_start_client_->service_is_ready() &&
          command_retry_elapsed_locked("head_scan_start", current_time))
        {
          head_scan_start_in_flight_ = true;
          sequence_mission_id = mission_.snapshot().request.mission_id;
          head_scan_dispatch_epoch = head_scan_operation_epoch_;
          dispatch_head_scan_start = true;
        }
      }

      if (
        decision.request_head_scan_pause &&
        !head_scan_pause_in_flight_ &&
        head_scan_pause_client_->service_is_ready() &&
        command_retry_elapsed_locked("head_scan_pause", current_time))
      {
        head_scan_pause_in_flight_ = true;
        sequence_mission_id = mission_.snapshot().request.mission_id;
        head_scan_dispatch_epoch = head_scan_operation_epoch_;
        dispatch_head_scan_pause = true;
      }

      if (decision.request_head_scan_resume) {
        if (!head_scan_started_at_.has_value()) {
          head_scan_started_at_ = current_time;
        }

        if (
          !head_scan_resume_in_flight_ &&
          head_scan_resume_client_->service_is_ready() &&
          command_retry_elapsed_locked("head_scan_resume", current_time))
        {
          head_scan_resume_in_flight_ = true;
          sequence_mission_id = mission_.snapshot().request.mission_id;
          head_scan_dispatch_epoch = head_scan_operation_epoch_;
          dispatch_head_scan_resume = true;
        }
      }

      if (
        decision.request_monitor_mode &&
        command_retry_elapsed_locked("monitor_mode", current_time))
      {
        publish_monitor_mode = true;
      }

      if (
        decision.request_cancel_session &&
        command_retry_elapsed_locked("cancel_session", current_time))
      {
        publish_cancel_session = true;
      }

      if (
        decision.request_handoff_cancel &&
        !handoff_cancel_in_flight_ &&
        handoff_cancel_client_->service_is_ready() &&
        command_retry_elapsed_locked("handoff_cancel", current_time))
      {
        handoff_cancel_in_flight_ = true;
        dispatch_handoff_cancel = true;
      }

      if (decision.request_map_save) {
        if (!inputs_.map_save_started) {
          inputs_.map_save_started = true;
          inputs_.map_save_reason = "waiting_for_map_save_service";
          map_save_started_at_ = current_time;
        }

        if (
          !map_save_in_flight_ &&
          !inputs_.map_save_complete &&
          map_save_client_->service_is_ready())
        {
          map_save_in_flight_ = true;
          inputs_.map_save_reason = "automatic_map_save_in_progress";
          map_save_mission_id = mission_.snapshot().request.mission_id;
          dispatch_map_save = true;
        }
      }

      if (
        decision.request_saved_map_verification &&
        !inputs_.verification_started &&
        !inputs_.verification_complete)
      {
        inputs_.verification_started = true;
        inputs_.verification_reason =
          "saved_map_verification_in_progress";
        verification_dispatch_pending_ = true;
        verification_started_this_cycle = true;
      }

      if (
        verification_dispatch_pending_ &&
        !verification_started_this_cycle &&
        !verification_in_flight_ &&
        !inputs_.verification_complete &&
        mission_.snapshot().state ==
        autonomous::MissionState::Verifying)
      {
        verification_dispatch_pending_ = false;
        verification_in_flight_ = true;
        verification_directory = inputs_.saved_session_directory;
        verification_map_id = mission_.snapshot().request.map_id;
        dispatch_verification = true;
      }

      if (
        mission_.snapshot().state !=
        autonomous::MissionState::Verifying)
      {
        verification_dispatch_pending_ = false;
      }

      status = make_status_locked();
      feedback_handle = goal_handle_;

      if (decision.terminal && goal_handle_) {
        terminal_handle = goal_handle_;
        terminal_snapshot = mission_.snapshot();
        feedback_handle.reset();
        goal_handle_.reset();
      }
    }

    status_publisher_->publish(status);

    if (feedback_handle) {
      auto feedback = std::make_shared<RunMission::Feedback>();
      feedback->status = status;
      feedback_handle->publish_feedback(feedback);
    }

    if (publish_start_session) {
      publish_string(start_session_command_publisher_, status.mission_id);
    }

    if (publish_frontier_mode) {
      publish_string(mode_command_publisher_, "autonomous:frontier");
    }

    if (publish_scan360_mode) {
      publish_string(mode_command_publisher_, "autonomous:scan360");
    }

    if (publish_monitor_mode) {
      publish_string(mode_command_publisher_, "monitor_only");
    }

    if (publish_cancel_session) {
      publish_string(cancel_session_command_publisher_, status.mission_id);
    }

    if (capture_start_pose) {
      capture_start_pose_request();
    }

    if (dispatch_handoff_cancel) {
      dispatch_cancel_request();
    }

    if (dispatch_scan360_start) {
      dispatch_sequence_trigger(
        scan360_start_client_,
        "scan360_start",
        sequence_mission_id,
        scan360_dispatch_epoch);
    }

    if (dispatch_scan360_cancel) {
      dispatch_sequence_trigger(
        scan360_cancel_client_,
        "scan360_cancel",
        sequence_mission_id,
        scan360_dispatch_epoch);
    }

    if (dispatch_head_scan_start) {
      dispatch_sequence_trigger(
        head_scan_start_client_,
        "head_scan_start",
        sequence_mission_id,
        head_scan_dispatch_epoch);
    }

    if (dispatch_head_scan_pause) {
      dispatch_sequence_trigger(
        head_scan_pause_client_,
        "head_scan_pause",
        sequence_mission_id,
        head_scan_dispatch_epoch);
    }

    if (dispatch_head_scan_resume) {
      dispatch_sequence_trigger(
        head_scan_resume_client_,
        "head_scan_resume",
        sequence_mission_id,
        head_scan_dispatch_epoch);
    }

    if (dispatch_map_save) {
      dispatch_map_save_request(map_save_mission_id);
    }

    if (dispatch_verification) {
      run_saved_map_verification(
        verification_directory,
        verification_map_id);
    }

    if (terminal_handle && terminal_snapshot.has_value()) {
      finish_action(
        terminal_handle,
        status,
        terminal_snapshot.value());
    }
  }

  void capture_start_pose_request()
  {
    TfPoseSnapshot pose;

    if (start_pose_reader_) {
      pose = start_pose_reader_->read();
    } else {
      pose.reason = "start_pose_reader_disabled";
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      start_pose_capture_in_flight_ = false;
      inputs_.start_pose_reason = pose.reason;

      if (pose.valid && pose.fresh) {
        ++inputs_.start_pose_generation;
        inputs_.start_pose_capture_complete = true;
        inputs_.start_pose_valid = true;
        inputs_.start_pose_reason = "start_pose_captured";

        start_pose_map_.header.stamp = pose.transform_stamp;
        start_pose_map_.header.frame_id = pose.target_frame;
        start_pose_map_.pose.position.x = pose.x_m;
        start_pose_map_.pose.position.y = pose.y_m;
        start_pose_map_.pose.position.z = pose.z_m;
        start_pose_map_.pose.orientation.x = pose.quaternion_x;
        start_pose_map_.pose.orientation.y = pose.quaternion_y;
        start_pose_map_.pose.orientation.z = pose.quaternion_z;
        start_pose_map_.pose.orientation.w = pose.quaternion_w;
      }
    }

    evaluate_and_apply();
  }

  void dispatch_sequence_trigger(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & operation,
    const std::string & mission_id,
    const std::uint64_t operation_epoch)
  {
    auto request = std::make_shared<Trigger::Request>();

    try {
      client->async_send_request(
        request,
        [this, operation, mission_id, operation_epoch](
          rclcpp::Client<Trigger>::SharedFuture future) {
          bool success = false;
          std::string reason{operation + "_empty_response"};

          try {
            const auto response = future.get();
            if (response) {
              success = response->success;
              reason = response->message;
            }
          } catch (const std::exception & exception) {
            reason = operation + "_response_error:" + exception.what();
          }

          {
            std::lock_guard<std::mutex> lock(mutex_);

            const bool scan_operation =
            operation.rfind("scan360_", 0) == 0;
            const bool head_operation =
            operation.rfind("head_scan_", 0) == 0;

            if (
              mission_.snapshot().request.mission_id != mission_id ||
              (scan_operation &&
              operation_epoch != scan360_operation_epoch_) ||
              (head_operation &&
              operation_epoch != head_scan_operation_epoch_))
            {
              return;
            }

            if (operation == "scan360_start") {
              scan360_start_in_flight_ = false;
              ++inputs_.scan360_generation;
              inputs_.scan360_started = true;
              scan360_start_acknowledged_ = success;
              inputs_.scan360_active = success;
              inputs_.scan360_complete = !success;
              inputs_.scan360_succeeded = false;
              inputs_.scan360_state = success ? "starting" : "failed";
              inputs_.scan360_reason = reason;
              if (!success) {
                scan360_started_at_.reset();
              }
            } else if (operation == "scan360_cancel") {
              scan360_cancel_in_flight_ = false;
              if (!success) {
                inputs_.scan360_reason = reason;
              }
            } else if (operation == "head_scan_start") {
              head_scan_start_in_flight_ = false;
              ++inputs_.head_scan_generation;
              inputs_.head_scan_started = true;
              head_scan_start_acknowledged_ = success;
              inputs_.head_scan_active = success;
              inputs_.head_scan_paused = false;
              inputs_.head_scan_complete = !success;
              inputs_.head_scan_succeeded = false;
              inputs_.head_scan_state = success ? "running" : "error";
              inputs_.head_scan_reason = reason;
              if (!success) {
                head_scan_started_at_.reset();
              }
            } else if (operation == "head_scan_pause") {
              head_scan_pause_in_flight_ = false;
              if (success) {
                inputs_.head_scan_active = false;
                inputs_.head_scan_paused = true;
                inputs_.head_scan_state = "paused";
                inputs_.head_scan_reason = reason;
              } else {
                inputs_.head_scan_reason = reason;
              }
            } else if (operation == "head_scan_resume") {
              head_scan_resume_in_flight_ = false;
              ++inputs_.head_scan_generation;
              head_scan_start_acknowledged_ = success;
              inputs_.head_scan_started = true;
              inputs_.head_scan_active = success;
              inputs_.head_scan_paused = false;
              inputs_.head_scan_complete = !success;
              inputs_.head_scan_succeeded = false;
              inputs_.head_scan_state = success ? "running" : "error";
              inputs_.head_scan_reason = reason;
              if (!success) {
                head_scan_started_at_.reset();
              }
            }
          }

          evaluate_and_apply();
        });
    } catch (const std::exception & exception) {
      {
        std::lock_guard<std::mutex> lock(mutex_);

        const bool scan_operation =
          operation.rfind("scan360_", 0) == 0;
        const bool head_operation =
          operation.rfind("head_scan_", 0) == 0;

        if (
          mission_.snapshot().request.mission_id != mission_id ||
          (scan_operation &&
          operation_epoch != scan360_operation_epoch_) ||
          (head_operation &&
          operation_epoch != head_scan_operation_epoch_))
        {
          return;
        }

        const std::string reason =
          operation + "_dispatch_error:" + exception.what();

        if (operation == "scan360_start") {
          scan360_start_in_flight_ = false;
          scan360_start_acknowledged_ = false;
          ++inputs_.scan360_generation;
          inputs_.scan360_started = true;
          inputs_.scan360_active = false;
          inputs_.scan360_complete = true;
          inputs_.scan360_succeeded = false;
          inputs_.scan360_state = "failed";
          inputs_.scan360_reason = reason;
        } else if (operation == "scan360_cancel") {
          scan360_cancel_in_flight_ = false;
          inputs_.scan360_reason = reason;
        } else if (operation == "head_scan_start") {
          head_scan_start_in_flight_ = false;
          head_scan_start_acknowledged_ = false;
          ++inputs_.head_scan_generation;
          inputs_.head_scan_started = true;
          inputs_.head_scan_active = false;
          inputs_.head_scan_complete = true;
          inputs_.head_scan_succeeded = false;
          inputs_.head_scan_state = "error";
          inputs_.head_scan_reason = reason;
        } else if (operation == "head_scan_pause") {
          head_scan_pause_in_flight_ = false;
          inputs_.head_scan_reason = reason;
        } else if (operation == "head_scan_resume") {
          head_scan_resume_in_flight_ = false;
          head_scan_start_acknowledged_ = false;
          ++inputs_.head_scan_generation;
          inputs_.head_scan_started = true;
          inputs_.head_scan_active = false;
          inputs_.head_scan_paused = false;
          inputs_.head_scan_complete = true;
          inputs_.head_scan_succeeded = false;
          inputs_.head_scan_state = "error";
          inputs_.head_scan_reason = reason;
        }
      }

      evaluate_and_apply();
    }
  }

  void dispatch_map_save_request(const std::string & mission_id)
  {
    auto request = std::make_shared<Trigger::Request>();

    try {
      map_save_client_->async_send_request(
        request,
        [this, mission_id](rclcpp::Client<Trigger>::SharedFuture future) {
          bool success = false;
          std::string reason{"automatic_map_save_empty_response"};
          std::string directory;

          try {
            const auto response = future.get();
            if (response) {
              success = response->success;
              reason = response->message;

              if (success) {
                const auto parsed = saved_session_directory(
                  response->message);
                if (!parsed.has_value()) {
                  success = false;
                  reason = "automatic_map_save_invalid_response";
                } else {
                  directory = parsed.value();
                  reason = "map_session_saved";
                }
              }
            }
          } catch (const std::exception & exception) {
            reason = std::string{"automatic_map_save_error:"} +
            exception.what();
          }

          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (
              mission_.snapshot().request.mission_id != mission_id)
            {
              return;
            }

            map_save_in_flight_ = false;

            if (
              mission_.snapshot().state !=
              autonomous::MissionState::Saving)
            {
              return;
            }
            inputs_.map_save_complete = true;
            inputs_.map_save_succeeded = success;
            inputs_.map_save_reason = reason;
            inputs_.saved_session_directory = directory;
          }

          evaluate_and_apply();
        });
    } catch (const std::exception & exception) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        map_save_in_flight_ = false;
        inputs_.map_save_complete = true;
        inputs_.map_save_succeeded = false;
        inputs_.map_save_reason =
          std::string{"automatic_map_save_dispatch_error:"} +
        exception.what();
      }

      evaluate_and_apply();
    }
  }

  void run_saved_map_verification(
    const std::string & session_directory,
    const std::string & map_id)
  {
    bool success = false;
    std::string reason{"saved_map_verification_not_run"};

    try {
      const auto verification =
        session::verify_saved_map_session(
        session_directory,
        map_id,
        saved_map_expected_frame_);
      success = verification.valid;
      reason = verification.reason;
    } catch (const std::exception & exception) {
      reason = std::string{"saved_map_verification_error:"} +
      exception.what();
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      verification_in_flight_ = false;

      if (
        mission_.snapshot().request.map_id != map_id ||
        mission_.snapshot().state !=
        autonomous::MissionState::Verifying)
      {
        return;
      }

      inputs_.verification_complete = true;
      inputs_.verification_succeeded = success;
      inputs_.verification_reason = reason;
    }

    evaluate_and_apply();
  }

  void dispatch_cancel_request()
  {
    auto request = std::make_shared<Trigger::Request>();

    try {
      handoff_cancel_client_->async_send_request(
        request,
        [this](rclcpp::Client<Trigger>::SharedFuture future) {
          try {
            const auto response = future.get();
            RCLCPP_INFO(
              get_logger(),
              "exploration handoff cancel response success=%s reason=%s",
              response->success ? "true" : "false",
              response->message.c_str());
          } catch (const std::exception & exception) {
            RCLCPP_ERROR(
              get_logger(),
              "exploration handoff cancel response failed: %s",
              exception.what());
          }

          {
            std::lock_guard<std::mutex> lock(mutex_);
            handoff_cancel_in_flight_ = false;
          }

          evaluate_and_apply();
        });
    } catch (const std::exception & exception) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        handoff_cancel_in_flight_ = false;
      }

      RCLCPP_ERROR(
        get_logger(),
        "exploration handoff cancel request failed: %s",
        exception.what());
    }
  }

  MissionStatus make_status_locked() const
  {
    const auto & snapshot = mission_.snapshot();

    MissionStatus status;
    status.contract_version = MissionStatus::CONTRACT_VERSION;
    status.stamp = now();
    status.mission_id = snapshot.request.mission_id;
    status.actor_id = snapshot.request.actor_id;
    status.map_id = snapshot.request.map_id;
    status.map_revision = snapshot.request.map_revision;
    status.strategy = static_cast<std::uint8_t>(snapshot.request.strategy);
    status.state = static_cast<std::uint8_t>(snapshot.state);
    status.state_text = std::string{autonomous::to_string(snapshot.state)};
    status.result_code = static_cast<std::uint8_t>(snapshot.result);
    status.active = snapshot.active;
    status.runtime_authorized = snapshot.runtime_authorized;
    status.mapping_ready = snapshot.mapping_ready;
    status.safety_stop_active = snapshot.safety_stop_active;
    status.handoff_active = snapshot.handoff_active;
    status.auto_save = snapshot.request.auto_save;
    status.require_quality_approval =
      snapshot.request.require_quality_approval;
    status.goals_succeeded = snapshot.goals_succeeded;
    status.goals_failed = snapshot.goals_failed;
    status.start_pose_capture_started =
      snapshot.start_pose_capture_started;
    status.start_pose_capture_complete =
      snapshot.start_pose_capture_complete;
    status.start_pose_valid = snapshot.start_pose_valid;
    status.start_pose_map = start_pose_map_;
    status.start_map_generation = snapshot.start_map_generation;
    status.start_pose_reason = snapshot.start_pose_reason;
    status.initial_scan360_complete =
      snapshot.initial_scan360_complete;
    status.initial_scan360_succeeded =
      snapshot.initial_scan360_succeeded;
    status.initial_head_scan_complete =
      snapshot.initial_head_scan_complete;
    status.initial_head_scan_succeeded =
      snapshot.initial_head_scan_succeeded;
    status.conditional_scan360_completed =
      snapshot.conditional_scan360_completed;
    status.scan360_started = snapshot.scan360_started;
    status.scan360_active = snapshot.scan360_active;
    status.scan360_complete = snapshot.scan360_complete;
    status.scan360_succeeded = snapshot.scan360_succeeded;
    status.scan360_stage = snapshot.scan360_stage;
    status.scan360_state = snapshot.scan360_state;
    status.scan360_reason = snapshot.scan360_reason;
    status.head_scan_started = snapshot.head_scan_started;
    status.head_scan_active = snapshot.head_scan_active;
    status.head_scan_paused = snapshot.head_scan_paused;
    status.head_scan_complete = snapshot.head_scan_complete;
    status.head_scan_succeeded = snapshot.head_scan_succeeded;
    status.head_scan_stage = snapshot.head_scan_stage;
    status.head_scan_state = snapshot.head_scan_state;
    status.head_scan_reason = snapshot.head_scan_reason;
    status.frontier_status_received =
      snapshot.frontier_status_received;
    status.frontier_status_fresh = snapshot.frontier_status_fresh;
    status.frontier_planning_status =
      snapshot.frontier_planning_status;
    status.frontier_plan_sequence =
      snapshot.frontier_plan_sequence;
    status.frontier_map_generation =
      snapshot.frontier_map_generation;
    status.detected_frontiers = snapshot.detected_frontiers;
    status.reachable_frontiers = snapshot.reachable_frontiers;
    status.exhaustion_observations =
      snapshot.exhaustion_observations;
    status.exhaustion_stable_duration_s =
      snapshot.exhaustion_stable_duration_s;
    status.completion_candidate = snapshot.completion_candidate;
    status.completion_confirmed = snapshot.completion_confirmed;
    status.completion_reason = snapshot.completion_reason;
    status.map_save_started = snapshot.map_save_started;
    status.map_save_complete = snapshot.map_save_complete;
    status.map_saved = snapshot.map_saved;
    status.map_save_reason = snapshot.map_save_reason;
    status.saved_session_directory =
      snapshot.saved_session_directory;
    status.verification_started = snapshot.verification_started;
    status.verification_complete = snapshot.verification_complete;
    status.map_verified = snapshot.map_verified;
    status.verification_reason = snapshot.verification_reason;
    status.reason = snapshot.reason;
    return status;
  }

  void finish_action(
    const std::shared_ptr<GoalHandle> & goal_handle,
    const MissionStatus & status,
    const autonomous::MissionSnapshot & snapshot)
  {
    auto result = std::make_shared<RunMission::Result>();
    result->success =
      snapshot.result == autonomous::MissionResult::Succeeded &&
      snapshot.state == autonomous::MissionState::Completed;
    result->result_code = static_cast<std::uint8_t>(snapshot.result);
    result->reason = snapshot.reason;
    result->final_status = status;
    result->map_saved = snapshot.map_saved;
    result->map_release_id.clear();

    if (
      snapshot.result == autonomous::MissionResult::Canceled &&
      goal_handle->is_canceling())
    {
      goal_handle->canceled(result);
    } else if (result->success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  static void publish_string(
    const rclcpp::Publisher<StringMessage>::SharedPtr & publisher,
    const std::string & value)
  {
    StringMessage message;
    message.data = value;
    publisher->publish(message);
  }

  std::string action_name_;
  std::string control_service_name_;
  std::string status_topic_;
  std::string mode_topic_;
  std::string exploration_mode_topic_;
  std::string workflow_phase_topic_;
  std::string session_state_topic_;
  std::string readiness_topic_;
  std::string safety_stop_topic_;
  std::string runtime_authority_topic_;
  std::string handoff_state_topic_;
  std::string frontier_status_topic_;
  std::string scan360_state_topic_;
  std::string head_scan_state_topic_;
  std::string scan360_start_service_name_;
  std::string scan360_cancel_service_name_;
  std::string head_scan_start_service_name_;
  std::string head_scan_pause_service_name_;
  std::string head_scan_resume_service_name_;
  std::string start_pose_target_frame_;
  std::string start_pose_source_frame_;
  std::string mode_command_topic_;
  std::string start_session_command_topic_;
  std::string cancel_session_command_topic_;
  std::string handoff_cancel_service_name_;
  std::string map_save_service_name_;
  std::string saved_map_expected_frame_;

  std::int64_t evaluation_period_ms_{250};
  std::int64_t command_retry_period_ms_{1000};
  double default_mission_timeout_s_{0.0};
  double mission_timeout_s_{0.0};
  double map_save_operation_timeout_s_{45.0};
  double start_pose_lookup_timeout_s_{0.20};
  double start_pose_stale_timeout_s_{1.0};
  double start_pose_operation_timeout_s_{10.0};
  double scan360_operation_timeout_s_{180.0};
  double head_scan_operation_timeout_s_{180.0};

  mutable std::mutex mutex_;
  autonomous::AutonomousMappingMission mission_;
  autonomous::FrontierCompletionDetector completion_detector_;
  autonomous::MissionInputs inputs_;
  std::shared_ptr<GoalHandle> goal_handle_;
  geometry_msgs::msg::PoseStamped start_pose_map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<TfPoseReader> start_pose_reader_;

  std::optional<std::chrono::steady_clock::time_point>
  mission_started_at_;
  std::optional<std::chrono::steady_clock::time_point>
  last_command_attempt_;
  std::optional<std::chrono::steady_clock::time_point>
  map_save_started_at_;
  std::optional<std::chrono::steady_clock::time_point>
  start_pose_capture_started_at_;
  std::optional<std::chrono::steady_clock::time_point>
  scan360_started_at_;
  std::optional<std::chrono::steady_clock::time_point>
  head_scan_started_at_;
  std::string last_command_key_;

  bool timeout_abort_requested_{false};
  bool handoff_cancel_in_flight_{false};
  bool start_pose_capture_in_flight_{false};
  bool scan360_start_in_flight_{false};
  bool scan360_cancel_in_flight_{false};
  bool scan360_start_acknowledged_{false};
  bool head_scan_start_in_flight_{false};
  bool head_scan_pause_in_flight_{false};
  bool head_scan_resume_in_flight_{false};
  bool head_scan_start_acknowledged_{false};
  bool map_save_in_flight_{false};
  bool verification_in_flight_{false};
  bool verification_dispatch_pending_{false};
  bool goal_reserved_{false};
  std::uint64_t scan360_operation_epoch_{0U};
  std::uint64_t head_scan_operation_epoch_{0U};
  autonomous::MissionState observed_sequence_state_{
    autonomous::MissionState::Idle};

  rclcpp_action::Server<RunMission>::SharedPtr action_server_;
  rclcpp::Service<ControlMission>::SharedPtr control_service_;
  rclcpp::Client<Trigger>::SharedPtr handoff_cancel_client_;
  rclcpp::Client<Trigger>::SharedPtr map_save_client_;
  rclcpp::Client<Trigger>::SharedPtr scan360_start_client_;
  rclcpp::Client<Trigger>::SharedPtr scan360_cancel_client_;
  rclcpp::Client<Trigger>::SharedPtr head_scan_start_client_;
  rclcpp::Client<Trigger>::SharedPtr head_scan_pause_client_;
  rclcpp::Client<Trigger>::SharedPtr head_scan_resume_client_;

  rclcpp::Publisher<MissionStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<StringMessage>::SharedPtr mode_command_publisher_;
  rclcpp::Publisher<StringMessage>::SharedPtr start_session_command_publisher_;
  rclcpp::Publisher<StringMessage>::SharedPtr cancel_session_command_publisher_;

  rclcpp::Subscription<StringMessage>::SharedPtr mode_subscription_;
  rclcpp::Subscription<StringMessage>::SharedPtr exploration_mode_subscription_;
  rclcpp::Subscription<StringMessage>::SharedPtr workflow_phase_subscription_;
  rclcpp::Subscription<StringMessage>::SharedPtr session_state_subscription_;
  rclcpp::Subscription<StringMessage>::SharedPtr readiness_subscription_;
  rclcpp::Subscription<BoolMessage>::SharedPtr safety_stop_subscription_;
  rclcpp::Subscription<BoolMessage>::SharedPtr runtime_authority_subscription_;
  rclcpp::Subscription<StringMessage>::SharedPtr handoff_state_subscription_;
  rclcpp::Subscription<FrontierStatus>::SharedPtr
    frontier_status_subscription_;
  rclcpp::Subscription<StringMessage>::SharedPtr
    scan360_state_subscription_;
  rclcpp::Subscription<StringMessage>::SharedPtr
    head_scan_state_subscription_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
};

}  // namespace savo_mapping

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<
        savo_mapping::AutonomousMappingOrchestratorNode>());
  } catch (const std::exception & exception) {
    std::cerr
      << "autonomous_mapping_orchestrator_node failed: "
      << exception.what()
      << '\n';

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
