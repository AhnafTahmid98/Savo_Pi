// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/action/run_autonomous_mapping.hpp"
#include "savo_msgs/msg/autonomous_mapping_status.hpp"
#include "savo_msgs/msg/location_record.hpp"
#include "savo_msgs/srv/control_autonomous_mapping.hpp"
#include "savo_msgs/srv/resolve_location.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "savo_bridge/ros_command_dispatcher.hpp"

namespace savo_bridge
{

namespace
{

using namespace std::chrono_literals;

using NavigateToPose =
  nav2_msgs::action::NavigateToPose;

using NavigationServerGoalHandle =
  rclcpp_action::ServerGoalHandle<NavigateToPose>;

using ResolveLocation =
  savo_msgs::srv::ResolveLocation;

using RunAutonomousMapping =
  savo_msgs::action::RunAutonomousMapping;

using MappingServerGoalHandle =
  rclcpp_action::ServerGoalHandle<RunAutonomousMapping>;

using ControlAutonomousMapping =
  savo_msgs::srv::ControlAutonomousMapping;

[[nodiscard]] bool twist_is_zero(
  const geometry_msgs::msg::Twist & message)
{
  constexpr double epsilon = 1.0e-9;

  return
    std::abs(message.linear.x) <= epsilon &&
    std::abs(message.linear.y) <= epsilon &&
    std::abs(message.linear.z) <= epsilon &&
    std::abs(message.angular.x) <= epsilon &&
    std::abs(message.angular.y) <= epsilon &&
    std::abs(message.angular.z) <= epsilon;
}

class RosCommandDispatcherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    const std::uint64_t instance =
      next_instance_.fetch_add(1U);

    const std::string suffix =
      std::to_string(instance);

    const std::string prefix =
      "/savo_bridge_dispatcher_test_" + suffix;

    fixture_node_ =
      std::make_shared<rclcpp::Node>(
      "savo_bridge_dispatcher_fixture_" + suffix);

    dispatcher_node_ =
      std::make_shared<rclcpp::Node>(
      "savo_bridge_dispatcher_under_test_" + suffix);

    config_.mode_command_topic =
      prefix + "/mode_cmd";

    config_.mode_state_topic =
      prefix + "/mode_state";

    config_.external_stop_topic =
      prefix + "/external_stop";

    config_.safety_stop_topic =
      prefix + "/safety_stop";

    config_.manual_velocity_topic =
      prefix + "/cmd_vel_manual";

    config_.safe_velocity_topic =
      prefix + "/cmd_vel_safe";

    config_.navigation_readiness_topic =
      prefix + "/navigation_readiness";

    config_.navigation_action_name =
      prefix + "/navigate_to_pose";

    config_.location_resolve_service =
      prefix + "/resolve_location";

    config_.map_context_status_topic =
      prefix + "/map_context";

    config_.mapping_action_name =
      prefix + "/run_mapping";

    config_.mapping_control_service =
      prefix + "/control_mapping";

    config_.mapping_status_topic =
      prefix + "/mapping_status";

    config_.supervisor_state_topic =
      prefix + "/supervisor_state";

    config_.active_map_id = "test-map";
    config_.active_map_revision = 7U;
    config_.require_active_map_context = true;

    config_.observed_state_timeout_ms = 500;
    config_.mode_transition_timeout_ms = 600;
    config_.stop_confirmation_timeout_ms = 800;
    config_.location_service_timeout_ms = 800;
    config_.navigation_server_timeout_ms = 800;
    config_.navigation_goal_response_timeout_ms = 800;
    config_.navigation_execution_timeout_ms = 3000;
    config_.teleop_cancel_timeout_ms = 800;
    config_.navigation_cancel_timeout_ms = 800;
    config_.mapping_server_timeout_ms = 800;
    config_.mapping_control_timeout_ms = 800;

    config_.maximum_teleop_duration_ms = 1000;
    config_.teleop_publish_period_ms = 20;
    config_.final_zero_publication_count = 3U;

    rclcpp::QoS latched_qos(
      rclcpp::KeepLast(1));

    latched_qos.reliable();
    latched_qos.transient_local();

    rclcpp::QoS reliable_qos(
      rclcpp::KeepLast(20));

    reliable_qos.reliable();

    mode_state_publisher_ =
      fixture_node_->create_publisher<
      std_msgs::msg::String>(
      config_.mode_state_topic,
      latched_qos);

    external_stop_publisher_ =
      fixture_node_->create_publisher<
      std_msgs::msg::Bool>(
      config_.external_stop_topic,
      reliable_qos);

    safety_stop_publisher_ =
      fixture_node_->create_publisher<
      std_msgs::msg::Bool>(
      config_.safety_stop_topic,
      reliable_qos);

    safe_velocity_publisher_ =
      fixture_node_->create_publisher<
      geometry_msgs::msg::Twist>(
      config_.safe_velocity_topic,
      reliable_qos);

    navigation_readiness_publisher_ =
      fixture_node_->create_publisher<
      std_msgs::msg::String>(
      config_.navigation_readiness_topic,
      latched_qos);

    map_context_publisher_ =
      fixture_node_->create_publisher<
      std_msgs::msg::String>(
      config_.map_context_status_topic,
      latched_qos);

    mapping_status_publisher_ =
      fixture_node_->create_publisher<
      savo_msgs::msg::AutonomousMappingStatus>(
      config_.mapping_status_topic,
      latched_qos);

    supervisor_state_publisher_ =
      fixture_node_->create_publisher<
      std_msgs::msg::String>(
      config_.supervisor_state_topic,
      latched_qos);

    mapping_status_message_.contract_version =
      savo_msgs::msg::AutonomousMappingStatus::CONTRACT_VERSION;
    mapping_status_message_.mission_id = "mission-test-1";
    mapping_status_message_.map_id = "mapping-test-map";
    mapping_status_message_.map_revision = 3U;
    mapping_status_message_.state =
      savo_msgs::msg::AutonomousMappingStatus::STATE_AWAITING_APPROVAL;
    mapping_status_message_.state_text = "awaiting_approval";
    mapping_status_message_.reason = "test_waiting_for_operator";
    mapping_status_message_.active = true;
    mapping_status_message_.mapping_ready = true;
    mapping_status_message_.goals_succeeded = 4U;
    mapping_status_message_.detected_frontiers = 2U;
    mapping_status_message_.reachable_frontiers = 1U;
    mapping_status_message_.coverage_completion_ratio = 0.75;
    mapping_status_message_.scan360_stage = "conditional";
    mapping_status_message_.scan360_state = "complete";
    mapping_status_message_.map_save_complete = true;
    mapping_status_message_.map_saved = true;
    mapping_status_message_.verification_complete = true;
    mapping_status_message_.map_verified = true;
    mapping_status_message_.location_verification_complete = true;
    mapping_status_message_.location_verification_passed = true;
    mapping_status_message_.approved_location_count = 2U;
    mapping_status_message_.review_generation = 8U;
    mapping_status_message_.approval_pending = true;
    mapping_status_message_.release_state = "awaiting_approval";

    resolve_location_service_ =
      fixture_node_->create_service<ResolveLocation>(
      config_.location_resolve_service,
      [this](
        const std::shared_ptr<
          ResolveLocation::Request> request,
        std::shared_ptr<
          ResolveLocation::Response> response)
      {
        resolve_request_count_.fetch_add(1U);

        response->normalized_query =
        request->query;

        response->match_type =
        ResolveLocation::Response::MATCH_LOCATION_ID;

        if (
          request->query != "A201" ||
          (
            request->enforce_map_context &&
            (
              request->map_id !=
              config_.active_map_id ||
              request->map_revision !=
              config_.active_map_revision
            )
          ))
        {
          response->resolved = false;
          response->result_code =
          ResolveLocation::Response::RESULT_NOT_FOUND;
          response->reason =
          "test_location_not_found";
          return;
        }

        response->resolved = true;
        response->result_code =
        ResolveLocation::Response::RESULT_RESOLVED;
        response->reason =
        "test_location_resolved";

        response->location.state =
        savo_msgs::msg::LocationRecord::STATE_APPROVED;

        response->location.enabled = true;
        response->location.location_id = "A201";
        response->location.display_name = "Room A201";
        response->location.map_id =
        config_.active_map_id;

        response->location.map_revision =
        config_.active_map_revision;

        response->location.approach_pose.header.frame_id =
        config_.map_frame;

        response->location.approach_pose.pose.position.x =
        2.5;

        response->location.approach_pose.pose.position.y =
        -1.25;

        response->location.approach_pose.pose.orientation.w =
        1.0;
      });

    navigation_action_server_ =
      rclcpp_action::create_server<NavigateToPose>(
      fixture_node_,
      config_.navigation_action_name,
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToPose::Goal>)
      {
        navigation_goal_request_count_.fetch_add(1U);

        return
          rclcpp_action::GoalResponse::
          ACCEPT_AND_EXECUTE;
      },
      [this](
        const std::shared_ptr<
          NavigationServerGoalHandle>)
      {
        navigation_cancel_request_count_.fetch_add(1U);

        return
          rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](
        const std::shared_ptr<
          NavigationServerGoalHandle> goal_handle)
      {
        navigation_goal_pose_x_.store(
          goal_handle->get_goal()->
          pose.pose.position.x);

        navigation_action_thread_ =
        std::thread(
          [this, goal_handle]()
          {
            const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(
                navigation_action_delay_ms_.load());

            while (
              std::chrono::steady_clock::now() <
              deadline)
            {
              if (goal_handle->is_canceling()) {
                auto result =
                std::make_shared<
                  NavigateToPose::Result>();

                result->error_code =
                NavigateToPose::Result::NONE;

                navigation_canceled_count_.fetch_add(
                  1U);

                goal_handle->canceled(result);

                return;
              }

              std::this_thread::sleep_for(10ms);
            }

            auto result =
            std::make_shared<
              NavigateToPose::Result>();

            result->error_code =
            NavigateToPose::Result::NONE;

            navigation_succeeded_count_.fetch_add(
              1U);

            goal_handle->succeed(result);
          });

        navigation_goal_accept_count_.fetch_add(1U);
      });

    mapping_control_service_ =
      fixture_node_->create_service<ControlAutonomousMapping>(
      config_.mapping_control_service,
      [this](
        const std::shared_ptr<
          ControlAutonomousMapping::Request> request,
        std::shared_ptr<
          ControlAutonomousMapping::Response> response)
      {
        mapping_control_request_count_.fetch_add(1U);
        last_mapping_control_command_.store(request->command);
        response->accepted = true;
        response->result_code =
        ControlAutonomousMapping::Response::RESULT_ACCEPTED;
        response->reason = "test_mapping_control_accepted";
        response->status = mapping_status_message_;
      });

    mapping_action_server_ =
      rclcpp_action::create_server<RunAutonomousMapping>(
      fixture_node_,
      config_.mapping_action_name,
      [this](
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const RunAutonomousMapping::Goal> goal)
      {
        mapping_goal_request_count_.fetch_add(1U);
        last_mapping_goal_auto_save_.store(goal->auto_save);
        last_mapping_goal_requires_approval_.store(
          goal->require_quality_approval);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<MappingServerGoalHandle>)
      {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<MappingServerGoalHandle> goal_handle)
      {
        mapping_action_thread_ = std::thread(
          [this, goal_handle]()
          {
            mapping_goal_accept_count_.fetch_add(1U);
            auto result = std::make_shared<RunAutonomousMapping::Result>();
            result->success = true;
            result->result_code =
            RunAutonomousMapping::Result::RESULT_SUCCEEDED;
            result->reason = "test_mapping_succeeded";
            result->map_saved = true;
            result->map_release_id = "release-test-1";
            result->final_status = mapping_status_message_;
            goal_handle->succeed(result);
          });
      });

    mode_command_subscription_ =
      fixture_node_->create_subscription<
      std_msgs::msg::String>(
      config_.mode_command_topic,
      reliable_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        mode_command_count_.fetch_add(1U);

        mode_state_publisher_->publish(*message);
      });

    manual_velocity_subscription_ =
      fixture_node_->create_subscription<
      geometry_msgs::msg::Twist>(
      config_.manual_velocity_topic,
      reliable_qos,
      [this](
        const geometry_msgs::msg::Twist::SharedPtr message)
      {
        if (twist_is_zero(*message)) {
          manual_zero_count_.fetch_add(1U);
        } else {
          manual_nonzero_count_.fetch_add(1U);
        }
      });

    external_stop_subscription_ =
      fixture_node_->create_subscription<
      std_msgs::msg::Bool>(
      config_.external_stop_topic,
      reliable_qos,
      [this](
        const std_msgs::msg::Bool::SharedPtr message)
      {
        last_external_stop_.store(message->data);

        if (message->data) {
          external_stop_true_count_.fetch_add(1U);
        } else {
          external_stop_false_count_.fetch_add(1U);
        }
      });

    observation_timer_ =
      fixture_node_->create_wall_timer(
      20ms,
      [this]()
      {
        publish_continuous_observations();
      });

    dispatcher_ =
      std::make_unique<RosCommandDispatcher>(
      *dispatcher_node_,
      config_);

    executor_ =
      std::make_unique<
      rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(),
      4U);

    executor_->add_node(fixture_node_);
    executor_->add_node(dispatcher_node_);

    executor_thread_ =
      std::thread(
      [this]()
      {
        executor_->spin();
      });

    std::this_thread::sleep_for(100ms);

    publish_baseline();

    ASSERT_TRUE(
      wait_for(
        [this]()
        {
          const auto state =
          dispatcher_->snapshot();

          return
            state.mode_state_observed &&
            state.mode_state == "STOP" &&
            state.external_stop_observed &&
            !state.external_stop_active &&
            state.safety_stop_observed &&
            !state.safety_stop_active &&
            state.safe_velocity_observed &&
            state.safe_velocity_zero;
        },
        1500ms))
      << "dispatcher baseline observations were not established";
  }

  void TearDown() override
  {
    if (dispatcher_) {
      dispatcher_->shutdown();
      dispatcher_.reset();
    }

    if (navigation_action_thread_.joinable()) {
      navigation_action_thread_.join();
    }

    if (mapping_action_thread_.joinable()) {
      mapping_action_thread_.join();
    }

    if (executor_) {
      executor_->cancel();
    }

    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    observation_timer_.reset();

    navigation_action_server_.reset();
    mapping_action_server_.reset();
    mapping_control_service_.reset();
    resolve_location_service_.reset();

    mode_command_subscription_.reset();
    manual_velocity_subscription_.reset();
    external_stop_subscription_.reset();

    mode_state_publisher_.reset();
    external_stop_publisher_.reset();
    safety_stop_publisher_.reset();
    safe_velocity_publisher_.reset();
    navigation_readiness_publisher_.reset();
    map_context_publisher_.reset();
    mapping_status_publisher_.reset();
    supervisor_state_publisher_.reset();

    executor_.reset();
    dispatcher_node_.reset();
    fixture_node_.reset();
  }

  template<typename Predicate>
  [[nodiscard]] bool wait_for(
    Predicate predicate,
    const std::chrono::milliseconds timeout)
  {
    const auto deadline =
      std::chrono::steady_clock::now() +
      timeout;

    while (
      std::chrono::steady_clock::now() <
      deadline)
    {
      if (predicate()) {
        return true;
      }

      std::this_thread::sleep_for(10ms);
    }

    return predicate();
  }

  void publish_continuous_observations()
  {
    std_msgs::msg::Bool safety_message;

    safety_message.data =
      safety_stop_active_.load();

    safety_stop_publisher_->publish(
      safety_message);

    geometry_msgs::msg::Twist safe_velocity;

    safe_velocity_publisher_->publish(
      safe_velocity);

    std_msgs::msg::String readiness;
    readiness.data = "READY";

    navigation_readiness_publisher_->publish(
      readiness);

    mapping_status_publisher_->publish(
      mapping_status_message_);

    std_msgs::msg::String supervisor;
    supervisor.data =
      "state=READY;startup_ready=true;system_armed=false;"
      "fault_latched=false;remote_commands_ready=true";
    supervisor_state_publisher_->publish(supervisor);
  }

  void publish_baseline()
  {
    safety_stop_active_.store(false);

    std_msgs::msg::String mode_message;
    mode_message.data = "STOP";

    std_msgs::msg::Bool external_stop_message;
    external_stop_message.data = false;

    geometry_msgs::msg::Twist safe_velocity;

    std_msgs::msg::String readiness;
    readiness.data = "READY";

    for (std::size_t index = 0U; index < 10U; ++index) {
      mode_state_publisher_->publish(
        mode_message);

      external_stop_publisher_->publish(
        external_stop_message);

      safe_velocity_publisher_->publish(
        safe_velocity);

      std_msgs::msg::Bool safety_message;
      safety_message.data = false;

      safety_stop_publisher_->publish(
        safety_message);

      navigation_readiness_publisher_->publish(
        readiness);

      std_msgs::msg::String map_context;
      map_context.data =
        "state=synchronized;map_id=test-map;map_revision=7;"
        "map_release_id=release-test-map;synchronized=true";
      map_context_publisher_->publish(map_context);

      mapping_status_publisher_->publish(
        mapping_status_message_);

      std_msgs::msg::String supervisor;
      supervisor.data =
        "state=READY;startup_ready=true;system_armed=false;"
        "fault_latched=false;remote_commands_ready=true";
      supervisor_state_publisher_->publish(supervisor);

      std::this_thread::sleep_for(20ms);
    }
  }

  [[nodiscard]] ValidatedCommand make_stop_command(
    const std::string & command_id)
  {
    ValidatedCommand command;

    command.command_id = command_id;
    command.command_type = CommandType::Stop;
    command.source = "test";
    command.origin_agent = "safety_agent";
    command.priority = CommandPriority::Emergency;
    command.issued_at_unix_ms = 1;
    command.expires_at_unix_ms = 60000;

    StopCommandPayload payload;
    payload.reason = "dispatcher_test_stop";
    payload.scope = "all_movement";

    command.payload = std::move(payload);

    return command;
  }

  [[nodiscard]] ValidatedCommand make_teleop_command(
    const std::string & command_id,
    const std::int64_t duration_ms)
  {
    ValidatedCommand command;

    command.command_id = command_id;
    command.command_type =
      CommandType::TeleopNudge;

    command.source = "test";
    command.origin_agent = "teleop_agent";
    command.priority = CommandPriority::Normal;
    command.issued_at_unix_ms = 1;
    command.expires_at_unix_ms = 60000;

    TeleopNudgeCommandPayload payload;

    payload.direction = "forward";
    payload.linear_x_mps = 0.10;
    payload.linear_y_mps = 0.0;
    payload.angular_z_radps = 0.0;
    payload.duration_ms = duration_ms;

    command.payload = std::move(payload);

    return command;
  }

  [[nodiscard]] ValidatedCommand make_cancel_command(
    const std::string & command_id,
    const std::string & target_command_id)
  {
    ValidatedCommand command;

    command.command_id = command_id;
    command.command_type =
      CommandType::CancelAction;

    command.source = "test";
    command.origin_agent = "teleop_agent";
    command.priority = CommandPriority::High;
    command.issued_at_unix_ms = 1;
    command.expires_at_unix_ms = 60000;

    CancelActionCommandPayload payload;

    payload.target_command_id =
      target_command_id;

    payload.reason =
      "dispatcher_test_cancel";

    command.payload = std::move(payload);

    return command;
  }

  [[nodiscard]] ValidatedCommand
  make_navigation_command(
    const std::string & command_id)
  {
    ValidatedCommand command;

    command.command_id = command_id;
    command.command_type =
      CommandType::NavigateToLocation;

    command.source = "test";
    command.origin_agent = "navigation_agent";
    command.priority = CommandPriority::Normal;
    command.issued_at_unix_ms = 1;
    command.expires_at_unix_ms = 60000;

    NavigateToLocationCommandPayload payload;

    payload.location_id = "A201";
    payload.map_id = config_.active_map_id;

    command.payload = std::move(payload);

    return command;
  }

  [[nodiscard]] ValidatedCommand
  make_navigation_cancel_command(
    const std::string & command_id,
    const std::string & target_command_id)
  {
    ValidatedCommand command;

    command.command_id = command_id;
    command.command_type =
      CommandType::CancelNavigation;

    command.source = "test";
    command.origin_agent = "navigation_agent";
    command.priority = CommandPriority::High;
    command.issued_at_unix_ms = 1;
    command.expires_at_unix_ms = 60000;

    CancelNavigationCommandPayload payload;

    payload.target_command_id =
      target_command_id;

    payload.reason =
      "dispatcher_test_cancel_navigation";

    command.payload = std::move(payload);

    return command;
  }

  [[nodiscard]] ValidatedCommand make_start_mapping_command(
    const std::string & command_id)
  {
    ValidatedCommand command;
    command.command_id = command_id;
    command.command_type = CommandType::StartAutonomousMapping;
    command.source = "test";
    command.origin_agent = "mapping_agent";
    command.request_id = "mapping-start-request";
    command.priority = CommandPriority::Normal;
    command.issued_at_unix_ms = 1;
    command.expires_at_unix_ms = 60000;

    StartAutonomousMappingCommandPayload payload;
    payload.mission_id = "mission-test-1";
    payload.map_id = "mapping-test-map";
    payload.map_revision = 3U;
    payload.mission_timeout_ms = 5000;
    payload.auto_save = true;
    payload.require_quality_approval = true;
    command.payload = std::move(payload);
    return command;
  }

  [[nodiscard]] ValidatedCommand make_mapping_control_command(
    const std::string & command_id,
    const std::string & operation)
  {
    ValidatedCommand command;
    command.command_id = command_id;
    command.command_type = CommandType::ControlMapping;
    command.source = "test";
    command.origin_agent = "mapping_agent";
    command.request_id = "mapping-control-request";
    command.priority = CommandPriority::Normal;
    command.issued_at_unix_ms = 1;
    command.expires_at_unix_ms = 60000;

    ControlMappingCommandPayload payload;
    payload.operation = operation;
    payload.mission_id = "mission-test-1";
    payload.reason = "dispatcher_mapping_control_test";
    command.payload = std::move(payload);
    return command;
  }

  [[nodiscard]] ValidatedCommand make_query_command(
    const std::string & command_id,
    const CommandType type,
    const std::string & agent,
    const std::string & scope)
  {
    ValidatedCommand command;
    command.command_id = command_id;
    command.command_type = type;
    command.source = "test";
    command.origin_agent = agent;
    command.request_id = "query-request";
    command.priority = CommandPriority::Normal;
    command.issued_at_unix_ms = 1;
    command.expires_at_unix_ms = 60000;

    QueryStateCommandPayload payload;
    payload.scope = scope;
    command.payload = std::move(payload);
    return command;
  }

  RosCommandDispatcherConfig config_;

  std::shared_ptr<rclcpp::Node> fixture_node_;
  std::shared_ptr<rclcpp::Node> dispatcher_node_;

  std::unique_ptr<RosCommandDispatcher>
  dispatcher_;

  std::unique_ptr<
    rclcpp::executors::MultiThreadedExecutor>
  executor_;

  std::thread executor_thread_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    mode_state_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    external_stop_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::Bool>::SharedPtr
    safety_stop_publisher_;

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr
    safe_velocity_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    navigation_readiness_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    map_context_publisher_;

  rclcpp::Publisher<
    savo_msgs::msg::AutonomousMappingStatus>::SharedPtr
    mapping_status_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::String>::SharedPtr
    supervisor_state_publisher_;

  rclcpp::Service<ResolveLocation>::SharedPtr
    resolve_location_service_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr
    navigation_action_server_;

  rclcpp::Service<ControlAutonomousMapping>::SharedPtr
    mapping_control_service_;

  rclcpp_action::Server<RunAutonomousMapping>::SharedPtr
    mapping_action_server_;

  rclcpp::Subscription<
    std_msgs::msg::String>::SharedPtr
    mode_command_subscription_;

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr
    manual_velocity_subscription_;

  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr
    external_stop_subscription_;

  rclcpp::TimerBase::SharedPtr
    observation_timer_;

  std::atomic<bool> safety_stop_active_{false};
  std::atomic<bool> last_external_stop_{false};

  std::atomic<std::size_t>
  manual_nonzero_count_{0U};

  std::atomic<std::size_t>
  manual_zero_count_{0U};

  std::atomic<std::size_t>
  mode_command_count_{0U};

  std::atomic<std::size_t>
  external_stop_true_count_{0U};

  std::atomic<std::size_t>
  external_stop_false_count_{0U};

  savo_msgs::msg::AutonomousMappingStatus mapping_status_message_;

  std::thread navigation_action_thread_;
  std::thread mapping_action_thread_;

  std::atomic<std::size_t> mapping_control_request_count_{0U};
  std::atomic<std::uint8_t> last_mapping_control_command_{0U};
  std::atomic<std::size_t> mapping_goal_request_count_{0U};
  std::atomic<std::size_t> mapping_goal_accept_count_{0U};
  std::atomic<bool> last_mapping_goal_auto_save_{false};
  std::atomic<bool> last_mapping_goal_requires_approval_{false};

  std::atomic<std::int64_t>
  navigation_action_delay_ms_{100};

  std::atomic<std::size_t>
  resolve_request_count_{0U};

  std::atomic<std::size_t>
  navigation_goal_request_count_{0U};

  std::atomic<std::size_t>
  navigation_goal_accept_count_{0U};

  std::atomic<std::size_t>
  navigation_cancel_request_count_{0U};

  std::atomic<std::size_t>
  navigation_succeeded_count_{0U};

  std::atomic<std::size_t>
  navigation_canceled_count_{0U};

  std::atomic<double>
  navigation_goal_pose_x_{0.0};

  static std::atomic<std::uint64_t>
  next_instance_;
};

std::atomic<std::uint64_t>
RosCommandDispatcherTest::next_instance_{1U};

TEST_F(
  RosCommandDispatcherTest,
  StopRequiresFreshDownstreamConfirmation)
{
  const auto result =
    dispatcher_->dispatch(
    make_stop_command("stop-confirmed-1"));

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.state, "accepted");
  EXPECT_EQ(result.reason, "bridge_stop_confirmed");
  EXPECT_TRUE(result.dispatch_attempted);

  EXPECT_GE(
    result.ros_publications,
    config_.final_zero_publication_count + 2U);

  const auto state =
    dispatcher_->snapshot();

  EXPECT_TRUE(state.external_stop_observed);
  EXPECT_TRUE(state.external_stop_active);

  EXPECT_TRUE(state.mode_state_observed);
  EXPECT_EQ(state.mode_state, "STOP");

  EXPECT_TRUE(state.safe_velocity_observed);
  EXPECT_TRUE(state.safe_velocity_zero);

  EXPECT_EQ(
    state.last_terminal_command_id,
    "stop-confirmed-1");

  EXPECT_EQ(
    state.last_reason,
    "bridge_stop_confirmed");
}

TEST_F(
  RosCommandDispatcherTest,
  TeleopPublishesBoundedManualCommandAndFinalZero)
{
  const auto result =
    dispatcher_->dispatch(
    make_teleop_command(
      "teleop-complete-1",
      300));

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.reason, "bridge_teleop_admitted");

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        return
          manual_nonzero_count_.load() > 0U;
      },
      1200ms))
    << "teleop never published a non-zero manual command";

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        const auto state =
        dispatcher_->snapshot();

        return
          !state.teleop_active &&
          !state.command_active &&
          state.last_terminal_command_id ==
          "teleop-complete-1";
      },
      2000ms))
    << "teleop did not reach its bounded terminal state";

  const auto state =
    dispatcher_->snapshot();

  EXPECT_EQ(
    state.last_reason,
    "bridge_teleop_completed");

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        return
          manual_zero_count_.load() >=
          config_.final_zero_publication_count;
      },
      1200ms))
    << "final zero manual-velocity publications were not observed";

  EXPECT_GT(
    mode_command_count_.load(),
    0U);
}

TEST_F(
  RosCommandDispatcherTest,
  TeleopRuntimeSafetyLossForcesExternalStop)
{
  const auto result =
    dispatcher_->dispatch(
    make_teleop_command(
      "teleop-safety-loss-1",
      900));

  ASSERT_TRUE(result.accepted);

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        return
          manual_nonzero_count_.load() > 0U;
      },
      1200ms));

  safety_stop_active_.store(true);

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        const auto state =
        dispatcher_->snapshot();

        return
          !state.teleop_active &&
          state.last_terminal_command_id ==
          "teleop-safety-loss-1";
      },
      2000ms))
    << "teleop did not terminate after safety-state loss";

  const auto state =
    dispatcher_->snapshot();

  EXPECT_EQ(
    state.last_reason,
    "bridge_teleop_runtime_state_lost");

  EXPECT_TRUE(
    wait_for(
      [this]()
      {
        return
          external_stop_true_count_.load() > 0U &&
          last_external_stop_.load();
      },
      1000ms));

  EXPECT_GE(
    manual_zero_count_.load(),
    config_.final_zero_publication_count);
}

TEST_F(
  RosCommandDispatcherTest,
  CancelActionRequiresExactActiveTeleopCommandId)
{
  const auto teleop_result =
    dispatcher_->dispatch(
    make_teleop_command(
      "teleop-cancel-target-1",
      1000));

  ASSERT_TRUE(teleop_result.accepted);

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        return
          manual_nonzero_count_.load() > 0U;
      },
      1200ms));

  const auto wrong_target_result =
    dispatcher_->dispatch(
    make_cancel_command(
      "cancel-wrong-target-1",
      "another-command-id"));

  EXPECT_FALSE(wrong_target_result.accepted);

  EXPECT_EQ(
    wrong_target_result.reason,
    "bridge_cancel_target_mismatch");

  EXPECT_TRUE(
    dispatcher_->snapshot().teleop_active);

  const auto cancel_result =
    dispatcher_->dispatch(
    make_cancel_command(
      "cancel-exact-target-1",
      "teleop-cancel-target-1"));

  ASSERT_TRUE(cancel_result.accepted);

  EXPECT_EQ(
    cancel_result.reason,
    "bridge_cancel_confirmed");

  const auto state =
    dispatcher_->snapshot();

  EXPECT_FALSE(state.teleop_active);
  EXPECT_FALSE(state.command_active);
  EXPECT_FALSE(state.teleop_cancel_requested);

  EXPECT_EQ(
    state.last_terminal_command_id,
    "teleop-cancel-target-1");

  EXPECT_EQ(
    state.last_reason,
    "bridge_cancel_confirmed");

  EXPECT_TRUE(
    wait_for(
      [this]()
      {
        return
          external_stop_true_count_.load() > 0U &&
          last_external_stop_.load();
      },
      1000ms));
}


TEST_F(
  RosCommandDispatcherTest,
  NavigationResolvesExactApprovedLocationAndUsesGuardedAction)
{
  const auto result =
    dispatcher_->dispatch(
    make_navigation_command(
      "navigation-success-1"));

  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(
    result.reason,
    "bridge_navigation_admitted");

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        return
          navigation_goal_accept_count_.load() > 0U;
      },
      2000ms))
    << "guarded navigation action did not receive the goal";

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        const auto state =
        dispatcher_->snapshot();

        return
          !state.navigation_goal_active &&
          !state.command_active &&
          state.last_terminal_command_id ==
          "navigation-success-1";
      },
      2500ms))
    << "navigation did not reach a terminal result";

  const auto state =
    dispatcher_->snapshot();

  EXPECT_EQ(
    state.last_reason,
    "bridge_navigation_succeeded");

  EXPECT_EQ(
    resolve_request_count_.load(),
    1U);

  EXPECT_EQ(
    navigation_goal_request_count_.load(),
    1U);

  EXPECT_EQ(
    navigation_succeeded_count_.load(),
    1U);

  EXPECT_NEAR(
    navigation_goal_pose_x_.load(),
    2.5,
    1.0e-9);
}

TEST_F(
  RosCommandDispatcherTest,
  NavigationCancellationRequiresExactTargetAndWaitsForTerminal)
{
  navigation_action_delay_ms_.store(5000);

  const auto navigation_result =
    dispatcher_->dispatch(
    make_navigation_command(
      "navigation-cancel-target-1"));

  ASSERT_TRUE(navigation_result.accepted);

  ASSERT_TRUE(
    wait_for(
      [this]()
      {
        return
          navigation_goal_accept_count_.load() > 0U &&
          dispatcher_->snapshot().
          navigation_goal_active;
      },
      2000ms))
    << "navigation goal was not accepted";

  const auto wrong_target_result =
    dispatcher_->dispatch(
    make_navigation_cancel_command(
      "navigation-cancel-wrong-1",
      "different-navigation-command"));

  EXPECT_FALSE(wrong_target_result.accepted);

  EXPECT_EQ(
    wrong_target_result.reason,
    "bridge_navigation_cancel_target_mismatch");

  const auto cancel_result =
    dispatcher_->dispatch(
    make_navigation_cancel_command(
      "navigation-cancel-exact-1",
      "navigation-cancel-target-1"));

  ASSERT_TRUE(cancel_result.accepted);

  EXPECT_EQ(
    cancel_result.reason,
    "bridge_navigation_cancel_confirmed");

  const auto state =
    dispatcher_->snapshot();

  EXPECT_FALSE(state.navigation_goal_active);
  EXPECT_FALSE(state.navigation_cancel_requested);
  EXPECT_FALSE(state.command_active);

  EXPECT_EQ(
    state.last_terminal_command_id,
    "navigation-cancel-target-1");

  EXPECT_EQ(
    state.last_reason,
    "bridge_navigation_cancel_confirmed");

  EXPECT_GT(
    navigation_cancel_request_count_.load(),
    0U);

  EXPECT_EQ(
    navigation_canceled_count_.load(),
    1U);
}


TEST_F(
  RosCommandDispatcherTest,
  AutonomousMappingUsesTypedActionAndKeepsApprovalRequired)
{
  const auto result = dispatcher_->dispatch(
    make_start_mapping_command("mapping-start-dispatch-1"));

  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.reason, "bridge_mapping_goal_accepted");
  EXPECT_TRUE(result.dispatch_attempted);

  ASSERT_TRUE(wait_for(
      [this]()
      {
        return mapping_goal_accept_count_.load() == 1U;
      },
      1500ms));

  EXPECT_EQ(mapping_goal_request_count_.load(), 1U);
  EXPECT_TRUE(last_mapping_goal_auto_save_.load());
  EXPECT_TRUE(last_mapping_goal_requires_approval_.load());
}

TEST_F(
  RosCommandDispatcherTest,
  MappingScan360UsesTypedControlService)
{
  const auto result = dispatcher_->dispatch(
    make_mapping_control_command(
      "mapping-scan360-dispatch-1",
      "request_scan360"));

  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(
    result.reason,
    "bridge_mapping_control_accepted:test_mapping_control_accepted");
  EXPECT_EQ(mapping_control_request_count_.load(), 1U);
  EXPECT_EQ(
    last_mapping_control_command_.load(),
    ControlAutonomousMapping::Request::COMMAND_REQUEST_SCAN360);
}

TEST_F(
  RosCommandDispatcherTest,
  MappingQueryReturnsStructuredAm8State)
{
  ASSERT_TRUE(wait_for(
      [this]()
      {
        return dispatcher_->snapshot().mapping_status_observed;
      },
      1000ms));

  const auto result = dispatcher_->dispatch(
    make_query_command(
      "mapping-query-dispatch-1",
      CommandType::QueryMappingState,
      "mapping_agent",
      "mapping"));

  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_EQ(result.reason, "bridge_mapping_state_available");

  const auto document = nlohmann::json::parse(result.result_json);
  EXPECT_EQ(document.at("mission_id"), "mission-test-1");
  EXPECT_EQ(document.at("state"), "awaiting_approval");
  EXPECT_TRUE(document.at("map_save").at("saved"));
  EXPECT_TRUE(document.at("verification").at("passed"));
  EXPECT_TRUE(document.at("review").at("approval_pending"));
  EXPECT_EQ(document.at("review").at("generation"), 8U);
  EXPECT_EQ(document.at("release").at("state"), "awaiting_approval");
  EXPECT_FALSE(
    document.at("release").at("joint_active_release_verified"));
}

TEST_F(
  RosCommandDispatcherTest,
  SupervisorQueryReturnsStructuredReadOnlySummary)
{
  ASSERT_TRUE(wait_for(
      [this]()
      {
        return dispatcher_->snapshot().supervisor_state_observed;
      },
      1000ms));

  const auto result = dispatcher_->dispatch(
    make_query_command(
      "supervisor-query-dispatch-1",
      CommandType::QuerySupervisorState,
      "supervisor_agent",
      "supervisor"));

  ASSERT_TRUE(result.accepted) << result.reason;
  const auto document = nlohmann::json::parse(result.result_json);
  EXPECT_NE(
    document.at("summary").get<std::string>().find("startup_ready=true"),
    std::string::npos);
}

}  // namespace

}  // namespace savo_bridge
