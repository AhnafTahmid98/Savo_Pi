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
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/msg/location_record.hpp"
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

    if (executor_) {
      executor_->cancel();
    }

    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    observation_timer_.reset();

    navigation_action_server_.reset();
    resolve_location_service_.reset();

    mode_command_subscription_.reset();
    manual_velocity_subscription_.reset();
    external_stop_subscription_.reset();

    mode_state_publisher_.reset();
    external_stop_publisher_.reset();
    safety_stop_publisher_.reset();
    safe_velocity_publisher_.reset();
    navigation_readiness_publisher_.reset();

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

  rclcpp::Service<ResolveLocation>::SharedPtr
    resolve_location_service_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr
    navigation_action_server_;

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

  std::thread navigation_action_thread_;

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

  EXPECT_GE(
    manual_zero_count_.load(),
    config_.final_zero_publication_count);

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

}  // namespace

}  // namespace savo_bridge
