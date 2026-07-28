#include "savo_mapping/scan360_rotate_action_binding.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/action/rotate_to_heading.hpp"

namespace
{

namespace scan360 =
  savo_mapping::scan360;

using NativeClient =
  savo_mapping::Scan360RotateActionClient;

using RotateAction =
  savo_msgs::action::RotateToHeading;

using ServerGoalHandle =
  rclcpp_action::ServerGoalHandle<RotateAction>;

struct StateMapping
{
  NativeClient::State native_state;
  scan360::RotationClientState expected_state;
};

NativeClient::Snapshot native_snapshot(
  const NativeClient::State state,
  std::string reason = "native_reason")
{
  NativeClient::Snapshot snapshot;
  snapshot.state = state;
  snapshot.reason = std::move(reason);
  return snapshot;
}

void construct_callbacks_with_null_client()
{
  const auto callbacks =
    scan360::make_scan360_rotate_action_callbacks(
    NativeClient::SharedPtr{});

  (void)callbacks;
}

class BindingFixtureServer
{
public:
  BindingFixtureServer(
    rclcpp::Node::SharedPtr node,
    std::string action_name)
  : node_(std::move(node)),
    action_name_(std::move(action_name))
  {
    server_ =
      rclcpp_action::create_server<RotateAction>(
      node_,
      action_name_,
      std::bind(
        &BindingFixtureServer::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &BindingFixtureServer::handle_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &BindingFixtureServer::handle_accepted,
        this,
        std::placeholders::_1));
  }

  ~BindingFixtureServer()
  {
    stop_requested_.store(true);

    std::vector<std::thread> workers;

    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      workers.swap(workers_);
    }

    for (auto & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  BindingFixtureServer(
    const BindingFixtureServer &) = delete;

  BindingFixtureServer & operator=(
    const BindingFixtureServer &) = delete;

  [[nodiscard]] int goal_count() const noexcept
  {
    return goal_count_.load();
  }

  [[nodiscard]] int cancel_count() const noexcept
  {
    return cancel_count_.load();
  }

  [[nodiscard]] double target_yaw_rad() const noexcept
  {
    return target_yaw_rad_.load();
  }

  [[nodiscard]] double max_duration_sec() const noexcept
  {
    return max_duration_sec_.load();
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<
      const RotateAction::Goal> goal)
  {
    goal_count_.fetch_add(1);

    if (goal) {
      target_yaw_rad_.store(
        goal->target_yaw_rad);

      max_duration_sec_.store(
        goal->max_duration_sec);
    }

    return
      rclcpp_action::GoalResponse::
      ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ServerGoalHandle>)
  {
    cancel_count_.fetch_add(1);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<ServerGoalHandle>
    goal_handle)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);

    workers_.emplace_back(
      [this, goal_handle]()
      {
        while (
          rclcpp::ok() &&
          !stop_requested_.load())
        {
          if (goal_handle->is_canceling()) {
            auto result =
            std::make_shared<RotateAction::Result>();

            result->success = false;
            result->reason = "canceled";
            goal_handle->canceled(result);
            return;
          }

          std::this_thread::sleep_for(
            std::chrono::milliseconds(5));
        }
      });
  }

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;

  rclcpp_action::Server<
    RotateAction>::SharedPtr server_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<int> goal_count_{0};
  std::atomic<int> cancel_count_{0};
  std::atomic<double> target_yaw_rad_{0.0};
  std::atomic<double> max_duration_sec_{0.0};

  std::mutex worker_mutex_;
  std::vector<std::thread> workers_;
};

class BindingRuntimeHarness
{
public:
  explicit BindingRuntimeHarness(
    const bool create_server,
    NativeClient::Options options =
    NativeClient::Options{})
  {
    const auto identifier =
      next_identifier_.fetch_add(1);

    const auto suffix =
      std::to_string(identifier);

    action_name_ =
      "/fixture/scan360_rotate_action_binding_" +
      suffix;

    client_node_ =
      std::make_shared<rclcpp::Node>(
      "scan360_binding_client_" + suffix);

    if (create_server) {
      server_node_ =
        std::make_shared<rclcpp::Node>(
        "scan360_binding_server_" + suffix);

      server_ =
        std::make_shared<BindingFixtureServer>(
        server_node_,
        action_name_);
    }

    options.action_name = action_name_;

    client_ = NativeClient::create(
      client_node_,
      std::move(options));

    callbacks_ =
      scan360::make_scan360_rotate_action_callbacks(
      client_);

    executor_ = std::make_shared<
      rclcpp::executors::MultiThreadedExecutor>();

    executor_->add_node(client_node_);

    if (server_node_) {
      executor_->add_node(server_node_);
    }

    spin_thread_ =
      std::thread(
      [this]()
      {
        executor_->spin();
      });
  }

  ~BindingRuntimeHarness()
  {
    callbacks_ = {};
    client_.reset();

    if (executor_) {
      executor_->cancel();
    }

    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    server_.reset();
    server_node_.reset();
    client_node_.reset();
    executor_.reset();
  }

  BindingRuntimeHarness(
    const BindingRuntimeHarness &) = delete;

  BindingRuntimeHarness & operator=(
    const BindingRuntimeHarness &) = delete;

  [[nodiscard]] scan360::Scan360RotationCallbacks &
  callbacks()
  {
    return callbacks_;
  }

  [[nodiscard]]
  std::shared_ptr<BindingFixtureServer>
  server() const
  {
    return server_;
  }

private:
  static std::atomic<unsigned int>
  next_identifier_;

  std::string action_name_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;

  std::shared_ptr<BindingFixtureServer> server_;
  NativeClient::SharedPtr client_;
  scan360::Scan360RotationCallbacks callbacks_;

  std::shared_ptr<
    rclcpp::executors::MultiThreadedExecutor>
  executor_;

  std::thread spin_thread_;
};

std::atomic<unsigned int>
BindingRuntimeHarness::next_identifier_{1U};

template<typename Predicate>
bool wait_until(
  scan360::Scan360RotationCallbacks & callbacks,
  Predicate predicate,
  const std::chrono::milliseconds timeout =
  std::chrono::milliseconds(4000))
{
  const auto deadline =
    std::chrono::steady_clock::now() + timeout;

  while (
    rclcpp::ok() &&
    std::chrono::steady_clock::now() < deadline)
  {
    callbacks.tick();

    if (predicate(callbacks.snapshot())) {
      return true;
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(5));
  }

  callbacks.tick();
  return predicate(callbacks.snapshot());
}

class Scan360RotateActionBindingTest
  : public testing::Test
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
};

TEST_F(
  Scan360RotateActionBindingTest,
  NullNativeClientIsRejected)
{
  EXPECT_THROW(
    construct_callbacks_with_null_client(),
    std::invalid_argument);
}

TEST_F(
  Scan360RotateActionBindingTest,
  EveryNativeStateMapsToExpectedPhase)
{
  const std::vector<StateMapping> mappings{
    {
      NativeClient::State::kIdle,
      scan360::RotationClientState::Idle},
    {
      NativeClient::State::kWaitingForServer,
      scan360::RotationClientState::Pending},
    {
      NativeClient::State::kWaitingForGoalResponse,
      scan360::RotationClientState::Pending},
    {
      NativeClient::State::kActive,
      scan360::RotationClientState::Active},
    {
      NativeClient::State::kCanceling,
      scan360::RotationClientState::Canceling},
    {
      NativeClient::State::kSucceeded,
      scan360::RotationClientState::Succeeded},
    {
      NativeClient::State::kCanceled,
      scan360::RotationClientState::Canceled},
    {
      NativeClient::State::kRejected,
      scan360::RotationClientState::Rejected},
    {
      NativeClient::State::kAborted,
      scan360::RotationClientState::Failed},
    {
      NativeClient::State::kTimedOut,
      scan360::RotationClientState::Failed},
    {
      NativeClient::State::kFailed,
      scan360::RotationClientState::Failed},
  };

  for (const auto & mapping : mappings) {
    const auto mapped =
      scan360::map_scan360_rotate_action_snapshot(
      native_snapshot(mapping.native_state));

    EXPECT_EQ(mapped.state, mapping.expected_state);
  }
}

TEST_F(
  Scan360RotateActionBindingTest,
  NativeReasonIsPreservedExactly)
{
  const std::vector<std::string> reasons{
    "goal_reached",
    "canceled",
    "timeout",
    "safety_stop",
    "odom_stale",
    "disabled",
    "scan360_action_server_unavailable",
    "scan360_goal_response_timeout",
    "scan360_goal_rejected",
    "scan360_feedback_stale",
    "scan360_rotation_timeout",
    "scan360_cancel_timeout",
    "scan360_cancel_rejected",
    "scan360_action_unknown_result",
  };

  for (const auto & reason : reasons) {
    const auto mapped =
      scan360::map_scan360_rotate_action_snapshot(
      native_snapshot(
        NativeClient::State::kFailed,
        reason));

    EXPECT_EQ(mapped.reason, reason);
    EXPECT_EQ(
      mapped.state,
      scan360::RotationClientState::Failed);
  }
}

TEST_F(
  Scan360RotateActionBindingTest,
  CallbacksRetainNativeClientLifetime)
{
  auto node =
    std::make_shared<rclcpp::Node>(
    "scan360_binding_lifetime_test");

  NativeClient::Options options;

  options.action_name =
    "/fixture/scan360_rotate_action_binding_lifetime";

  auto client = NativeClient::create(
    node,
    std::move(options));

  std::weak_ptr<NativeClient> weak_client =
    client;

  auto callbacks =
    scan360::make_scan360_rotate_action_callbacks(
    client);

  client.reset();

  EXPECT_FALSE(weak_client.expired());

  EXPECT_EQ(
    callbacks.snapshot().state,
    scan360::RotationClientState::Idle);

  callbacks = {};

  EXPECT_TRUE(weak_client.expired());
}

TEST_F(
  Scan360RotateActionBindingTest,
  RotationAndCancellationRequestsAreForwarded)
{
  BindingRuntimeHarness harness(true);
  auto & callbacks = harness.callbacks();

  ASSERT_TRUE(
    callbacks.request_rotation(0.75, 2.5));

  ASSERT_TRUE(
    wait_until(
      callbacks,
      [](const scan360::RotationClientSnapshot & snapshot)
      {
        return
          snapshot.state ==
          scan360::RotationClientState::Active;
      }));

  ASSERT_TRUE(harness.server());
  EXPECT_EQ(harness.server()->goal_count(), 1);
  EXPECT_NEAR(
    harness.server()->target_yaw_rad(),
    0.75,
    1.0e-9);
  EXPECT_NEAR(
    harness.server()->max_duration_sec(),
    2.5,
    1.0e-9);

  ASSERT_TRUE(callbacks.request_cancel());

  ASSERT_TRUE(
    wait_until(
      callbacks,
      [](const scan360::RotationClientSnapshot & snapshot)
      {
        return
          snapshot.state ==
          scan360::RotationClientState::Canceled;
      }));

  EXPECT_GE(harness.server()->cancel_count(), 1);
}

TEST_F(
  Scan360RotateActionBindingTest,
  TickIsForwardedToNativeClient)
{
  NativeClient::Options options;
  options.server_wait_timeout_sec = 0.05;

  BindingRuntimeHarness harness(
    false,
    options);

  auto & callbacks = harness.callbacks();

  ASSERT_TRUE(
    callbacks.request_rotation(0.25, 1.0));

  std::this_thread::sleep_for(
    std::chrono::milliseconds(80));

  EXPECT_EQ(
    callbacks.snapshot().state,
    scan360::RotationClientState::Pending);

  callbacks.tick();

  const auto snapshot = callbacks.snapshot();

  EXPECT_EQ(
    snapshot.state,
    scan360::RotationClientState::Failed);

  EXPECT_EQ(
    snapshot.reason,
    "scan360_action_server_unavailable");
}

TEST_F(
  Scan360RotateActionBindingTest,
  BindingAddsNoControllerEventOrdering)
{
  const auto mapped =
    scan360::map_scan360_rotate_action_snapshot(
    native_snapshot(
      NativeClient::State::kSucceeded,
      "goal_reached"));

  EXPECT_EQ(
    mapped.state,
    scan360::RotationClientState::Succeeded);

  EXPECT_EQ(mapped.reason, "goal_reached");
}

}  // namespace
