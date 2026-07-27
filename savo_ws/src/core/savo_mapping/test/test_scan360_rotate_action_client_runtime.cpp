#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_mapping/scan360_rotate_action_client.hpp"
#include "savo_msgs/action/rotate_to_heading.hpp"

namespace
{

using RotateAction =
  savo_msgs::action::RotateToHeading;

using ClientAdapter =
  savo_mapping::Scan360RotateActionClient;

using ServerGoalHandle =
  rclcpp_action::ServerGoalHandle<RotateAction>;

constexpr double kPi =
  3.14159265358979323846;

class FakeRotateToHeadingServer
{
public:
  enum class Mode
  {
    kSucceed,
    kRejectGoal,
    kAbort,
    kHoldUntilCancel,
    kHoldWithoutFeedback,
    kRejectCancel,
  };

  FakeRotateToHeadingServer(
    rclcpp::Node::SharedPtr node,
    std::string action_name,
    const Mode mode)
  : node_(std::move(node)),
    action_name_(std::move(action_name)),
    mode_(mode)
  {
    server_ =
      rclcpp_action::create_server<RotateAction>(
      node_,
      action_name_,
      std::bind(
        &FakeRotateToHeadingServer::handle_goal,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      std::bind(
        &FakeRotateToHeadingServer::handle_cancel,
        this,
        std::placeholders::_1),
      std::bind(
        &FakeRotateToHeadingServer::handle_accepted,
        this,
        std::placeholders::_1));
  }

  ~FakeRotateToHeadingServer()
  {
    stop_requested_.store(true);

    std::vector<std::thread> workers;

    {
      std::lock_guard<std::mutex> lock(
        worker_mutex_);

      workers.swap(workers_);
    }

    for (auto & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  FakeRotateToHeadingServer(
    const FakeRotateToHeadingServer &) = delete;

  FakeRotateToHeadingServer & operator=(
    const FakeRotateToHeadingServer &) = delete;

  [[nodiscard]] int goal_count() const noexcept
  {
    return goal_count_.load();
  }

  [[nodiscard]] int cancel_count() const noexcept
  {
    return cancel_count_.load();
  }

  [[nodiscard]] int feedback_count() const noexcept
  {
    return feedback_count_.load();
  }

  [[nodiscard]] double last_target_yaw() const noexcept
  {
    return last_target_yaw_.load();
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<
      const RotateAction::Goal> goal)
  {
    goal_count_.fetch_add(1);

    if (goal) {
      last_target_yaw_.store(
        goal->target_yaw_rad);
    }

    if (mode_ == Mode::kRejectGoal) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    return
      rclcpp_action::GoalResponse::
      ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<ServerGoalHandle>)
  {
    cancel_count_.fetch_add(1);

    if (mode_ == Mode::kRejectCancel) {
      return
        rclcpp_action::CancelResponse::REJECT;
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<ServerGoalHandle>
    goal_handle)
  {
    std::lock_guard<std::mutex> lock(
      worker_mutex_);

    workers_.emplace_back(
      [this, goal_handle]()
      {
        execute(goal_handle);
      });
  }

  void execute(
    const std::shared_ptr<ServerGoalHandle> &
    goal_handle)
  {
    if (!goal_handle) {
      return;
    }

    const auto goal = goal_handle->get_goal();

    if (!goal) {
      return;
    }

    if (mode_ == Mode::kSucceed) {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(30));

      if (stop_requested_.load()) {
        return;
      }

      auto feedback =
        std::make_shared<RotateAction::Feedback>();

      feedback->current_yaw_rad =
        goal->target_yaw_rad - 0.10;

      feedback->target_yaw_rad =
        goal->target_yaw_rad;

      feedback->error_rad = 0.10;
      feedback->commanded_wz_rad_s = 0.25;
      feedback->elapsed_sec = 0.05;
      feedback->safety_stop_active = false;
      feedback->state = "rotating";

      goal_handle->publish_feedback(feedback);
      feedback_count_.fetch_add(1);

      std::this_thread::sleep_for(
        std::chrono::milliseconds(30));

      if (stop_requested_.load()) {
        return;
      }

      auto result =
        std::make_shared<RotateAction::Result>();

      result->success = true;

      result->final_yaw_rad =
        goal->target_yaw_rad;

      result->final_error_rad = 0.0;
      result->reason = "goal_reached";

      goal_handle->succeed(result);
      return;
    }

    if (mode_ == Mode::kAbort) {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(40));

      if (stop_requested_.load()) {
        return;
      }

      auto result =
        std::make_shared<RotateAction::Result>();

      result->success = false;

      result->final_yaw_rad =
        goal->target_yaw_rad - 0.40;

      result->final_error_rad = 0.40;
      result->reason = "safety_stop";

      goal_handle->abort(result);
      return;
    }

    while (
      rclcpp::ok() &&
      !stop_requested_.load())
    {
      if (goal_handle->is_canceling()) {
        auto result =
          std::make_shared<RotateAction::Result>();

        result->success = false;

        result->final_yaw_rad =
          goal->target_yaw_rad - 0.20;

        result->final_error_rad = 0.20;
        result->reason = "canceled";

        goal_handle->canceled(result);
        return;
      }

      std::this_thread::sleep_for(
        std::chrono::milliseconds(5));
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  Mode mode_;

  rclcpp_action::Server<
    RotateAction>::SharedPtr server_;

  std::atomic<bool> stop_requested_{false};
  std::atomic<int> goal_count_{0};
  std::atomic<int> cancel_count_{0};
  std::atomic<int> feedback_count_{0};
  std::atomic<double> last_target_yaw_{0.0};

  std::mutex worker_mutex_;
  std::vector<std::thread> workers_;
};


class RuntimeHarness
{
public:
  RuntimeHarness(
    const bool create_server,
    const FakeRotateToHeadingServer::Mode mode,
    ClientAdapter::Options options)
  {
    const auto identifier =
      next_identifier_.fetch_add(1);

    const auto suffix =
      std::to_string(identifier);

    action_name_ =
      "/fixture/scan360_rotate_to_heading_" +
      suffix;

    client_node_ = std::make_shared<rclcpp::Node>(
      "scan360_rotate_client_test_" + suffix);

    if (create_server) {
      server_node_ =
        std::make_shared<rclcpp::Node>(
        "scan360_rotate_server_test_" + suffix);

      server_ =
        std::make_shared<
        FakeRotateToHeadingServer>(
        server_node_,
        action_name_,
        mode);
    }

    options.action_name = action_name_;

    adapter_ = ClientAdapter::create(
      client_node_,
      std::move(options));

    executor_ = std::make_shared<
      rclcpp::executors::MultiThreadedExecutor>();

    executor_->add_node(client_node_);

    if (server_node_) {
      executor_->add_node(server_node_);
    }

    spin_thread_ = std::thread(
      [this]()
      {
        executor_->spin();
      });
  }

  ~RuntimeHarness()
  {
    if (adapter_) {
      adapter_->set_update_callback({});
      adapter_.reset();
    }

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

  RuntimeHarness(const RuntimeHarness &) = delete;

  RuntimeHarness & operator=(
    const RuntimeHarness &) = delete;

  [[nodiscard]] ClientAdapter::SharedPtr
  adapter() const
  {
    return adapter_;
  }

  [[nodiscard]]
  std::shared_ptr<FakeRotateToHeadingServer>
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

  std::shared_ptr<FakeRotateToHeadingServer>
  server_;

  ClientAdapter::SharedPtr adapter_;

  std::shared_ptr<
    rclcpp::executors::MultiThreadedExecutor>
  executor_;

  std::thread spin_thread_;
};


std::atomic<unsigned int>
RuntimeHarness::next_identifier_{1U};


ClientAdapter::Options default_options()
{
  ClientAdapter::Options options;

  options.server_wait_timeout_sec = 1.5;
  options.goal_response_timeout_sec = 1.0;
  options.feedback_stale_timeout_sec = 1.0;
  options.cancel_timeout_sec = 0.5;
  options.execution_grace_sec = 0.0;

  return options;
}


template<typename Predicate>
bool wait_until(
  const ClientAdapter::SharedPtr & adapter,
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
    adapter->tick();

    if (predicate(adapter->snapshot())) {
      return true;
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(5));
  }

  adapter->tick();
  return predicate(adapter->snapshot());
}


class Scan360RotateActionClientRuntimeTest
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
  Scan360RotateActionClientRuntimeTest,
  RejectsInvalidRequestsAndNormalizesYaw)
{
  auto options = default_options();

  RuntimeHarness harness(
    false,
    FakeRotateToHeadingServer::Mode::kSucceed,
    options);

  const auto adapter = harness.adapter();

  EXPECT_FALSE(
    adapter->request_rotation(
      std::numeric_limits<double>::quiet_NaN(),
      1.0));

  EXPECT_FALSE(
    adapter->request_rotation(0.0, 0.0));

  EXPECT_FALSE(
    adapter->request_rotation(0.0, -1.0));

  const double normalized =
    ClientAdapter::normalize_yaw(3.0 * kPi);

  EXPECT_NEAR(
    std::abs(normalized),
    kPi,
    1.0e-9);

  const auto snapshot = adapter->snapshot();

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kIdle);

  EXPECT_FALSE(snapshot.active);
  EXPECT_FALSE(snapshot.terminal);
}


TEST_F(
  Scan360RotateActionClientRuntimeTest,
  TimesOutWhenActionServerIsUnavailable)
{
  auto options = default_options();

  options.server_wait_timeout_sec = 0.12;

  RuntimeHarness harness(
    false,
    FakeRotateToHeadingServer::Mode::kSucceed,
    options);

  const auto adapter = harness.adapter();

  ASSERT_TRUE(
    adapter->request_rotation(0.5, 1.0));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return snapshot.terminal;
      }));

  const auto snapshot = adapter->snapshot();

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kTimedOut);

  EXPECT_EQ(
    snapshot.reason,
    "scan360_action_server_unavailable");

  EXPECT_FALSE(snapshot.active);
}


TEST_F(
  Scan360RotateActionClientRuntimeTest,
  ReportsRejectedGoal)
{
  RuntimeHarness harness(
    true,
    FakeRotateToHeadingServer::Mode::kRejectGoal,
    default_options());

  const auto adapter = harness.adapter();

  ASSERT_TRUE(
    adapter->request_rotation(0.75, 1.0));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return snapshot.terminal;
      }));

  const auto snapshot = adapter->snapshot();

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kRejected);

  EXPECT_EQ(
    snapshot.reason,
    "scan360_goal_rejected");

  ASSERT_TRUE(harness.server());

  EXPECT_EQ(
    harness.server()->goal_count(),
    1);
}


TEST_F(
  Scan360RotateActionClientRuntimeTest,
  PropagatesFeedbackAndSuccessfulResult)
{
  RuntimeHarness harness(
    true,
    FakeRotateToHeadingServer::Mode::kSucceed,
    default_options());

  const auto adapter = harness.adapter();

  ASSERT_TRUE(
    adapter->request_rotation(
      3.0 * kPi,
      1.0));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return snapshot.terminal;
      }));

  const auto snapshot = adapter->snapshot();

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kSucceeded);

  EXPECT_EQ(snapshot.reason, "goal_reached");
  EXPECT_TRUE(snapshot.terminal);
  EXPECT_FALSE(snapshot.active);

  EXPECT_NEAR(
    std::abs(snapshot.target_yaw_rad),
    kPi,
    1.0e-9);

  EXPECT_NEAR(
    snapshot.current_yaw_rad,
    snapshot.target_yaw_rad,
    1.0e-9);

  EXPECT_NEAR(snapshot.error_rad, 0.0, 1.0e-9);

  EXPECT_NEAR(
    snapshot.commanded_wz_rad_s,
    0.25,
    1.0e-9);

  ASSERT_TRUE(harness.server());

  EXPECT_EQ(
    harness.server()->goal_count(),
    1);

  EXPECT_EQ(
    harness.server()->feedback_count(),
    1);

  EXPECT_NEAR(
    std::abs(
      harness.server()->last_target_yaw()),
    kPi,
    1.0e-9);
}


TEST_F(
  Scan360RotateActionClientRuntimeTest,
  EnforcesSingleOwnerAndAcknowledgesCancel)
{
  std::atomic<bool> saw_cancel_ack{false};

  auto options = default_options();

  options.feedback_stale_timeout_sec = 2.0;

  RuntimeHarness harness(
    true,
    FakeRotateToHeadingServer::
    Mode::kHoldUntilCancel,
    options);

  const auto adapter = harness.adapter();

  adapter->set_update_callback(
    [&saw_cancel_ack](
      const ClientAdapter::Snapshot & snapshot)
    {
      if (snapshot.cancel_acknowledged) {
        saw_cancel_ack.store(true);
      }
    });

  ASSERT_TRUE(
    adapter->request_rotation(1.0, 2.0));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return
          snapshot.state ==
          ClientAdapter::State::kActive;
      }));

  EXPECT_FALSE(
    adapter->request_rotation(-1.0, 2.0));

  ASSERT_TRUE(
    adapter->request_cancel(
      "operator_cancel"));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return snapshot.terminal;
      }));

  adapter->set_update_callback({});

  const auto snapshot = adapter->snapshot();

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kCanceled);

  EXPECT_EQ(snapshot.reason, "canceled");
  EXPECT_TRUE(saw_cancel_ack.load());

  ASSERT_TRUE(harness.server());

  EXPECT_EQ(
    harness.server()->goal_count(),
    1);

  EXPECT_GE(
    harness.server()->cancel_count(),
    1);
}


TEST_F(
  Scan360RotateActionClientRuntimeTest,
  ExecutionTimeoutUsesNativeCancellation)
{
  auto options = default_options();

  options.feedback_stale_timeout_sec = 2.0;
  options.cancel_timeout_sec = 0.5;
  options.execution_grace_sec = 0.0;

  RuntimeHarness harness(
    true,
    FakeRotateToHeadingServer::
    Mode::kHoldUntilCancel,
    options);

  const auto adapter = harness.adapter();

  ASSERT_TRUE(
    adapter->request_rotation(1.2, 0.12));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return snapshot.terminal;
      }));

  const auto snapshot = adapter->snapshot();

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kTimedOut);

  EXPECT_EQ(
    snapshot.reason,
    "scan360_rotation_timeout");

  ASSERT_TRUE(harness.server());

  EXPECT_GE(
    harness.server()->cancel_count(),
    1);
}


TEST_F(
  Scan360RotateActionClientRuntimeTest,
  FeedbackStaleUsesNativeCancellation)
{
  auto options = default_options();

  options.feedback_stale_timeout_sec = 0.12;
  options.cancel_timeout_sec = 0.5;

  RuntimeHarness harness(
    true,
    FakeRotateToHeadingServer::
    Mode::kHoldWithoutFeedback,
    options);

  const auto adapter = harness.adapter();

  ASSERT_TRUE(
    adapter->request_rotation(1.4, 2.0));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return snapshot.terminal;
      }));

  const auto snapshot = adapter->snapshot();

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kFailed);

  EXPECT_EQ(
    snapshot.reason,
    "scan360_feedback_stale");

  ASSERT_TRUE(harness.server());

  EXPECT_GE(
    harness.server()->cancel_count(),
    1);
}


TEST_F(
  Scan360RotateActionClientRuntimeTest,
  RejectedCancelEndsInCancelTimeout)
{
  std::atomic<bool> saw_cancel_rejected{false};

  auto options = default_options();

  options.feedback_stale_timeout_sec = 2.0;
  options.cancel_timeout_sec = 0.15;

  RuntimeHarness harness(
    true,
    FakeRotateToHeadingServer::Mode::kRejectCancel,
    options);

  const auto adapter = harness.adapter();

  adapter->set_update_callback(
    [&saw_cancel_rejected](
      const ClientAdapter::Snapshot & snapshot)
    {
      if (
        snapshot.reason ==
        "scan360_cancel_rejected")
      {
        saw_cancel_rejected.store(true);
      }
    });

  ASSERT_TRUE(
    adapter->request_rotation(1.6, 2.0));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return
          snapshot.state ==
          ClientAdapter::State::kActive;
      }));

  ASSERT_TRUE(adapter->request_cancel());

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return snapshot.terminal;
      }));

  adapter->set_update_callback({});

  const auto snapshot = adapter->snapshot();

  EXPECT_TRUE(saw_cancel_rejected.load());

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kTimedOut);

  EXPECT_EQ(
    snapshot.reason,
    "scan360_cancel_timeout");

  ASSERT_TRUE(harness.server());

  EXPECT_GE(
    harness.server()->cancel_count(),
    1);
}


TEST_F(
  Scan360RotateActionClientRuntimeTest,
  PropagatesAbortedActionReason)
{
  RuntimeHarness harness(
    true,
    FakeRotateToHeadingServer::Mode::kAbort,
    default_options());

  const auto adapter = harness.adapter();

  ASSERT_TRUE(
    adapter->request_rotation(1.8, 1.0));

  ASSERT_TRUE(
    wait_until(
      adapter,
      [](const ClientAdapter::Snapshot & snapshot)
      {
        return snapshot.terminal;
      }));

  const auto snapshot = adapter->snapshot();

  EXPECT_EQ(
    snapshot.state,
    ClientAdapter::State::kAborted);

  EXPECT_EQ(snapshot.reason, "safety_stop");
  EXPECT_FALSE(snapshot.active);
  EXPECT_TRUE(snapshot.terminal);
}

}  // namespace
