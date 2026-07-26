#include "savo_mapping/scan360_controller.hpp"
#include "savo_mapping/scan360_planner.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace
{

namespace scan360 =
  savo_mapping::scan360;

scan360::Scan360Plan make_test_plan()
{
  scan360::Scan360PlanOptions options;

  options.step_angle_rad =
    scan360::kPi;

  const auto result =
    scan360::make_plan(
    0.0,
    options);

  if (!result.accepted) {
    throw std::runtime_error(
            "test plan was rejected");
  }

  return result.plan;
}

}  // namespace

TEST(
  Scan360ControllerContract,
  StateAndActionStringsAreStable)
{
  EXPECT_EQ(
    std::string{
    scan360::to_string(
        scan360::ControllerState::
      Rotating)},
    "rotating");

  EXPECT_EQ(
    std::string{
    scan360::to_string(
        scan360::ControllerAction::
      IssueRotationRequest)},
    "issue_rotation_request");
}

TEST(
  Scan360ControllerContract,
  RunsPlanSequentiallyToCompletion)
{
  scan360::Scan360Controller controller;

  auto decision =
    controller.load_plan(
    make_test_plan());

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Ready);

  decision =
    controller.handle(
    scan360::ControllerEvent::Start);

  ASSERT_TRUE(decision.target.has_value());

  EXPECT_EQ(
    decision.action,
    scan360::ControllerAction::
    IssueRotationRequest);

  EXPECT_EQ(
    decision.target->index,
    0U);

  decision =
    controller.handle(
    scan360::ControllerEvent::
    MotionAccepted);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Rotating);

  decision =
    controller.handle(
    scan360::ControllerEvent::
    TargetReached);

  EXPECT_EQ(
    decision.action,
    scan360::ControllerAction::
    StartSettleTimer);

  decision =
    controller.handle(
    scan360::ControllerEvent::
    SettleComplete);

  ASSERT_TRUE(decision.target.has_value());

  EXPECT_EQ(
    decision.action,
    scan360::ControllerAction::
    IssueRotationRequest);

  EXPECT_EQ(
    decision.target->index,
    1U);

  controller.handle(
    scan360::ControllerEvent::
    MotionAccepted);

  controller.handle(
    scan360::ControllerEvent::
    TargetReached);

  decision =
    controller.handle(
    scan360::ControllerEvent::
    SettleComplete);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Complete);

  EXPECT_EQ(
    decision.action,
    scan360::ControllerAction::
    ScanComplete);

  EXPECT_FALSE(
    decision.target.has_value());
}

TEST(
  Scan360ControllerContract,
  SafetyStopCancelsAndResumesSameTarget)
{
  scan360::Scan360Controller controller;

  controller.load_plan(
    make_test_plan());

  auto decision =
    controller.handle(
    scan360::ControllerEvent::Start);

  ASSERT_TRUE(decision.target.has_value());
  EXPECT_EQ(decision.target->index, 0U);

  controller.handle(
    scan360::ControllerEvent::
    MotionAccepted);

  decision =
    controller.handle(
    scan360::ControllerEvent::
    SafetyStop);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Canceling);

  EXPECT_EQ(
    decision.action,
    scan360::ControllerAction::
    RequestCancel);

  decision =
    controller.handle(
    scan360::ControllerEvent::
    CancelAcknowledged);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Paused);

  decision =
    controller.handle(
    scan360::ControllerEvent::Start);

  ASSERT_TRUE(decision.target.has_value());

  EXPECT_EQ(
    decision.target->index,
    0U);

  EXPECT_EQ(
    decision.action,
    scan360::ControllerAction::
    IssueRotationRequest);
}

TEST(
  Scan360ControllerContract,
  OperatorCancelTerminatesAfterAcknowledgement)
{
  scan360::Scan360Controller controller;

  controller.load_plan(
    make_test_plan());

  controller.handle(
    scan360::ControllerEvent::Start);

  auto decision =
    controller.handle(
    scan360::ControllerEvent::
    OperatorCancel);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Canceling);

  EXPECT_EQ(
    decision.action,
    scan360::ControllerAction::
    RequestCancel);

  decision =
    controller.handle(
    scan360::ControllerEvent::
    CancelAcknowledged);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Canceled);
}

TEST(
  Scan360ControllerContract,
  AuthorityLossWhileSettlingPausesSafely)
{
  scan360::Scan360Controller controller;

  controller.load_plan(
    make_test_plan());

  controller.handle(
    scan360::ControllerEvent::Start);

  controller.handle(
    scan360::ControllerEvent::
    TargetReached);

  const auto decision =
    controller.handle(
    scan360::ControllerEvent::
    AuthorityLost);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Paused);

  EXPECT_EQ(
    decision.action,
    scan360::ControllerAction::None);
}

TEST(
  Scan360ControllerContract,
  MotionAndCancellationFailuresAreTerminal)
{
  scan360::Scan360Controller controller;

  controller.load_plan(
    make_test_plan());

  controller.handle(
    scan360::ControllerEvent::Start);

  auto decision =
    controller.handle(
    scan360::ControllerEvent::
    MotionRejected);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Failed);

  controller.load_plan(
    make_test_plan());

  controller.handle(
    scan360::ControllerEvent::Start);

  controller.handle(
    scan360::ControllerEvent::
    SafetyStop);

  decision =
    controller.handle(
    scan360::ControllerEvent::
    CancelRejected);

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Failed);
}

TEST(
  Scan360ControllerContract,
  EmptyPlanIsRejected)
{
  scan360::Scan360Controller controller;

  const auto decision =
    controller.load_plan(
    scan360::Scan360Plan{});

  EXPECT_EQ(
    decision.state,
    scan360::ControllerState::Failed);

  EXPECT_EQ(
    decision.reason,
    "invalid_plan");

  EXPECT_FALSE(controller.has_plan());
}
