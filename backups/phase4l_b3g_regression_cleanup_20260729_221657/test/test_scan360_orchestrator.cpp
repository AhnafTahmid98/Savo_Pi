#include "savo_mapping/scan360_orchestrator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{

namespace scan360 =
  savo_mapping::scan360;

class RotationClientFixture
{
public:
  scan360::Scan360RotationCallbacks callbacks()
  {
    return {
      [this](
        const double target_yaw_rad,
        const double duration_sec)
      {
        ++rotation_request_count;
        requested_yaw_rad = target_yaw_rad;
        requested_duration_sec = duration_sec;
        return rotation_request_result;
      },
      [this]()
      {
        ++cancel_request_count;
        return cancel_request_result;
      },
      [this]()
      {
        ++tick_count;
      },
      [this]()
      {
        return snapshot;
      }};
  }

  scan360::RotationClientSnapshot snapshot;

  bool rotation_request_result{true};
  bool cancel_request_result{true};

  int rotation_request_count{0};
  int cancel_request_count{0};
  int tick_count{0};

  double requested_yaw_rad{0.0};
  double requested_duration_sec{0.0};
};

scan360::ControllerDecision rotation_decision(
  const double target_yaw_rad)
{
  scan360::ControllerDecision decision;

  decision.action =
    scan360::ControllerAction::
    IssueRotationRequest;

  scan360::Scan360Target target;
  target.normalized_yaw_rad = target_yaw_rad;
  decision.target = target;

  return decision;
}

scan360::ControllerDecision cancel_decision()
{
  scan360::ControllerDecision decision;

  decision.action =
    scan360::ControllerAction::RequestCancel;

  return decision;
}

void construct_with_incomplete_callbacks()
{
  scan360::Scan360Orchestrator orchestrator(
    scan360::Scan360RotationCallbacks{},
    10.0);

  (void)orchestrator;
}

void construct_with_duration(
  const double duration_sec)
{
  RotationClientFixture fixture;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    duration_sec);

  (void)orchestrator;
}

}  // namespace

TEST(
  Scan360OrchestratorContract,
  IncompleteCallbacksAreRejected)
{
  EXPECT_THROW(
    construct_with_incomplete_callbacks(),
    std::invalid_argument);
}

TEST(
  Scan360OrchestratorContract,
  InvalidDurationIsRejected)
{
  EXPECT_THROW(
    construct_with_duration(0.0),
    std::invalid_argument);

  EXPECT_THROW(
    construct_with_duration(
      std::numeric_limits<double>::
      infinity()),
    std::invalid_argument);
}

TEST(
  Scan360OrchestratorContract,
  MissingTargetProducesMotionRejected)
{
  RotationClientFixture fixture;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  scan360::ControllerDecision decision;

  decision.action =
    scan360::ControllerAction::
    IssueRotationRequest;

  orchestrator.dispatch(decision);

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    MotionRejected});

  EXPECT_EQ(
    orchestrator.last_reason(),
    "scan360_target_missing");

  EXPECT_EQ(fixture.rotation_request_count, 0);
}

TEST(
  Scan360OrchestratorContract,
  NonFiniteTargetProducesMotionRejected)
{
  RotationClientFixture fixture;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(
    rotation_decision(
      std::numeric_limits<double>::
      quiet_NaN()));

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    MotionRejected});

  EXPECT_EQ(
    orchestrator.last_reason(),
    "scan360_target_yaw_not_finite");

  EXPECT_EQ(fixture.rotation_request_count, 0);
}

TEST(
  Scan360OrchestratorContract,
  RotationRequestForwardsYawAndDuration)
{
  RotationClientFixture fixture;

  fixture.snapshot.state =
    scan360::RotationClientState::Pending;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    7.5);

  orchestrator.dispatch(
    rotation_decision(-1.25));

  EXPECT_EQ(fixture.rotation_request_count, 1);
  EXPECT_DOUBLE_EQ(fixture.requested_yaw_rad, -1.25);
  EXPECT_DOUBLE_EQ(fixture.requested_duration_sec, 7.5);
  EXPECT_TRUE(orchestrator.take_events().empty());
}

TEST(
  Scan360OrchestratorContract,
  ActiveStateProducesMotionAcceptedExactlyOnce)
{
  RotationClientFixture fixture;

  fixture.snapshot.state =
    scan360::RotationClientState::Pending;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(rotation_decision(0.5));

  fixture.snapshot.state =
    scan360::RotationClientState::Active;

  orchestrator.tick();
  orchestrator.tick();

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    MotionAccepted});

  EXPECT_EQ(fixture.tick_count, 2);
}

TEST(
  Scan360OrchestratorContract,
  ImmediateSuccessPreservesControllerEventOrder)
{
  RotationClientFixture fixture;

  fixture.snapshot.state =
    scan360::RotationClientState::Succeeded;

  fixture.snapshot.reason = "goal_reached";

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(rotation_decision(1.0));

  EXPECT_EQ(
    orchestrator.take_events(),
    (std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    MotionAccepted,
    scan360::ControllerEvent::
    TargetReached}));

  EXPECT_EQ(
    orchestrator.last_reason(),
    "goal_reached");
}

TEST(
  Scan360OrchestratorContract,
  SuccessAfterActiveProducesTargetReachedOnce)
{
  RotationClientFixture fixture;

  fixture.snapshot.state =
    scan360::RotationClientState::Active;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(rotation_decision(1.0));

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    MotionAccepted});

  fixture.snapshot.state =
    scan360::RotationClientState::Succeeded;

  fixture.snapshot.reason = "settled";

  orchestrator.tick();
  orchestrator.tick();

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    TargetReached});

  EXPECT_EQ(orchestrator.last_reason(), "settled");
}

TEST(
  Scan360OrchestratorContract,
  CancelSuccessProducesCancelAcknowledged)
{
  RotationClientFixture fixture;

  fixture.snapshot.state =
    scan360::RotationClientState::Canceled;

  fixture.snapshot.reason = "operator_cancel";

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(cancel_decision());

  EXPECT_EQ(fixture.cancel_request_count, 1);

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    CancelAcknowledged});

  EXPECT_EQ(
    orchestrator.last_reason(),
    "operator_cancel");
}

TEST(
  Scan360OrchestratorContract,
  SynchronousRequestRejectionProducesMotionRejected)
{
  RotationClientFixture fixture;

  fixture.rotation_request_result = false;

  fixture.snapshot.state =
    scan360::RotationClientState::Rejected;

  fixture.snapshot.reason = "server_busy";

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(rotation_decision(0.75));

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    MotionRejected});

  EXPECT_EQ(orchestrator.last_reason(), "server_busy");
}

TEST(
  Scan360OrchestratorContract,
  RuntimeFailureProducesMotionFailed)
{
  RotationClientFixture fixture;

  fixture.snapshot.state =
    scan360::RotationClientState::Pending;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(rotation_decision(0.75));

  fixture.snapshot.state =
    scan360::RotationClientState::Failed;

  fixture.snapshot.reason = "transport_lost";

  orchestrator.tick();
  orchestrator.tick();

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    MotionFailed});

  EXPECT_EQ(
    orchestrator.last_reason(),
    "transport_lost");
}

TEST(
  Scan360OrchestratorContract,
  CancelFailureProducesCancelRejected)
{
  RotationClientFixture fixture;

  fixture.snapshot.state =
    scan360::RotationClientState::Canceling;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(cancel_decision());

  fixture.snapshot.state =
    scan360::RotationClientState::Failed;

  fixture.snapshot.reason = "cancel_timeout";

  orchestrator.tick();
  orchestrator.tick();

  EXPECT_EQ(
    orchestrator.take_events(),
    std::vector<scan360::ControllerEvent>{
    scan360::ControllerEvent::
    CancelRejected});

  EXPECT_EQ(
    orchestrator.last_reason(),
    "cancel_timeout");
}

TEST(
  Scan360OrchestratorContract,
  IdleWithoutPendingOperationEmitsNothing)
{
  RotationClientFixture fixture;

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.tick();

  EXPECT_TRUE(orchestrator.take_events().empty());
  EXPECT_EQ(orchestrator.last_reason(), "idle");
}

TEST(
  Scan360OrchestratorContract,
  ResetClearsQueuedAndTerminalState)
{
  RotationClientFixture fixture;

  fixture.snapshot.state =
    scan360::RotationClientState::Succeeded;

  fixture.snapshot.reason = "goal_reached";

  scan360::Scan360Orchestrator orchestrator(
    fixture.callbacks(),
    10.0);

  orchestrator.dispatch(rotation_decision(1.0));
  orchestrator.reset();

  EXPECT_TRUE(orchestrator.take_events().empty());
  EXPECT_EQ(orchestrator.last_reason(), "idle");

  orchestrator.tick();

  EXPECT_TRUE(orchestrator.take_events().empty());
}
