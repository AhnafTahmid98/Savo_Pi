// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "gtest/gtest.h"

#include "savo_nav/coverage_execution_model.hpp"

namespace
{

using Backend = savo_nav::CoverageBackendTerminal;
using Code = savo_nav::CoverageTerminalCode;
using Event = savo_nav::CoverageWatchdogEvent;
using State = savo_nav::CoverageExecutionState;

TEST(CoverageExecutionModelTest, ResolvesDefaultTimeout)
{
  savo_nav::CoverageExecutionModel model;

  ASSERT_TRUE(model.Start(0.0, 0.0));

  EXPECT_DOUBLE_EQ(
    model.Snapshot().resolved_execution_timeout_seconds,
    300.0);
}

TEST(CoverageExecutionModelTest, CapsRequestedTimeout)
{
  savo_nav::CoverageExecutionModel model;

  ASSERT_TRUE(model.Start(0.0, 5000.0));

  EXPECT_DOUBLE_EQ(
    model.Snapshot().resolved_execution_timeout_seconds,
    3600.0);
}

TEST(CoverageExecutionModelTest, MarksBackendExecuting)
{
  savo_nav::CoverageExecutionModel model;

  ASSERT_TRUE(model.Start(0.0, 100.0));
  ASSERT_TRUE(model.MarkBackendAccepted(1.0));
  ASSERT_TRUE(model.MarkFeedback(2.0));

  EXPECT_EQ(model.Snapshot().state, State::kExecuting);
}

TEST(CoverageExecutionModelTest, ExecutionTimeoutRequestsCancel)
{
  savo_nav::CoverageExecutionPolicy policy;
  policy.default_execution_timeout_seconds = 2.0;
  policy.maximum_execution_timeout_seconds = 10.0;

  savo_nav::CoverageExecutionModel model(policy);

  ASSERT_TRUE(model.Start(0.0, 0.0));
  ASSERT_TRUE(model.MarkBackendAccepted(0.0));

  EXPECT_EQ(
    model.CheckWatchdogs(3.0),
    Event::kExecutionTimeout);

  EXPECT_EQ(model.Snapshot().state, State::kCanceling);
}

TEST(CoverageExecutionModelTest, FeedbackStaleRequestsCancel)
{
  savo_nav::CoverageExecutionPolicy policy;
  policy.feedback_stale_timeout_seconds = 2.0;

  savo_nav::CoverageExecutionModel model(policy);

  ASSERT_TRUE(model.Start(0.0, 100.0));
  ASSERT_TRUE(model.MarkBackendAccepted(0.0));

  EXPECT_EQ(
    model.CheckWatchdogs(3.0),
    Event::kFeedbackStale);
}

TEST(CoverageExecutionModelTest, ExternalCancelAcknowledges)
{
  savo_nav::CoverageExecutionModel model;

  ASSERT_TRUE(model.Start(0.0, 100.0));
  ASSERT_TRUE(model.MarkBackendAccepted(1.0));

  ASSERT_TRUE(
    model.RequestCancel(
      2.0,
      "external_cancel"));

  ASSERT_TRUE(
    model.ObserveBackendTerminal(
      Backend::kCanceled,
      "backend_canceled"));

  EXPECT_EQ(model.Snapshot().state, State::kCanceled);
  EXPECT_EQ(model.Snapshot().terminal_code, Code::kCanceled);
  EXPECT_TRUE(model.Snapshot().release_ownership);
}

TEST(CoverageExecutionModelTest, TimeoutCancelMapsToTimedOut)
{
  savo_nav::CoverageExecutionPolicy policy;
  policy.default_execution_timeout_seconds = 1.0;
  policy.maximum_execution_timeout_seconds = 10.0;

  savo_nav::CoverageExecutionModel model(policy);

  ASSERT_TRUE(model.Start(0.0, 0.0));
  ASSERT_TRUE(model.MarkBackendAccepted(0.0));

  ASSERT_EQ(
    model.CheckWatchdogs(2.0),
    Event::kExecutionTimeout);

  ASSERT_TRUE(
    model.ObserveBackendTerminal(
      Backend::kCanceled,
      "backend_canceled"));

  EXPECT_EQ(model.Snapshot().state, State::kTimedOut);
  EXPECT_EQ(model.Snapshot().terminal_code, Code::kTimedOut);
}

TEST(CoverageExecutionModelTest, CancelTimeoutQuarantinesOwnership)
{
  savo_nav::CoverageExecutionPolicy policy;
  policy.cancel_timeout_seconds = 1.0;

  savo_nav::CoverageExecutionModel model(policy);

  ASSERT_TRUE(model.Start(0.0, 100.0));
  ASSERT_TRUE(model.MarkBackendAccepted(0.0));

  ASSERT_TRUE(
    model.RequestCancel(
      1.0,
      "external_cancel"));

  ASSERT_EQ(
    model.CheckWatchdogs(3.0),
    Event::kCancelTimeout);

  EXPECT_TRUE(model.Snapshot().public_terminal);
  EXPECT_FALSE(model.Snapshot().release_ownership);
  EXPECT_TRUE(model.Snapshot().backend_terminal_pending);

  ASSERT_TRUE(
    model.ObserveBackendTerminal(
      Backend::kCanceled,
      "late_cancel_ack"));

  EXPECT_TRUE(model.Snapshot().release_ownership);
}

TEST(CoverageExecutionModelTest, LateSuccessAfterCancelWins)
{
  savo_nav::CoverageExecutionModel model;

  ASSERT_TRUE(model.Start(0.0, 100.0));
  ASSERT_TRUE(model.MarkBackendAccepted(1.0));

  ASSERT_TRUE(
    model.RequestCancel(
      2.0,
      "external_cancel"));

  ASSERT_TRUE(
    model.ObserveBackendTerminal(
      Backend::kSucceeded,
      "late_success"));

  EXPECT_EQ(model.Snapshot().state, State::kSucceeded);
  EXPECT_EQ(model.Snapshot().terminal_code, Code::kSucceeded);
}

TEST(CoverageExecutionModelTest, FailsBeforeBackend)
{
  savo_nav::CoverageExecutionModel model;

  ASSERT_TRUE(model.Start(0.0, 100.0));

  ASSERT_TRUE(
    model.FailBeforeBackend(
      Code::kBackendUnavailable,
      "backend_unavailable"));

  EXPECT_EQ(model.Snapshot().state, State::kFailed);
  EXPECT_TRUE(model.Snapshot().release_ownership);
}

TEST(CoverageExecutionModelTest, ResetReturnsIdle)
{
  savo_nav::CoverageExecutionModel model;

  ASSERT_TRUE(model.Start(0.0, 100.0));
  model.Reset();

  EXPECT_EQ(model.Snapshot().state, State::kIdle);
}

}  // namespace
