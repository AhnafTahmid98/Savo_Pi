// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <nav2_msgs/action/follow_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <savo_msgs/action/execute_coverage_path.hpp>

#include "savo_nav/coverage_action_adapter.hpp"
#include "savo_nav/coverage_execution_model.hpp"
#include "savo_nav/coverage_progress_tracker.hpp"
#include "savo_nav/goal_gateway.hpp"
#include "savo_nav/map_context.hpp"
#include "savo_nav/navigation_readiness.hpp"

namespace savo_nav
{

class CoverageFollowPathGateway
{
public:
  using Action =
    savo_msgs::action::ExecuteCoveragePath;

  using FollowPath =
    nav2_msgs::action::FollowPath;

  using ExternalGoalHandle =
    rclcpp_action::ServerGoalHandle<Action>;

  using InternalGoalHandle =
    rclcpp_action::ClientGoalHandle<FollowPath>;

  using PublishCallback =
    std::function<void(const std::string &)>;

  CoverageFollowPathGateway(
    rclcpp::Node & node,
    std::mutex & shared_mutex,
    GoalGateway & gateway,
    MapContext & map_context,
    NavigationReadinessResult & readiness,
    std::string coverage_action,
    std::string follow_path_action,
    std::string controller_id,
    std::string goal_checker_id,
    std::string progress_checker_id,
    CoverageActionAdapterPolicy adapter_policy,
    CoverageExecutionPolicy execution_policy,
    PublishCallback publish);

  [[nodiscard]] bool Active() const;

  void CheckWatchdogs();

private:
  [[nodiscard]] rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    const std::shared_ptr<const Action::Goal> & goal);

  [[nodiscard]] rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<ExternalGoalHandle> & goal_handle);

  void HandleAccepted(
    const std::shared_ptr<ExternalGoalHandle> & goal_handle);

  void OnBackendGoalResponse(
    const std::string & goal_id,
    std::uint64_t sequence,
    const InternalGoalHandle::SharedPtr & internal_handle);

  void OnBackendFeedback(
    const std::string & goal_id,
    std::uint64_t sequence,
    const std::shared_ptr<const FollowPath::Feedback> & feedback);

  void OnBackendResult(
    const std::string & goal_id,
    std::uint64_t sequence,
    const InternalGoalHandle::WrappedResult & result);

  void ForwardBackendCancel(
    const InternalGoalHandle::SharedPtr & internal_handle);

  void PublishFeedbackLocked();

  void ClearLocked();

  [[nodiscard]] bool MatchesLocked(
    const std::string & goal_id,
    std::uint64_t sequence) const;

  [[nodiscard]] static double MonotonicSeconds();

  [[nodiscard]] static CoverageBackendTerminal
  BackendTerminalFrom(
    rclcpp_action::ResultCode code);

  [[nodiscard]] static std::string BackendReason(
    rclcpp_action::ResultCode code,
    const std::shared_ptr<FollowPath::Result> & result);

  rclcpp::Node & node_;
  std::mutex & mutex_;

  GoalGateway & gateway_;
  MapContext & map_context_;
  NavigationReadinessResult & readiness_;

  std::string controller_id_;
  std::string goal_checker_id_;
  std::string progress_checker_id_;

  CoverageActionAdapterPolicy adapter_policy_;
  CoverageExecutionModel execution_model_;
  CoverageProgressTracker progress_tracker_;

  PublishCallback publish_;

  bool active_{false};
  bool cancel_forwarded_{false};
  bool public_terminal_sent_{false};

  std::uint64_t sequence_counter_{0};

  std::optional<GoalContext> active_context_;

  std::shared_ptr<ExternalGoalHandle>
  external_goal_handle_;

  InternalGoalHandle::SharedPtr
    internal_goal_handle_;

  rclcpp_action::Client<FollowPath>::SharedPtr
    follow_path_client_;

  rclcpp_action::Server<Action>::SharedPtr
    coverage_server_;
};

}  // namespace savo_nav
