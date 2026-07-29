// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <savo_msgs/action/execute_coverage_path.hpp>

#include "savo_nav/goal_admission_policy.hpp"

namespace savo_nav
{

class CoverageAdmissionProxy
{
public:
  using Action =
    savo_msgs::action::ExecuteCoveragePath;

  using ExternalGoalHandle =
    rclcpp_action::ServerGoalHandle<Action>;

  using InternalGoalHandle =
    rclcpp_action::ClientGoalHandle<Action>;

  using EvaluateCallback =
    std::function<GoalAdmissionDecision()>;

  using PublishCallback =
    std::function<void(
        const std::string &,
        const std::string &)>;

  CoverageAdmissionProxy(
    rclcpp::Node & node,
    std::mutex & shared_mutex,
    bool & slot_reserved,
    bool & active_goal,
    bool & cancellation_requested,
    bool & internal_cancel_sent,
    std::string & cancellation_reason,
    std::uint64_t & generation_counter,
    double internal_server_timeout_seconds,
    std::string public_action,
    std::string internal_action,
    EvaluateCallback evaluate,
    PublishCallback publish);

  [[nodiscard]] bool Active() const;

  [[nodiscard]] bool ActiveLocked() const noexcept;

  void RequestCancel(std::string reason);

private:
  [[nodiscard]] rclcpp_action::GoalResponse HandleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Action::Goal> goal);

  [[nodiscard]] rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<ExternalGoalHandle> & goal_handle);

  void HandleAccepted(
    const std::shared_ptr<ExternalGoalHandle> & goal_handle);

  void ForwardGoal(
    std::uint64_t generation,
    const std::shared_ptr<ExternalGoalHandle> & external_handle);

  void OnInternalGoalResponse(
    std::uint64_t generation,
    const InternalGoalHandle::SharedPtr & internal_handle);

  void OnInternalFeedback(
    std::uint64_t generation,
    const std::shared_ptr<const Action::Feedback> & feedback);

  void OnInternalResult(
    std::uint64_t generation,
    const InternalGoalHandle::WrappedResult & wrapped);

  void RequestInternalCancel(std::uint64_t generation);

  void FinishLocally(
    std::uint64_t generation,
    const std::string & reason,
    bool canceled);

  void ReleaseLocked();

  [[nodiscard]] static std::shared_ptr<Action::Result>
  MakeErrorResult(
    const std::string & reason,
    std::uint8_t result_code);

  rclcpp::Node & node_;
  std::mutex & mutex_;

  bool & slot_reserved_;
  bool & active_goal_;
  bool & cancellation_requested_;
  bool & internal_cancel_sent_;

  std::string & cancellation_reason_;
  std::uint64_t & generation_counter_;

  double internal_server_timeout_seconds_{1.0};

  EvaluateCallback evaluate_;
  PublishCallback publish_;

  bool active_{false};
  std::uint64_t generation_{0};

  std::shared_ptr<ExternalGoalHandle>
  external_goal_handle_;

  InternalGoalHandle::SharedPtr
    internal_goal_handle_;

  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};

}  // namespace savo_nav
