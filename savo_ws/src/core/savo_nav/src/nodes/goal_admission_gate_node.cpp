// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <action_msgs/srv/cancel_goal.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "savo_nav/action_names.hpp"
#include "savo_nav/coverage_admission_proxy.hpp"
#include "savo_nav/goal_admission_policy.hpp"
#include "savo_nav/topic_names.hpp"

namespace
{

class GoalAdmissionGateNode final : public rclcpp::Node
{
public:
  using NavigateToPose =
    nav2_msgs::action::NavigateToPose;

  using ExternalGoalHandle =
    rclcpp_action::ServerGoalHandle<NavigateToPose>;

  using InternalGoalHandle =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

  using ActionClient =
    rclcpp_action::Client<NavigateToPose>;

  using CancelResponse =
    action_msgs::srv::CancelGoal::Response;

  enum class GoalSource : std::uint8_t
  {
    kNavigation = 0,
    kExploration
  };

  GoalAdmissionGateNode()
  : Node("goal_admission_gate_node")
  {
    publish_hz_ = declare_parameter<double>(
      "publish_hz",
      20.0);

    guard_timeout_seconds_ =
      declare_parameter<double>(
      "guard_timeout_seconds",
      0.75);

    internal_server_timeout_seconds_ =
      declare_parameter<double>(
      "internal_server_timeout_seconds",
      1.0);

    ValidatePositive(
      publish_hz_,
      "publish_hz");

    ValidatePositive(
      guard_timeout_seconds_,
      "guard_timeout_seconds");

    ValidatePositive(
      internal_server_timeout_seconds_,
      "internal_server_timeout_seconds");

    const std::string guard_allowed_topic =
      declare_parameter<std::string>(
      "control_recovery_allowed_topic",
      std::string(
        savo_nav::topics::kControlRecoveryAllowed));

    const std::string guard_reason_topic =
      declare_parameter<std::string>(
      "control_recovery_reason_topic",
      std::string(
        savo_nav::topics::kControlRecoveryReason));

    public_navigation_action_ =
      declare_parameter<std::string>(
      "public_navigation_action",
      std::string(
        savo_nav::actions::kNavigationNavigateToPose));

    public_exploration_action_ =
      declare_parameter<std::string>(
      "public_exploration_action",
      std::string(
        savo_nav::actions::kExplorationNavigateToPose));

    internal_navigation_action_ =
      declare_parameter<std::string>(
      "internal_navigation_action",
      std::string(
        savo_nav::actions::kInternalNavigationNavigateToPose));

    internal_exploration_action_ =
      declare_parameter<std::string>(
      "internal_exploration_action",
      std::string(
        savo_nav::actions::kInternalExplorationNavigateToPose));

    const std::string public_coverage_action =
      declare_parameter<std::string>(
      "public_coverage_action",
      std::string(
        savo_nav::actions::kCoverageExecutePath));

    const std::string internal_coverage_action =
      declare_parameter<std::string>(
      "internal_coverage_action",
      std::string(
        savo_nav::actions::kInternalCoverageExecutePath));

    const std::string state_topic =
      declare_parameter<std::string>(
      "state_topic",
      std::string(
        savo_nav::topics::kGoalAdmissionState));

    const std::string reason_topic =
      declare_parameter<std::string>(
      "reason_topic",
      std::string(
        savo_nav::topics::kGoalAdmissionReason));

    const std::string status_topic =
      declare_parameter<std::string>(
      "status_topic",
      std::string(
        savo_nav::topics::kGoalAdmissionStatus));

    auto retained_qos =
      rclcpp::QoS(rclcpp::KeepLast(1));

    retained_qos.reliable();
    retained_qos.transient_local();

    guard_allowed_subscription_ =
      create_subscription<std_msgs::msg::Bool>(
      guard_allowed_topic,
      retained_qos,
      [this](
        const std_msgs::msg::Bool::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);

        guard_observed_ = true;
        guard_allowed_ = message->data;
        guard_stamp_seconds_ = now().seconds();
      });

    guard_reason_subscription_ =
      create_subscription<std_msgs::msg::String>(
      guard_reason_topic,
      retained_qos,
      [this](
        const std_msgs::msg::String::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        guard_reason_ = message->data;

        if (
          active_goal_ &&
          cancellation_requested_ &&
          cancellation_reason_ != "external_cancel" &&
          !guard_allowed_ &&
          !message->data.empty())
        {
          cancellation_reason_ = message->data;
        }
      });

    state_publisher_ =
      create_publisher<std_msgs::msg::String>(
      state_topic,
      retained_qos);

    reason_publisher_ =
      create_publisher<std_msgs::msg::String>(
      reason_topic,
      retained_qos);

    status_publisher_ =
      create_publisher<std_msgs::msg::String>(
      status_topic,
      retained_qos);

    navigation_client_ =
      rclcpp_action::create_client<NavigateToPose>(
      this,
      internal_navigation_action_);

    exploration_client_ =
      rclcpp_action::create_client<NavigateToPose>(
      this,
      internal_exploration_action_);

    navigation_server_ =
      rclcpp_action::create_server<NavigateToPose>(
      this,
      public_navigation_action_,
      [this](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
      {
        return HandleGoal(
          GoalSource::kNavigation,
          uuid,
          std::move(goal));
      },
      [this](
        const std::shared_ptr<ExternalGoalHandle> goal_handle)
      {
        return HandleCancel(goal_handle);
      },
      [this](
        const std::shared_ptr<ExternalGoalHandle> goal_handle)
      {
        HandleAccepted(
          GoalSource::kNavigation,
          goal_handle);
      });

    exploration_server_ =
      rclcpp_action::create_server<NavigateToPose>(
      this,
      public_exploration_action_,
      [this](
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const NavigateToPose::Goal> goal)
      {
        return HandleGoal(
          GoalSource::kExploration,
          uuid,
          std::move(goal));
      },
      [this](
        const std::shared_ptr<ExternalGoalHandle> goal_handle)
      {
        return HandleCancel(goal_handle);
      },
      [this](
        const std::shared_ptr<ExternalGoalHandle> goal_handle)
      {
        HandleAccepted(
          GoalSource::kExploration,
          goal_handle);
      });

    coverage_proxy_ =
      std::make_unique<
      savo_nav::CoverageAdmissionProxy>(
      *this,
      mutex_,
      slot_reserved_,
      active_goal_,
      cancellation_requested_,
      internal_cancel_sent_,
      cancellation_reason_,
      active_generation_,
      internal_server_timeout_seconds_,
      public_coverage_action,
      internal_coverage_action,
      [this]()
      {
        return EvaluateLocked();
      },
      [this](
        const std::string & state,
        const std::string & reason)
      {
        PublishState(state, reason);
      });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(
        1.0 / publish_hz_),
      [this]()
      {
        OnTimer();
      });

    PublishState(
      "idle",
      "control_recovery_guard_unobserved");

    RCLCPP_INFO(
      get_logger(),
      "Goal admission gate started | public=%s,%s internal=%s,%s",
      public_navigation_action_.c_str(),
      public_exploration_action_.c_str(),
      internal_navigation_action_.c_str(),
      internal_exploration_action_.c_str());
  }

private:
  struct GuardSnapshot
  {
    bool observed{false};
    bool fresh{false};
    bool allowed{false};
    std::string reason{
      "control_recovery_guard_unobserved"};
  };

  static void ValidatePositive(
    const double value,
    const char * const name)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(
              std::string(name) +
              " must be finite and positive");
    }
  }

  static const char * SourceText(
    const GoalSource source) noexcept
  {
    return source == GoalSource::kNavigation ?
           "navigation" :
           "exploration";
  }

  GuardSnapshot GuardSnapshotLocked(
    const double now_seconds) const
  {
    GuardSnapshot snapshot;

    snapshot.observed = guard_observed_;

    if (!guard_observed_) {
      return snapshot;
    }

    if (
      !std::isfinite(now_seconds) ||
      now_seconds < guard_stamp_seconds_ ||
      (now_seconds - guard_stamp_seconds_) >
      guard_timeout_seconds_)
    {
      snapshot.reason =
        "control_recovery_guard_stale";

      return snapshot;
    }

    snapshot.fresh = true;
    snapshot.allowed = guard_allowed_;

    if (!guard_allowed_) {
      snapshot.reason =
        guard_reason_.empty() ||
        guard_reason_ == "control_recovery_ready" ?
        "control_recovery_guard_blocked" :
        guard_reason_;

      return snapshot;
    }

    snapshot.reason = "control_recovery_ready";
    return snapshot;
  }

  savo_nav::GoalAdmissionDecision EvaluateLocked() const
  {
    const GuardSnapshot guard =
      GuardSnapshotLocked(now().seconds());

    savo_nav::GoalAdmissionInput input;

    input.guard_observed = guard.observed;
    input.guard_fresh = guard.fresh;
    input.guard_allowed = guard.allowed;
    input.active_goal = active_goal_;
    input.slot_reserved = slot_reserved_;
    input.cancellation_requested =
      cancellation_requested_;
    input.guard_reason = guard.reason;

    return savo_nav::GoalAdmissionPolicy::Evaluate(
      input);
  }

  rclcpp_action::GoalResponse HandleGoal(
    const GoalSource source,
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal>)
  {
    savo_nav::GoalAdmissionDecision decision;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      decision = EvaluateLocked();

      if (decision.accept_new_goal) {
        slot_reserved_ = true;
      }
    }

    if (!decision.accept_new_goal) {
      PublishState("rejected", decision.reason);

      return rclcpp_action::GoalResponse::REJECT;
    }

    PublishState(
      "reserved",
      std::string("goal_reserved_") +
      SourceText(source));

    return
      rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse HandleCancel(
    const std::shared_ptr<ExternalGoalHandle> goal_handle)
  {
    std::uint64_t generation = 0;
    bool request_internal_cancel = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_goal_ ||
        external_goal_handle_ != goal_handle)
      {
        return rclcpp_action::CancelResponse::REJECT;
      }

      if (!cancellation_requested_) {
        cancellation_requested_ = true;
        cancellation_reason_ = "external_cancel";
      }

      generation = active_generation_;

      request_internal_cancel =
        internal_goal_handle_ != nullptr &&
        !internal_cancel_sent_;
    }

    PublishState("canceling", "external_cancel");

    if (request_internal_cancel) {
      RequestInternalCancel(generation);
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void HandleAccepted(
    const GoalSource source,
    const std::shared_ptr<ExternalGoalHandle> goal_handle)
  {
    std::uint64_t generation = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      slot_reserved_ = false;
      active_goal_ = true;
      active_source_ = source;
      external_goal_handle_ = goal_handle;
      internal_goal_handle_.reset();

      cancellation_requested_ = false;
      internal_cancel_sent_ = false;
      cancellation_reason_.clear();

      generation = ++active_generation_;
    }

    PublishState(
      "forwarding",
      std::string("forwarding_") +
      SourceText(source));

    std::thread(
      [this, generation, source, goal_handle]()
      {
        ForwardGoal(
          generation,
          source,
          goal_handle);
      }).detach();
  }

  ActionClient::SharedPtr ClientFor(
    const GoalSource source) const
  {
    return source == GoalSource::kNavigation ?
           navigation_client_ :
           exploration_client_;
  }

  void ForwardGoal(
    const std::uint64_t generation,
    const GoalSource source,
    const std::shared_ptr<ExternalGoalHandle> external_handle)
  {
    const ActionClient::SharedPtr client =
      ClientFor(source);

    if (!client->wait_for_action_server(
        std::chrono::duration<double>(
          internal_server_timeout_seconds_)))
    {
      FinishLocally(
        generation,
        "internal_goal_gateway_unavailable",
        false);

      return;
    }

    bool finish_before_forwarding = false;
    bool canceled_before_forwarding = false;
    std::string local_reason;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_goal_ ||
        generation != active_generation_)
      {
        return;
      }

      const auto decision = EvaluateLocked();

      if (cancellation_requested_) {
        finish_before_forwarding = true;
        canceled_before_forwarding =
          cancellation_reason_ == "external_cancel";
        local_reason = cancellation_reason_;
      }

      if (
        !finish_before_forwarding &&
        !decision.accept_new_goal &&
        decision.reason != "goal_gateway_busy")
      {
        cancellation_requested_ = true;
        cancellation_reason_ = decision.reason;

        finish_before_forwarding = true;
        local_reason = decision.reason;
      }
    }

    if (finish_before_forwarding) {
      FinishLocally(
        generation,
        local_reason,
        canceled_before_forwarding);

      return;
    }

    ActionClient::SendGoalOptions options;

    options.goal_response_callback =
      [this, generation](
      const InternalGoalHandle::SharedPtr internal_handle)
      {
        OnInternalGoalResponse(
          generation,
          internal_handle);
      };

    options.feedback_callback =
      [this, generation](
      InternalGoalHandle::SharedPtr,
      const std::shared_ptr<
        const NavigateToPose::Feedback> feedback)
      {
        OnInternalFeedback(
          generation,
          feedback);
      };

    options.result_callback =
      [this, generation](
      const InternalGoalHandle::WrappedResult & result)
      {
        OnInternalResult(
          generation,
          result);
      };

    try {
      static_cast<void>(
        client->async_send_goal(
          *external_handle->get_goal(),
          options));
    } catch (const std::exception & exception) {
      FinishLocally(
        generation,
        std::string("internal_goal_send_error:") +
        exception.what(),
        false);
    }
  }

  void OnInternalGoalResponse(
    const std::uint64_t generation,
    const InternalGoalHandle::SharedPtr internal_handle)
  {
    bool request_cancel = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_goal_ ||
        generation != active_generation_)
      {
        return;
      }

      if (internal_handle == nullptr) {
        // The terminal transition is handled outside the lock.
      } else {
        internal_goal_handle_ = internal_handle;

        request_cancel =
          cancellation_requested_ &&
          !internal_cancel_sent_;
      }
    }

    if (internal_handle == nullptr) {
      FinishLocally(
        generation,
        "internal_goal_gateway_rejected_goal",
        false);

      return;
    }

    PublishState("active", "internal_goal_accepted");

    if (request_cancel) {
      RequestInternalCancel(generation);
    }
  }

  void OnInternalFeedback(
    const std::uint64_t generation,
    const std::shared_ptr<
      const NavigateToPose::Feedback> & feedback)
  {
    std::shared_ptr<ExternalGoalHandle> external_handle;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_goal_ ||
        generation != active_generation_)
      {
        return;
      }

      external_handle = external_goal_handle_;
    }

    if (external_handle != nullptr && feedback != nullptr) {
      const auto mutable_feedback =
        std::make_shared<NavigateToPose::Feedback>(
        *feedback);

      external_handle->publish_feedback(
        mutable_feedback);
    }
  }

  void RequestInternalCancel(
    const std::uint64_t generation)
  {
    ActionClient::SharedPtr client;
    InternalGoalHandle::SharedPtr internal_handle;
    std::string reason;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_goal_ ||
        generation != active_generation_ ||
        internal_goal_handle_ == nullptr ||
        internal_cancel_sent_)
      {
        return;
      }

      internal_cancel_sent_ = true;

      client = ClientFor(active_source_);
      internal_handle = internal_goal_handle_;
      reason = cancellation_reason_;
    }

    PublishState("canceling", reason);

    try {
      static_cast<void>(
        client->async_cancel_goal(
          internal_handle,
          [this, generation](
            const std::shared_ptr<CancelResponse>)
          {
            bool still_active = false;

            {
              std::lock_guard<std::mutex> lock(mutex_);
              still_active =
              active_goal_ &&
              generation == active_generation_;
            }

            if (still_active) {
              PublishState(
                "canceling",
                "internal_cancel_requested");
            }
          }));
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "Internal cancellation request failed: %s",
        exception.what());
    }
  }

  void OnInternalResult(
    const std::uint64_t generation,
    const InternalGoalHandle::WrappedResult & wrapped)
  {
    std::shared_ptr<ExternalGoalHandle> external_handle;
    bool external_cancel = false;
    std::string interruption_reason;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_goal_ ||
        generation != active_generation_)
      {
        return;
      }

      external_handle = external_goal_handle_;

      external_cancel =
        cancellation_reason_ == "external_cancel";

      interruption_reason = cancellation_reason_;

      ClearActiveLocked();
    }

    if (external_handle == nullptr) {
      return;
    }

    auto result = wrapped.result;

    if (result == nullptr) {
      result = MakeErrorResult(
        "internal_goal_returned_null_result");
    }

    switch (wrapped.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        external_handle->succeed(result);
        PublishState("succeeded", "internal_goal_succeeded");
        return;

      case rclcpp_action::ResultCode::CANCELED:
        if (external_cancel && external_handle->is_canceling()) {
          external_handle->canceled(result);
          PublishState("canceled", "external_cancel_acknowledged");
        } else {
          const std::string reason =
            interruption_reason.empty() ?
            "control_recovery_interruption" :
            interruption_reason;

          external_handle->abort(
            MakeErrorResult(reason));

          PublishState("interrupted", reason);
        }
        return;

      case rclcpp_action::ResultCode::ABORTED:
        external_handle->abort(result);
        PublishState("aborted", "internal_goal_aborted");
        return;

      case rclcpp_action::ResultCode::UNKNOWN:
        external_handle->abort(
          MakeErrorResult("internal_goal_unknown_result"));
        PublishState("error", "internal_goal_unknown_result");
        return;
    }
  }

  void FinishLocally(
    const std::uint64_t generation,
    const std::string & reason,
    const bool canceled)
  {
    std::shared_ptr<ExternalGoalHandle> external_handle;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (
        !active_goal_ ||
        generation != active_generation_)
      {
        return;
      }

      external_handle = external_goal_handle_;
      ClearActiveLocked();
    }

    if (external_handle == nullptr) {
      return;
    }

    if (canceled && external_handle->is_canceling()) {
      external_handle->canceled(
        MakeErrorResult(reason));

      PublishState("canceled", reason);
      return;
    }

    external_handle->abort(
      MakeErrorResult(reason));

    PublishState("aborted", reason);
  }

  void ClearActiveLocked()
  {
    active_goal_ = false;
    slot_reserved_ = false;

    external_goal_handle_.reset();
    internal_goal_handle_.reset();

    cancellation_requested_ = false;
    internal_cancel_sent_ = false;
    cancellation_reason_.clear();
  }

  void OnTimer()
  {
    std::uint64_t generation = 0;

    bool request_pose_cancel = false;
    bool request_coverage_cancel = false;
    bool active = false;

    savo_nav::GoalAdmissionDecision decision;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      active = active_goal_;
      decision = EvaluateLocked();

      if (decision.request_active_cancel) {
        cancellation_requested_ = true;
        cancellation_reason_ = decision.reason;
        generation = active_generation_;

        request_coverage_cancel =
          coverage_proxy_ &&
          coverage_proxy_->ActiveLocked();

        request_pose_cancel =
          !request_coverage_cancel &&
          internal_goal_handle_ != nullptr &&
          !internal_cancel_sent_;
      }
    }

    if (decision.request_active_cancel) {
      PublishState("canceling", decision.reason);
    } else if (!active) {
      PublishState(
        "idle",
        decision.accept_new_goal ?
        "goal_admission_allowed" :
        decision.reason);
    }

    if (request_coverage_cancel) {
      coverage_proxy_->RequestCancel(
        decision.reason);
    } else if (request_pose_cancel) {
      RequestInternalCancel(generation);
    }
  }

  std::shared_ptr<NavigateToPose::Result> MakeErrorResult(
    const std::string & reason) const
  {
    auto result =
      std::make_shared<NavigateToPose::Result>();

    result->error_code =
      std::numeric_limits<std::uint16_t>::max();

    result->error_msg = reason;

    return result;
  }

  void PublishState(
    const std::string & state,
    const std::string & reason)
  {
    std_msgs::msg::String state_message;
    state_message.data = state;
    state_publisher_->publish(state_message);

    std_msgs::msg::String reason_message;
    reason_message.data = reason;
    reason_publisher_->publish(reason_message);

    std_msgs::msg::String status_message;
    std::ostringstream stream;

    stream
      << "state=" << state
      << ";reason=" << reason;

    status_message.data = stream.str();
    status_publisher_->publish(status_message);
  }

  mutable std::mutex mutex_;

  double publish_hz_{20.0};
  double guard_timeout_seconds_{0.75};
  double internal_server_timeout_seconds_{1.0};

  bool guard_observed_{false};
  bool guard_allowed_{false};
  double guard_stamp_seconds_{0.0};
  std::string guard_reason_{
    "control_recovery_guard_unobserved"};

  bool slot_reserved_{false};
  bool active_goal_{false};
  GoalSource active_source_{
    GoalSource::kNavigation};

  std::uint64_t active_generation_{0};

  bool cancellation_requested_{false};
  bool internal_cancel_sent_{false};
  std::string cancellation_reason_{};

  std::shared_ptr<ExternalGoalHandle>
  external_goal_handle_;

  InternalGoalHandle::SharedPtr
    internal_goal_handle_;

  std::string public_navigation_action_;
  std::string public_exploration_action_;
  std::string internal_navigation_action_;
  std::string internal_exploration_action_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    guard_allowed_subscription_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    guard_reason_subscription_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    state_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    reason_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    status_publisher_;

  ActionClient::SharedPtr navigation_client_;
  ActionClient::SharedPtr exploration_client_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr
    navigation_server_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr
    exploration_server_;

  std::unique_ptr<
    savo_nav::CoverageAdmissionProxy>
  coverage_proxy_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    rclcpp::spin(
      std::make_shared<GoalAdmissionGateNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "goal_admission_gate_node"),
      "Fatal error: %s",
      exception.what());

    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
