#ifndef SAVO_MAPPING__SCAN360_ROTATE_ACTION_CLIENT_HPP_
#define SAVO_MAPPING__SCAN360_ROTATE_ACTION_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "savo_msgs/action/rotate_to_heading.hpp"

namespace savo_mapping
{

class Scan360RotateActionClient final
  : public std::enable_shared_from_this<Scan360RotateActionClient>
{
public:
  using Action = savo_msgs::action::RotateToHeading;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using SharedPtr = std::shared_ptr<Scan360RotateActionClient>;

  enum class State : std::uint8_t
  {
    kIdle = 0,
    kWaitingForServer,
    kWaitingForGoalResponse,
    kActive,
    kCanceling,
    kSucceeded,
    kCanceled,
    kRejected,
    kAborted,
    kTimedOut,
    kFailed,
  };

  struct Options
  {
    std::string action_name{"/savo_control/rotate_to_heading"};
    double server_wait_timeout_sec{1.0};
    double goal_response_timeout_sec{1.5};
    double feedback_stale_timeout_sec{3.0};
    double cancel_timeout_sec{2.0};
    double execution_grace_sec{1.0};
  };

  struct Snapshot
  {
    State state{State::kIdle};
    bool active{false};
    bool terminal{false};
    bool cancel_requested{false};
    bool cancel_acknowledged{false};
    double target_yaw_rad{0.0};
    double current_yaw_rad{0.0};
    double error_rad{0.0};
    double commanded_wz_rad_s{0.0};
    double elapsed_sec{0.0};
    bool safety_stop_active{false};
    std::string reason{"idle"};
  };

  using UpdateCallback = std::function<void(const Snapshot &)>;

  static SharedPtr create(const rclcpp::Node::SharedPtr & node);

  static SharedPtr create(
    const rclcpp::Node::SharedPtr & node,
    Options options);

  ~Scan360RotateActionClient();

  Scan360RotateActionClient(
    const Scan360RotateActionClient &) = delete;

  Scan360RotateActionClient & operator=(
    const Scan360RotateActionClient &) = delete;

  Scan360RotateActionClient(
    Scan360RotateActionClient &&) = delete;

  Scan360RotateActionClient & operator=(
    Scan360RotateActionClient &&) = delete;

  bool request_rotation(
    double target_yaw_rad,
    double max_duration_sec);

  bool request_cancel(
    const std::string & reason =
    "scan360_cancel_requested");

  void tick();

  [[nodiscard]] Snapshot snapshot() const;

  void set_update_callback(UpdateCallback callback);

  [[nodiscard]] static double normalize_yaw(
    double yaw_rad);

  [[nodiscard]] static const char * to_string(
    State state) noexcept;

private:
  using SteadyClock = std::chrono::steady_clock;
  using TimePoint = SteadyClock::time_point;

  Scan360RotateActionClient(
    rclcpp::Node::SharedPtr node,
    Options options);

  void initialize();
  void send_pending_goal();

  void begin_cancel(
    const std::string & reason,
    State terminal_state);

  void send_cancel_request(
    const GoalHandle::SharedPtr & goal_handle);

  void handle_goal_response(
    const GoalHandle::SharedPtr & goal_handle);

  void handle_feedback(
    GoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const Action::Feedback>
    feedback);

  void handle_result(
    const GoalHandle::WrappedResult &
    wrapped_result);

  void handle_cancel_response(
    const rclcpp_action::Client<Action>::
    CancelResponse::SharedPtr & response);

  void set_terminal_locked(
    State state,
    const std::string & reason);

  [[nodiscard]] bool busy_locked() const noexcept;

  void notify_update();

  rclcpp::Node::SharedPtr node_;
  Options options_;
  rclcpp_action::Client<Action>::SharedPtr client_;

  mutable std::mutex mutex_;
  Snapshot snapshot_;
  UpdateCallback update_callback_;
  GoalHandle::SharedPtr active_goal_;

  TimePoint state_deadline_{};
  TimePoint execution_deadline_{};
  TimePoint last_feedback_at_{};

  double requested_max_duration_sec_{0.0};
  State cancel_terminal_state_{State::kCanceled};
  std::string cancel_reason_{
    "scan360_cancel_requested"};
};

}  // namespace savo_mapping

#endif  // SAVO_MAPPING__SCAN360_ROTATE_ACTION_CLIENT_HPP_
