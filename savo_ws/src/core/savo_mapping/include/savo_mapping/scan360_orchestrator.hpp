#pragma once

#include "savo_mapping/scan360_controller.hpp"

#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace savo_mapping::scan360
{

enum class RotationClientState
{
  Idle,
  Pending,
  Active,
  Canceling,
  Succeeded,
  Canceled,
  Rejected,
  Failed
};

struct RotationClientSnapshot
{
  RotationClientState state{
    RotationClientState::Idle};

  std::string reason{"idle"};
};

struct Scan360RotationCallbacks
{
  std::function<bool(double, double)>
  request_rotation;

  std::function<bool()> request_cancel;
  std::function<void()> tick;

  std::function<RotationClientSnapshot()>
  snapshot;
};

class Scan360Orchestrator
{
public:
  Scan360Orchestrator(
    Scan360RotationCallbacks callbacks,
    double rotation_duration_sec);

  void dispatch(
    const ControllerDecision & decision);

  void tick();

  std::vector<ControllerEvent> take_events();

  void reset();

  std::string_view last_reason() const;

private:
  enum class PendingOperation
  {
    None,
    Rotation,
    Cancel
  };

  void dispatch_rotation(
    const ControllerDecision & decision);

  void dispatch_cancel();

  void observe(
    const RotationClientSnapshot & snapshot);

  void emit_motion_accepted();

  void emit_terminal(
    ControllerEvent event,
    const std::string & reason);

  Scan360RotationCallbacks callbacks_;
  double rotation_duration_sec_;

  PendingOperation pending_operation_{
    PendingOperation::None};

  bool motion_accepted_emitted_{false};
  bool terminal_event_emitted_{false};

  std::vector<ControllerEvent> events_;
  std::string last_reason_{"idle"};
};

}  // namespace savo_mapping::scan360
