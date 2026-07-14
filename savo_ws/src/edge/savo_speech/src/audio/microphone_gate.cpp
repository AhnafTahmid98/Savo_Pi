#include "savo_speech/audio/microphone_gate.hpp"

#include <chrono>
#include <stdexcept>

namespace savo_speech::audio
{

MicrophoneGate::MicrophoneGate(
  const MicrophoneGateConfig config)
: config_{config}
{
  if (!config_.is_valid()) {
    throw std::invalid_argument{
            "invalid microphone-gate configuration"};
  }
}

void MicrophoneGate::begin_playback()
{
  const std::scoped_lock lock{mutex_};

  if (!playback_active_) {
    ++playback_sessions_;
  }

  playback_active_ = true;
  hold_until_ = Clock::time_point{};
}

void MicrophoneGate::end_playback(
  const Clock::time_point now)
{
  const std::scoped_lock lock{mutex_};

  if (!playback_active_) {
    return;
  }

  playback_active_ = false;
  hold_until_ = now + config_.post_playback_hold;
}

void MicrophoneGate::set_manual_gate(
  const bool enabled)
{
  const std::scoped_lock lock{mutex_};
  manual_gate_ = enabled;
}

void MicrophoneGate::set_shutdown_gate(
  const bool enabled)
{
  const std::scoped_lock lock{mutex_};
  shutdown_gate_ = enabled;
}

bool MicrophoneGate::is_gated(
  const Clock::time_point now) const
{
  const std::scoped_lock lock{mutex_};
  return state_locked(now).gated;
}

MicrophoneGateState MicrophoneGate::state(
  const Clock::time_point now) const
{
  const std::scoped_lock lock{mutex_};
  return state_locked(now);
}

MicrophoneGateState MicrophoneGate::state_locked(
  const Clock::time_point now) const
{
  MicrophoneGateState result;

  result.playback_active = playback_active_;
  result.manual_gate = manual_gate_;
  result.shutdown_gate = shutdown_gate_;
  result.playback_sessions = playback_sessions_;

  if (shutdown_gate_) {
    result.gated = true;
    result.reason = MicrophoneGateReason::Shutdown;
    return result;
  }

  if (manual_gate_) {
    result.gated = true;
    result.reason = MicrophoneGateReason::Manual;
    return result;
  }

  if (playback_active_) {
    result.gated = true;
    result.reason = MicrophoneGateReason::Playback;
    return result;
  }

  if (hold_until_ > now) {
    result.gated = true;
    result.reason =
      MicrophoneGateReason::PostPlaybackHold;

    result.remaining_hold =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      hold_until_ - now);

    return result;
  }

  result.gated = false;
  result.reason = MicrophoneGateReason::Open;

  return result;
}

}  // namespace savo_speech::audio
