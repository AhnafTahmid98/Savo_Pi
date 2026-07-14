#ifndef SAVO_SPEECH__AUDIO__MICROPHONE_GATE_HPP_
#define SAVO_SPEECH__AUDIO__MICROPHONE_GATE_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string_view>

namespace savo_speech::audio
{

enum class MicrophoneGateReason : std::uint8_t
{
  Open = 0U,
  Playback = 1U,
  PostPlaybackHold = 2U,
  Manual = 3U,
  Shutdown = 4U
};

[[nodiscard]] constexpr std::string_view to_string(
  const MicrophoneGateReason reason) noexcept
{
  switch (reason) {
    case MicrophoneGateReason::Open:
      return "open";

    case MicrophoneGateReason::Playback:
      return "playback";

    case MicrophoneGateReason::PostPlaybackHold:
      return "post_playback_hold";

    case MicrophoneGateReason::Manual:
      return "manual";

    case MicrophoneGateReason::Shutdown:
      return "shutdown";
  }

  return "unknown";
}

struct MicrophoneGateConfig
{
  std::chrono::milliseconds post_playback_hold{250};

  [[nodiscard]] bool is_valid() const noexcept
  {
    return
      post_playback_hold.count() >= 0 &&
      post_playback_hold <= std::chrono::seconds{10};
  }
};

struct MicrophoneGateState
{
  bool gated{false};
  bool playback_active{false};
  bool manual_gate{false};
  bool shutdown_gate{false};

  MicrophoneGateReason reason{
    MicrophoneGateReason::Open};

  std::chrono::milliseconds remaining_hold{0};

  std::uint64_t playback_sessions{0U};
};

class MicrophoneGate final
{
public:
  using Clock = std::chrono::steady_clock;

  explicit MicrophoneGate(
    MicrophoneGateConfig config = MicrophoneGateConfig{});

  MicrophoneGate(const MicrophoneGate &) = delete;
  MicrophoneGate & operator=(const MicrophoneGate &) = delete;

  MicrophoneGate(MicrophoneGate &&) = delete;
  MicrophoneGate & operator=(MicrophoneGate &&) = delete;

  void begin_playback();

  void end_playback(
    Clock::time_point now = Clock::now());

  void set_manual_gate(bool enabled);
  void set_shutdown_gate(bool enabled);

  [[nodiscard]] bool is_gated(
    Clock::time_point now = Clock::now()) const;

  [[nodiscard]] MicrophoneGateState state(
    Clock::time_point now = Clock::now()) const;

private:
  [[nodiscard]] MicrophoneGateState state_locked(
    Clock::time_point now) const;

  mutable std::mutex mutex_;

  MicrophoneGateConfig config_;

  bool playback_active_{false};
  bool manual_gate_{false};
  bool shutdown_gate_{false};

  Clock::time_point hold_until_{};

  std::uint64_t playback_sessions_{0U};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__MICROPHONE_GATE_HPP_
