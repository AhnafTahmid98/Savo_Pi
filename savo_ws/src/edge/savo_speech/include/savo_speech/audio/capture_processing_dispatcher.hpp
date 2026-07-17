#ifndef SAVO_SPEECH__AUDIO__CAPTURE_PROCESSING_DISPATCHER_HPP_
#define SAVO_SPEECH__AUDIO__CAPTURE_PROCESSING_DISPATCHER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>

#include "savo_speech/audio/captured_audio_processor.hpp"
#include "savo_speech/audio/captured_frame_source.hpp"

namespace savo_speech::audio
{

enum class CaptureProcessingState : std::uint8_t
{
  Stopped = 0U,
  Starting = 1U,
  Running = 2U,
  Stopping = 3U,
  Faulted = 4U
};

[[nodiscard]] constexpr std::string_view to_string(
  const CaptureProcessingState state) noexcept
{
  switch (state) {
    case CaptureProcessingState::Stopped:
      return "stopped";

    case CaptureProcessingState::Starting:
      return "starting";

    case CaptureProcessingState::Running:
      return "running";

    case CaptureProcessingState::Stopping:
      return "stopping";

    case CaptureProcessingState::Faulted:
      return "faulted";
  }

  return "unknown";
}

struct CaptureProcessingConfig
{
  std::chrono::milliseconds wait_timeout{100};
  std::chrono::milliseconds freshness_timeout{1000};

  bool fault_on_processor_error{true};

  [[nodiscard]] bool is_valid() const noexcept
  {
    return
      wait_timeout >= std::chrono::milliseconds{1} &&
      wait_timeout <= std::chrono::seconds{5} &&
      freshness_timeout >= wait_timeout &&
      freshness_timeout <= std::chrono::seconds{30};
  }
};

struct CaptureProcessingStatistics
{
  std::uint64_t starts{0U};
  std::uint64_t stops{0U};

  std::uint64_t frames_received{0U};
  std::uint64_t frames_processed{0U};

  std::uint64_t wait_timeouts{0U};
  std::uint64_t processor_failures{0U};
  std::uint64_t source_failures{0U};

  std::uint64_t missing_sequence_frames{0U};
  std::uint64_t out_of_order_frames{0U};

  std::uint64_t faults{0U};

  std::uint64_t last_sequence{0U};
};

struct CaptureProcessingHealthSnapshot
{
  CaptureProcessingState state{
    CaptureProcessingState::Stopped};

  bool ready{false};
  bool fresh{false};

  std::int64_t last_frame_age_ms{-1};

  CaptureProcessingStatistics statistics{};

  std::string last_error{};
};

class CaptureProcessingDispatcher final
{
public:
  using Clock = std::chrono::steady_clock;

  CaptureProcessingDispatcher(
    CapturedFrameSource & source,
    CapturedAudioProcessor & processor,
    CaptureProcessingConfig config =
      CaptureProcessingConfig{});

  ~CaptureProcessingDispatcher();

  CaptureProcessingDispatcher(
    const CaptureProcessingDispatcher &) = delete;

  CaptureProcessingDispatcher & operator=(
    const CaptureProcessingDispatcher &) = delete;

  CaptureProcessingDispatcher(
    CaptureProcessingDispatcher &&) = delete;

  CaptureProcessingDispatcher & operator=(
    CaptureProcessingDispatcher &&) = delete;

  [[nodiscard]] bool start();
  void stop() noexcept;

  [[nodiscard]] CaptureProcessingState
  state() const noexcept;

  [[nodiscard]] bool running() const noexcept;

  [[nodiscard]] CaptureProcessingStatistics
  statistics() const noexcept;

  [[nodiscard]] CaptureProcessingHealthSnapshot
  health_snapshot() const;

  [[nodiscard]] std::string last_error() const;

private:
  void run(std::stop_token stop_token);

  void record_source_failure(
    const std::string & error) noexcept;

  void record_processor_failure(
    const std::string & error) noexcept;

  CapturedFrameSource & source_;
  CapturedAudioProcessor & processor_;

  CaptureProcessingConfig config_;

  mutable std::mutex mutex_;

  std::jthread thread_;

  CaptureProcessingState state_{
    CaptureProcessingState::Stopped};

  CaptureProcessingStatistics statistics_{};

  std::optional<Clock::time_point>
  last_processed_at_{};

  std::string last_error_{};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__CAPTURE_PROCESSING_DISPATCHER_HPP_
