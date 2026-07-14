#ifndef SAVO_SPEECH__AUDIO__CAPTURE_WORKER_HPP_
#define SAVO_SPEECH__AUDIO__CAPTURE_WORKER_HPP_

#include <cstdint>
#include <mutex>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>

#include "savo_speech/audio/capture_pipeline.hpp"
#include "savo_speech/audio/capture_source.hpp"

namespace savo_speech::audio
{

enum class CaptureWorkerState : std::uint8_t
{
  Stopped = 0U,
  Starting = 1U,
  Running = 2U,
  Stopping = 3U,
  Faulted = 4U
};

[[nodiscard]] constexpr std::string_view to_string(
  const CaptureWorkerState state) noexcept
{
  switch (state) {
    case CaptureWorkerState::Stopped:
      return "stopped";

    case CaptureWorkerState::Starting:
      return "starting";

    case CaptureWorkerState::Running:
      return "running";

    case CaptureWorkerState::Stopping:
      return "stopping";

    case CaptureWorkerState::Faulted:
      return "faulted";
  }

  return "unknown";
}

struct CaptureWorkerStatistics
{
  std::uint64_t starts{0U};
  std::uint64_t stops{0U};

  std::uint64_t frames_read{0U};
  std::uint64_t accepted_frames{0U};
  std::uint64_t gated_frames{0U};

  std::uint64_t queue_drop_events{0U};
  std::uint64_t rejected_frames{0U};

  std::uint64_t faults{0U};
};

class CaptureWorker final
{
public:
  CaptureWorker(
    CaptureSource & source,
    CapturePipeline & pipeline);

  ~CaptureWorker();

  CaptureWorker(const CaptureWorker &) = delete;
  CaptureWorker & operator=(const CaptureWorker &) = delete;

  CaptureWorker(CaptureWorker &&) = delete;
  CaptureWorker & operator=(CaptureWorker &&) = delete;

  [[nodiscard]] bool start();

  void stop() noexcept;

  [[nodiscard]] CaptureWorkerState state() const noexcept;
  [[nodiscard]] bool running() const noexcept;

  [[nodiscard]] CaptureWorkerStatistics
  statistics() const noexcept;

  [[nodiscard]] std::string last_error() const;

private:
  void run(std::stop_token stop_token);

  void record_pipeline_result(
    CapturePipelineResult result);

  void record_fault(const std::string & message) noexcept;

  CaptureSource & source_;
  CapturePipeline & pipeline_;

  mutable std::mutex mutex_;

  std::jthread thread_;

  CaptureWorkerState state_{
    CaptureWorkerState::Stopped};

  CaptureWorkerStatistics statistics_{};

  std::string last_error_{};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__CAPTURE_WORKER_HPP_
