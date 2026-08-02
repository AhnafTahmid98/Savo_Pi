// Copyright 2026 Ahnaf Tahmid
#ifndef SAVO_SPEECH__TRANSPORT__SAVOMIND_ROUND_TRIP_WORKER_HPP_
#define SAVO_SPEECH__TRANSPORT__SAVOMIND_ROUND_TRIP_WORKER_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <stop_token>

#include "savo_speech/audio/audio_runtime.hpp"
#include "savo_speech/session/completed_utterance_worker.hpp"
#include "savo_speech/transport/savomind_transport.hpp"

namespace savo_speech::transport
{

enum class RoundTripState : std::uint8_t
{
  Stopped = 0U,
  Starting = 1U,
  Waiting = 2U,
  Sending = 3U,
  AwaitingResponse = 4U,
  EnqueueingPlayback = 5U,
  Playing = 6U,
  AcknowledgingPlayback = 7U,
  Completed = 8U,
  Canceling = 9U,
  Faulted = 10U,
};

[[nodiscard]] constexpr std::string_view to_string(RoundTripState state) noexcept
{
  switch (state) {
    case RoundTripState::Stopped: return "stopped";
    case RoundTripState::Starting: return "starting";
    case RoundTripState::Waiting: return "waiting";
    case RoundTripState::Sending: return "sending";
    case RoundTripState::AwaitingResponse: return "awaiting_response";
    case RoundTripState::EnqueueingPlayback: return "enqueueing_playback";
    case RoundTripState::Playing: return "playing";
    case RoundTripState::AcknowledgingPlayback: return "acknowledging_playback";
    case RoundTripState::Completed: return "completed";
    case RoundTripState::Canceling: return "canceling";
    case RoundTripState::Faulted: return "faulted";
  }
  return "unknown";
}

struct RoundTripConfig
{
  std::string request_prefix{"robot-savo"};
  std::chrono::milliseconds source_wait_timeout{100};
  std::chrono::milliseconds playback_completion_timeout{60000};
  std::size_t maximum_tts_wav_bytes{16U * 1024U * 1024U};

  [[nodiscard]] bool is_valid() const noexcept;
};

struct RoundTripEvent
{
  RoundTripState state{RoundTripState::Stopped};
  std::uint64_t utterance_id{0U};
  std::string request_id;
  std::string transcript;
  std::string reply;
  std::string reason;
};

struct RoundTripStatistics
{
  std::uint64_t starts{0U};
  std::uint64_t stops{0U};
  std::uint64_t utterances_received{0U};
  std::uint64_t exchanges_succeeded{0U};
  std::uint64_t exchanges_failed{0U};
  std::uint64_t playback_enqueued{0U};
  std::uint64_t playback_completed{0U};
  std::uint64_t playback_failed{0U};
  std::uint64_t playback_acks_succeeded{0U};
  std::uint64_t playback_acks_failed{0U};
  std::uint64_t cancellations{0U};
};

struct RoundTripSnapshot
{
  RoundTripState state{RoundTripState::Stopped};
  bool running{false};
  bool transport_healthy{false};
  std::optional<std::uint64_t> active_utterance_id;
  std::string active_request_id;
  std::string last_transcript;
  std::string last_reply;
  std::string last_error;
  RoundTripStatistics statistics{};
};

class SavoMindRoundTripWorker final
{
public:
  using EventCallback = std::function<void(const RoundTripEvent &)>;

  SavoMindRoundTripWorker(
    session::CompletedUtteranceWorker & source,
    SavoMindTransport & transport,
    audio::AudioRuntime & audio_runtime,
    RoundTripConfig config,
    EventCallback callback = {});

  ~SavoMindRoundTripWorker();

  SavoMindRoundTripWorker(const SavoMindRoundTripWorker &) = delete;
  SavoMindRoundTripWorker & operator=(const SavoMindRoundTripWorker &) = delete;

  [[nodiscard]] bool start();
  void stop() noexcept;
  void cancel_active() noexcept;
  [[nodiscard]] RoundTripSnapshot snapshot() const;

private:
  void run(std::stop_token stop_token);
  void process(session::SerializedUtterance utterance, std::stop_token stop_token);
  void set_event(RoundTripEvent event);
  void fail(
    std::uint64_t utterance_id,
    const std::string & request_id,
    const std::string & reason,
    bool exchange_failure);
  [[nodiscard]] bool acknowledge_playback(
    const Request & request,
    const Response & response,
    bool playback_ok,
    const std::string & reason,
    std::uint64_t utterance_id,
    std::string & error);

  session::CompletedUtteranceWorker & source_;
  SavoMindTransport & transport_;
  audio::AudioRuntime & audio_runtime_;
  RoundTripConfig config_;
  EventCallback callback_;

  mutable std::mutex mutex_;
  std::jthread thread_;
  RoundTripSnapshot snapshot_{};
};

}  // namespace savo_speech::transport

#endif  // SAVO_SPEECH__TRANSPORT__SAVOMIND_ROUND_TRIP_WORKER_HPP_
