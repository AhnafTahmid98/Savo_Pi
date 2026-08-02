// Copyright 2026 Ahnaf Tahmid
#include "savo_speech/transport/savomind_round_trip_worker.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include "savo_speech/audio/wav_reader.hpp"

namespace savo_speech::transport
{

bool RoundTripConfig::is_valid() const noexcept
{
  return !request_prefix.empty() && request_prefix.size() <= 64U &&
         source_wait_timeout >= std::chrono::milliseconds{1} &&
         source_wait_timeout <= std::chrono::seconds{5} &&
         playback_completion_timeout >= std::chrono::seconds{1} &&
         playback_completion_timeout <= std::chrono::minutes{10} &&
         maximum_tts_wav_bytes >= 44U &&
         maximum_tts_wav_bytes <= 64U * 1024U * 1024U;
}

SavoMindRoundTripWorker::SavoMindRoundTripWorker(
  session::CompletedUtteranceWorker & source,
  SavoMindTransport & transport,
  audio::AudioRuntime & audio_runtime,
  RoundTripConfig config,
  EventCallback callback)
: source_{source},
  transport_{transport},
  audio_runtime_{audio_runtime},
  config_{std::move(config)},
  callback_{std::move(callback)}
{
  if (!config_.is_valid()) {throw std::invalid_argument{"invalid SavoMind round-trip config"};}
}

SavoMindRoundTripWorker::~SavoMindRoundTripWorker()
{
  stop();
}

bool SavoMindRoundTripWorker::start()
{
  std::lock_guard<std::mutex> lock{mutex_};
  if (thread_.joinable() || snapshot_.running) {return false;}
  snapshot_.state = RoundTripState::Starting;
  snapshot_.running = true;
  snapshot_.last_error.clear();
  ++snapshot_.statistics.starts;
  thread_ = std::jthread([this](const std::stop_token stop_token) {run(stop_token);});
  return true;
}

void SavoMindRoundTripWorker::stop() noexcept
{
  std::jthread thread;
  {
    std::lock_guard<std::mutex> lock{mutex_};
    if (!thread_.joinable()) {
      snapshot_.running = false;
      snapshot_.state = RoundTripState::Stopped;
      return;
    }
    snapshot_.state = RoundTripState::Canceling;
    if (!snapshot_.active_request_id.empty()) {
      transport_.cancel(snapshot_.active_request_id);
    }
    audio_runtime_.cancel_all_playback();
    thread_.request_stop();
    thread = std::move(thread_);
  }
  if (thread.joinable()) {thread.join();}
  {
    std::lock_guard<std::mutex> lock{mutex_};
    snapshot_.running = false;
    snapshot_.state = RoundTripState::Stopped;
    snapshot_.active_request_id.clear();
    snapshot_.active_utterance_id.reset();
    ++snapshot_.statistics.stops;
  }
}

void SavoMindRoundTripWorker::cancel_active() noexcept
{
  std::lock_guard<std::mutex> lock{mutex_};
  if (snapshot_.active_request_id.empty()) {return;}
  snapshot_.state = RoundTripState::Canceling;
  transport_.cancel(snapshot_.active_request_id);
  audio_runtime_.cancel_all_playback();
  ++snapshot_.statistics.cancellations;
}

RoundTripSnapshot SavoMindRoundTripWorker::snapshot() const
{
  std::lock_guard<std::mutex> lock{mutex_};
  RoundTripSnapshot result = snapshot_;
  result.transport_healthy = transport_.healthy();
  return result;
}

void SavoMindRoundTripWorker::set_event(RoundTripEvent event)
{
  EventCallback callback;
  {
    std::lock_guard<std::mutex> lock{mutex_};
    snapshot_.state = event.state;
    snapshot_.active_utterance_id = event.utterance_id == 0U ?
      std::optional<std::uint64_t>{} : std::optional<std::uint64_t>{event.utterance_id};
    snapshot_.active_request_id = event.request_id;
    if (!event.transcript.empty()) {snapshot_.last_transcript = event.transcript;}
    if (!event.reply.empty()) {snapshot_.last_reply = event.reply;}
    if (event.state == RoundTripState::Faulted) {snapshot_.last_error = event.reason;}
    callback = callback_;
  }
  if (callback) {callback(event);}
}

void SavoMindRoundTripWorker::fail(
  const std::uint64_t utterance_id,
  const std::string & request_id,
  const std::string & reason,
  const bool exchange_failure)
{
  if (exchange_failure) {
    std::lock_guard<std::mutex> lock{mutex_};
    ++snapshot_.statistics.exchanges_failed;
  }
  set_event({RoundTripState::Faulted, utterance_id, request_id, {}, {}, reason});
}

void SavoMindRoundTripWorker::run(const std::stop_token stop_token)
{
  set_event({RoundTripState::Waiting, 0U, {}, {}, {}, "waiting_for_utterance"});
  while (!stop_token.stop_requested()) {
    auto utterance = source_.wait_serialized_for(config_.source_wait_timeout);
    if (stop_token.stop_requested()) {break;}
    if (!utterance.has_value()) {continue;}
    {
      std::lock_guard<std::mutex> lock{mutex_};
      ++snapshot_.statistics.utterances_received;
    }
    process(std::move(*utterance), stop_token);
    if (!stop_token.stop_requested()) {
      set_event({RoundTripState::Waiting, 0U, {}, {}, {}, "waiting_for_utterance"});
    }
  }
}

bool SavoMindRoundTripWorker::acknowledge_playback(
  const Request & request,
  const Response & response,
  const bool playback_ok,
  const std::string & reason,
  const std::uint64_t utterance_id,
  std::string & error)
{
  if (!response.playback_ack_required) {return true;}
  if (response.playback_token.empty()) {
    error = "playback_token_missing";
    std::lock_guard<std::mutex> lock{mutex_};
    ++snapshot_.statistics.playback_acks_failed;
    return false;
  }
  set_event({
      RoundTripState::AcknowledgingPlayback,
      utterance_id,
      request.request_id,
      response.transcript,
      response.reply,
      playback_ok ? "acknowledging_playback_complete" :
      "acknowledging_playback_failure"});
  PlaybackAckRequest ack;
  ack.request_id = request.request_id;
  ack.session_id = request.session_id;
  ack.playback_token = response.playback_token;
  ack.playback_ok = playback_ok;
  ack.reason = reason;
  try {
    (void)transport_.acknowledge_playback(ack);
    std::lock_guard<std::mutex> lock{mutex_};
    ++snapshot_.statistics.playback_acks_succeeded;
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    std::lock_guard<std::mutex> lock{mutex_};
    ++snapshot_.statistics.playback_acks_failed;
    return false;
  }
}

void SavoMindRoundTripWorker::process(
  session::SerializedUtterance utterance,
  const std::stop_token stop_token)
{
  if (!utterance.is_valid()) {
    fail(utterance.utterance_id, {}, "serialized_utterance_invalid", true);
    return;
  }
  const std::string request_id = config_.request_prefix + "-" +
    std::to_string(utterance.utterance_id);
  Request request;
  request.request_id = request_id;
  request.session_id = config_.request_prefix + "-wake-" +
    std::to_string(utterance.wake_event_id);
  request.wav = std::move(utterance.wav_bytes);

  bool exchange_completed = false;
  try {
    set_event({RoundTripState::Sending, utterance.utterance_id, request_id, {}, {}, "sending"});
    set_event({RoundTripState::AwaitingResponse, utterance.utterance_id, request_id,
        {}, {}, "awaiting_savomind"});
    Response response = transport_.exchange(request);
    exchange_completed = true;
    {
      std::lock_guard<std::mutex> lock{mutex_};
      ++snapshot_.statistics.exchanges_succeeded;
    }
    if (stop_token.stop_requested()) {
      std::string ack_error;
      (void)acknowledge_playback(
        request, response, false, "speech_worker_stopped",
        utterance.utterance_id, ack_error);
      return;
    }

    audio::WavReadLimits limits;
    limits.maximum_file_bytes = config_.maximum_tts_wav_bytes;
    limits.maximum_audio_data_bytes = config_.maximum_tts_wav_bytes - 44U;
    limits.maximum_channels = 1U;
    limits.minimum_sample_rate_hz = 16000U;
    limits.maximum_sample_rate_hz = 16000U;
    auto buffer = audio::WavReader::decode(response.tts_wav, limits);

    set_event({RoundTripState::EnqueueingPlayback, utterance.utterance_id, request_id,
        response.transcript, response.reply, "enqueueing_playback"});
    audio::PlaybackRequest playback;
    playback.request_id = utterance.utterance_id;
    playback.audio = std::move(buffer);
    const auto enqueue = audio_runtime_.enqueue_playback(std::move(playback));
    if (enqueue != audio::PlaybackEnqueueResult::Accepted) {
      const std::string reason =
        "playback_enqueue_" + std::string{audio::to_string(enqueue)};
      std::string ack_error;
      (void)acknowledge_playback(
        request, response, false, reason, utterance.utterance_id, ack_error);
      {
        std::lock_guard<std::mutex> lock{mutex_};
        ++snapshot_.statistics.playback_failed;
      }
      fail(
        utterance.utterance_id,
        request_id,
        ack_error.empty() ? reason : reason + ":ack_failed:" + ack_error,
        false);
      return;
    }
    {
      std::lock_guard<std::mutex> lock{mutex_};
      ++snapshot_.statistics.playback_enqueued;
    }
    set_event({RoundTripState::Playing, utterance.utterance_id, request_id,
        response.transcript, response.reply, "playing"});

    const auto deadline = std::chrono::steady_clock::now() +
      config_.playback_completion_timeout;
    while (!stop_token.stop_requested() &&
      std::chrono::steady_clock::now() < deadline)
    {
      auto completion = audio_runtime_.wait_playback_completion_for(
        std::chrono::milliseconds{100});
      if (!completion.has_value()) {continue;}
      if (completion->request_id != utterance.utterance_id) {continue;}
      if (completion->status != audio::PlaybackCompletionStatus::Completed) {
        const std::string reason = completion->error.empty() ?
          "playback_not_completed" : completion->error;
        std::string ack_error;
        (void)acknowledge_playback(
          request, response, false, reason, utterance.utterance_id, ack_error);
        {
          std::lock_guard<std::mutex> lock{mutex_};
          ++snapshot_.statistics.playback_failed;
        }
        fail(
          utterance.utterance_id,
          request_id,
          ack_error.empty() ? reason : reason + ":ack_failed:" + ack_error,
          false);
        return;
      }
      {
        std::lock_guard<std::mutex> lock{mutex_};
        ++snapshot_.statistics.playback_completed;
      }
      std::string ack_error;
      if (!acknowledge_playback(
          request, response, true, {}, utterance.utterance_id, ack_error))
      {
        fail(
          utterance.utterance_id,
          request_id,
          "playback_ack_failed:" + ack_error,
          false);
        return;
      }
      set_event({RoundTripState::Completed, utterance.utterance_id, request_id,
          response.transcript, response.reply, "completed"});
      return;
    }

    audio_runtime_.cancel_all_playback();
    const std::string reason = stop_token.stop_requested() ?
      "speech_worker_stopped" : "playback_completion_timeout";
    std::string ack_error;
    (void)acknowledge_playback(
      request, response, false, reason, utterance.utterance_id, ack_error);
    if (!stop_token.stop_requested()) {
      {
        std::lock_guard<std::mutex> lock{mutex_};
        ++snapshot_.statistics.playback_failed;
      }
      fail(
        utterance.utterance_id,
        request_id,
        ack_error.empty() ? reason : reason + ":ack_failed:" + ack_error,
        false);
    }
  } catch (const std::exception & exception) {
    fail(utterance.utterance_id, request_id, exception.what(), !exchange_completed);
  }
}

}  // namespace savo_speech::transport
