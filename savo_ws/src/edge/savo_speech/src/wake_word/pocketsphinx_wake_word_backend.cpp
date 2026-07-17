#include "savo_speech/wake_word/pocketsphinx_wake_word_backend.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include <cmd_ln.h>
#include <pocketsphinx.h>
#include <ps_search.h>

#include "savo_speech/audio/audio_format.hpp"

namespace savo_speech::wake_word
{

namespace
{

[[nodiscard]] std::string normalize_hypothesis(
  const std::string_view hypothesis)
{
  std::string normalized;
  normalized.reserve(hypothesis.size());

  bool previous_was_space{true};

  for (const char character : hypothesis) {
    const auto unsigned_character =
      static_cast<unsigned char>(character);

    if (std::isspace(unsigned_character) != 0) {
      if (
        !normalized.empty() &&
        !previous_was_space)
      {
        normalized.push_back(' ');
      }

      previous_was_space = true;
      continue;
    }

    normalized.push_back(
      static_cast<char>(
        std::tolower(unsigned_character)));

    previous_was_space = false;
  }

  if (
    !normalized.empty() &&
    normalized.back() == ' ')
  {
    normalized.pop_back();
  }

  return normalized;
}

void validate_runtime_paths(
  const PocketSphinxWakeWordBackendConfig & config)
{
  if (
    !std::filesystem::is_directory(
      config.acoustic_model_path))
  {
    throw std::invalid_argument{
            "PocketSphinx acoustic model directory "
            "does not exist: " +
            config.acoustic_model_path.string()};
  }

  if (
    !std::filesystem::is_regular_file(
      config.dictionary_path))
  {
    throw std::invalid_argument{
            "PocketSphinx dictionary file does not exist: " +
            config.dictionary_path.string()};
  }

  if (
    !std::filesystem::is_regular_file(
      config.keyword_file_path))
  {
    throw std::invalid_argument{
            "PocketSphinx keyword file does not exist: " +
            config.keyword_file_path.string()};
  }
}

}  // namespace

bool PocketSphinxWakeWordBackendConfig::is_valid()
const noexcept
{
  return
    !acoustic_model_path.empty() &&
    !dictionary_path.empty() &&
    !keyword_file_path.empty() &&
    !search_name.empty() &&
    sample_rate_hz == 16000U;
}

struct PocketSphinxWakeWordBackend::Impl
{
  explicit Impl(
    PocketSphinxWakeWordBackendConfig backend_config)
  : config{std::move(backend_config)}
  {
    if (!config.is_valid()) {
      throw std::invalid_argument{
              "invalid PocketSphinx wake-word "
              "backend configuration"};
    }

    validate_runtime_paths(config);

    try {
      initialize();
    } catch (...) {
      shutdown();
      throw;
    }
  }

  ~Impl()
  {
    shutdown();
  }

  void initialize()
  {
    const std::string sample_rate =
      std::to_string(config.sample_rate_hz);

    if (config.suppress_decoder_log) {
      decoder_config = cmd_ln_init(
        nullptr,
        ps_args(),
        1,
        "-hmm",
        config.acoustic_model_path.string().c_str(),
        "-dict",
        config.dictionary_path.string().c_str(),
        "-samprate",
        sample_rate.c_str(),
        "-bestpath",
        "no",
        "-logfn",
        "/dev/null",
        nullptr);
    } else {
      decoder_config = cmd_ln_init(
        nullptr,
        ps_args(),
        1,
        "-hmm",
        config.acoustic_model_path.string().c_str(),
        "-dict",
        config.dictionary_path.string().c_str(),
        "-samprate",
        sample_rate.c_str(),
        "-bestpath",
        "no",
        nullptr);
    }

    if (decoder_config == nullptr) {
      throw std::runtime_error{
              "cmd_ln_init failed for PocketSphinx"};
    }

    decoder = ps_init(decoder_config);

    if (decoder == nullptr) {
      throw std::runtime_error{
              "ps_init failed for PocketSphinx"};
    }

    if (
      ps_set_kws(
        decoder,
        config.search_name.c_str(),
        config.keyword_file_path.string().c_str()) < 0)
    {
      throw std::runtime_error{
              "ps_set_kws failed for search: " +
              config.search_name};
    }

    if (
      ps_set_search(
        decoder,
        config.search_name.c_str()) < 0)
    {
      throw std::runtime_error{
              "ps_set_search failed for search: " +
              config.search_name};
    }

    if (ps_start_stream(decoder) < 0) {
      throw std::runtime_error{
              "ps_start_stream failed"};
    }

    stream_started = true;

    if (ps_start_utt(decoder) < 0) {
      throw std::runtime_error{
              "ps_start_utt failed"};
    }

    utterance_active = true;
    initialized = true;
    last_error.clear();
  }

  void shutdown() noexcept
  {
    if (
      decoder != nullptr &&
      utterance_active)
    {
      static_cast<void>(
        ps_end_utt(decoder));

      utterance_active = false;
    }

    if (decoder != nullptr) {
      static_cast<void>(
        ps_free(decoder));

      decoder = nullptr;
    }

    if (decoder_config != nullptr) {
      static_cast<void>(
        cmd_ln_free_r(decoder_config));

      decoder_config = nullptr;
    }

    initialized = false;
    stream_started = false;
  }

  void restart_utterance()
  {
    if (decoder == nullptr) {
      throw std::runtime_error{
              "PocketSphinx decoder is not initialized"};
    }

    if (utterance_active) {
      if (ps_end_utt(decoder) < 0) {
        ++statistics.restart_failures;

        throw std::runtime_error{
                "ps_end_utt failed during decoder restart"};
      }

      utterance_active = false;
    }

    if (ps_start_utt(decoder) < 0) {
      ++statistics.restart_failures;

      throw std::runtime_error{
              "ps_start_utt failed during decoder restart"};
    }

    utterance_active = true;

    ++statistics.decoder_restarts;
  }

  PocketSphinxWakeWordBackendConfig config;

  mutable std::mutex mutex;

  cmd_ln_t * decoder_config{nullptr};
  ps_decoder_t * decoder{nullptr};

  bool initialized{false};
  bool stream_started{false};
  bool utterance_active{false};

  PocketSphinxWakeWordBackendStatistics statistics{};

  std::string last_hypothesis{};
  std::string last_error{};
};

PocketSphinxWakeWordBackend::
PocketSphinxWakeWordBackend(
  PocketSphinxWakeWordBackendConfig config)
: impl_{std::make_unique<Impl>(std::move(config))}
{
}

PocketSphinxWakeWordBackend::
~PocketSphinxWakeWordBackend() = default;

WakeWordBackendResult
PocketSphinxWakeWordBackend::analyze(
  const audio::AudioFrame & frame)
{
  if (!frame.is_consistent()) {
    throw std::invalid_argument{
            "PocketSphinx backend received an "
            "invalid audio frame"};
  }

  if (frame.format.channels != 1U) {
    throw std::invalid_argument{
            "PocketSphinx backend requires mono audio"};
  }

  if (
    frame.format.sample_rate_hz !=
    impl_->config.sample_rate_hz)
  {
    throw std::invalid_argument{
            "PocketSphinx backend received an "
            "unexpected sample rate"};
  }

  if (
    frame.format.sample_format !=
    audio::PcmSampleFormat::Signed16LittleEndian)
  {
    throw std::invalid_argument{
            "PocketSphinx backend requires S16_LE audio"};
  }

  const std::scoped_lock lock{impl_->mutex};

  if (
    !impl_->initialized ||
    impl_->decoder == nullptr ||
    !impl_->utterance_active)
  {
    throw std::runtime_error{
            "PocketSphinx backend is not ready"};
  }

  ++impl_->statistics.frames_analyzed;

  impl_->statistics.samples_processed +=
    static_cast<std::uint64_t>(
    frame.interleaved_samples.size());

  impl_->statistics.last_frame_sequence =
    frame.sequence;

  const int process_result = ps_process_raw(
    impl_->decoder,
    reinterpret_cast<const int16 *>(
      frame.interleaved_samples.data()),
    frame.interleaved_samples.size(),
    0,
    0);

  if (process_result < 0) {
    ++impl_->statistics.process_failures;

    impl_->last_error =
      "ps_process_raw failed";

    throw std::runtime_error{
            impl_->last_error};
  }

  int32 best_score{0};

  const char * hypothesis =
    ps_get_hyp(
    impl_->decoder,
    &best_score);

  if (hypothesis == nullptr) {
    return {};
  }

  const std::string normalized =
    normalize_hypothesis(hypothesis);

  if (normalized.empty()) {
    return {};
  }

  impl_->last_hypothesis = normalized;

  impl_->statistics.last_best_score =
    static_cast<std::int32_t>(best_score);

  ++impl_->statistics.detections;

  try {
    impl_->restart_utterance();
  } catch (const std::exception & exception) {
    impl_->last_error = exception.what();
    throw;
  }

  impl_->last_error.clear();

  WakeWordBackendResult result;

  result.detected = true;
  result.phrase = normalized;

  // PocketSphinx keyword thresholds perform the real acceptance
  // decision. The 5prealpha partial hypothesis API does not provide
  // a calibrated probability suitable for cross-phrase comparison.
  result.confidence = 1.0;

  return result;
}

void PocketSphinxWakeWordBackend::reset() noexcept
{
  const std::scoped_lock lock{impl_->mutex};

  ++impl_->statistics.reset_requests;

  impl_->last_hypothesis.clear();
  impl_->last_error.clear();

  if (
    impl_->decoder == nullptr ||
    !impl_->initialized)
  {
    impl_->last_error =
      "PocketSphinx reset requested while "
      "decoder was not initialized";

    return;
  }

  try {
    impl_->restart_utterance();
  } catch (const std::exception & exception) {
    impl_->last_error = exception.what();
    impl_->initialized = false;
  } catch (...) {
    impl_->last_error =
      "unknown PocketSphinx reset failure";

    impl_->initialized = false;
  }
}

PocketSphinxWakeWordBackendSnapshot
PocketSphinxWakeWordBackend::snapshot() const
{
  const std::scoped_lock lock{impl_->mutex};

  PocketSphinxWakeWordBackendSnapshot snapshot;

  snapshot.initialized = impl_->initialized;
  snapshot.stream_started = impl_->stream_started;
  snapshot.utterance_active = impl_->utterance_active;

  if (impl_->decoder != nullptr) {
    const char * active_search =
      ps_get_search(impl_->decoder);

    if (active_search != nullptr) {
      snapshot.active_search = active_search;
    }
  }

  snapshot.last_hypothesis =
    impl_->last_hypothesis;

  snapshot.last_error =
    impl_->last_error;

  snapshot.statistics =
    impl_->statistics;

  return snapshot;
}

}  // namespace savo_speech::wake_word
