#ifndef SAVO_SPEECH__WAKE_WORD__POCKETSPHINX_WAKE_WORD_BACKEND_HPP_
#define SAVO_SPEECH__WAKE_WORD__POCKETSPHINX_WAKE_WORD_BACKEND_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "savo_speech/wake_word/wake_word_backend.hpp"

namespace savo_speech::wake_word
{

struct PocketSphinxWakeWordBackendConfig
{
  std::filesystem::path acoustic_model_path{};
  std::filesystem::path dictionary_path{};
  std::filesystem::path keyword_file_path{};

  std::string search_name{"savo_wake_words"};

  std::uint32_t sample_rate_hz{16000U};

  bool suppress_decoder_log{true};

  [[nodiscard]] bool is_valid() const noexcept;
};

struct PocketSphinxWakeWordBackendStatistics
{
  std::uint64_t frames_analyzed{0U};
  std::uint64_t samples_processed{0U};

  std::uint64_t detections{0U};
  std::uint64_t decoder_restarts{0U};
  std::uint64_t reset_requests{0U};

  std::uint64_t process_failures{0U};
  std::uint64_t restart_failures{0U};

  std::uint64_t last_frame_sequence{0U};

  std::int32_t last_best_score{0};
};

struct PocketSphinxWakeWordBackendSnapshot
{
  bool initialized{false};
  bool stream_started{false};
  bool utterance_active{false};

  std::string active_search{};
  std::string last_hypothesis{};
  std::string last_error{};

  PocketSphinxWakeWordBackendStatistics statistics{};
};

class PocketSphinxWakeWordBackend final :
  public WakeWordBackend
{
public:
  explicit PocketSphinxWakeWordBackend(
    PocketSphinxWakeWordBackendConfig config);

  ~PocketSphinxWakeWordBackend() override;

  PocketSphinxWakeWordBackend(
    const PocketSphinxWakeWordBackend &) = delete;

  PocketSphinxWakeWordBackend & operator=(
    const PocketSphinxWakeWordBackend &) = delete;

  PocketSphinxWakeWordBackend(
    PocketSphinxWakeWordBackend &&) = delete;

  PocketSphinxWakeWordBackend & operator=(
    PocketSphinxWakeWordBackend &&) = delete;

  [[nodiscard]] WakeWordBackendResult analyze(
    const audio::AudioFrame & frame) override;

  void reset() noexcept override;

  [[nodiscard]] PocketSphinxWakeWordBackendSnapshot
  snapshot() const;

private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace savo_speech::wake_word

#endif  // SAVO_SPEECH__WAKE_WORD__POCKETSPHINX_WAKE_WORD_BACKEND_HPP_
