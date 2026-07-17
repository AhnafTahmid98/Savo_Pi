#ifndef SAVO_SPEECH__AUDIO__CAPTURED_AUDIO_PROCESSOR_CHAIN_HPP_
#define SAVO_SPEECH__AUDIO__CAPTURED_AUDIO_PROCESSOR_CHAIN_HPP_

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "savo_speech/audio/captured_audio_processor.hpp"

namespace savo_speech::audio
{

struct CapturedAudioProcessorStatistics
{
  std::string name{};
  bool required{true};

  std::uint64_t invocations{0U};
  std::uint64_t successes{0U};
  std::uint64_t failures{0U};

  std::uint64_t last_sequence{0U};

  std::string last_error{};
};

struct CapturedAudioProcessorChainStatistics
{
  std::uint64_t frames_received{0U};
  std::uint64_t frames_completed{0U};
  std::uint64_t frames_failed{0U};

  std::uint64_t frames_with_optional_failures{0U};

  std::uint64_t processor_invocations{0U};
  std::uint64_t processor_successes{0U};
  std::uint64_t processor_failures{0U};

  std::uint64_t required_processor_failures{0U};
  std::uint64_t optional_processor_failures{0U};

  std::uint64_t last_sequence{0U};
};

struct CapturedAudioProcessorChainSnapshot
{
  bool sealed{false};
  std::size_t processor_count{0U};

  CapturedAudioProcessorChainStatistics statistics{};

  std::vector<CapturedAudioProcessorStatistics>
    processors{};

  std::string last_error{};
};

class CapturedAudioProcessorChain final :
  public CapturedAudioProcessor
{
public:
  CapturedAudioProcessorChain() = default;

  ~CapturedAudioProcessorChain() override = default;

  CapturedAudioProcessorChain(
    const CapturedAudioProcessorChain &) = delete;

  CapturedAudioProcessorChain & operator=(
    const CapturedAudioProcessorChain &) = delete;

  CapturedAudioProcessorChain(
    CapturedAudioProcessorChain &&) = delete;

  CapturedAudioProcessorChain & operator=(
    CapturedAudioProcessorChain &&) = delete;

  void add_processor(
    std::string name,
    CapturedAudioProcessor & processor,
    bool required = true);

  [[nodiscard]] bool seal();

  [[nodiscard]] bool sealed() const noexcept;
  [[nodiscard]] std::size_t processor_count() const noexcept;

  void process(const AudioFrame & frame) override;

  [[nodiscard]] CapturedAudioProcessorChainSnapshot
  snapshot() const;

  void reset_statistics() noexcept;

private:
  struct ProcessorEntry
  {
    std::string name{};
    CapturedAudioProcessor * processor{nullptr};
    bool required{true};

    CapturedAudioProcessorStatistics statistics{};
  };

  void record_processor_success(
    std::size_t index,
    std::uint64_t sequence) noexcept;

  [[nodiscard]] bool record_processor_failure(
    std::size_t index,
    std::uint64_t sequence,
    const std::string & error) noexcept;

  mutable std::mutex mutex_;

  std::vector<ProcessorEntry> processors_{};

  bool sealed_{false};

  CapturedAudioProcessorChainStatistics statistics_{};

  std::string last_error_{};
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__CAPTURED_AUDIO_PROCESSOR_CHAIN_HPP_
