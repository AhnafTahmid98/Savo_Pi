#include "savo_speech/audio/captured_audio_processor_chain.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace savo_speech::audio
{

void CapturedAudioProcessorChain::add_processor(
  std::string name,
  CapturedAudioProcessor & processor,
  const bool required)
{
  if (name.empty()) {
    throw std::invalid_argument{
            "captured-audio processor name must not be empty"};
  }

  const std::scoped_lock lock{mutex_};

  if (sealed_) {
    throw std::logic_error{
            "captured-audio processor chain is sealed"};
  }

  const auto duplicate = std::find_if(
    processors_.cbegin(),
    processors_.cend(),
    [&name](const ProcessorEntry & entry) {
      return entry.name == name;
    });

  if (duplicate != processors_.cend()) {
    throw std::invalid_argument{
            "duplicate captured-audio processor name: " +
            name};
  }

  ProcessorEntry entry;

  entry.name = std::move(name);
  entry.processor = &processor;
  entry.required = required;

  entry.statistics.name = entry.name;
  entry.statistics.required = required;

  processors_.push_back(std::move(entry));
}

bool CapturedAudioProcessorChain::seal()
{
  const std::scoped_lock lock{mutex_};

  if (sealed_) {
    return false;
  }

  if (processors_.empty()) {
    throw std::logic_error{
            "cannot seal an empty captured-audio "
            "processor chain"};
  }

  sealed_ = true;
  return true;
}

bool CapturedAudioProcessorChain::sealed() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return sealed_;
}

std::size_t
CapturedAudioProcessorChain::processor_count() const noexcept
{
  const std::scoped_lock lock{mutex_};
  return processors_.size();
}

void CapturedAudioProcessorChain::process(
  const AudioFrame & frame)
{
  if (!frame.is_consistent()) {
    throw std::invalid_argument{
            "captured-audio processor chain received "
            "an invalid frame"};
  }

  std::size_t processor_count_value{0U};

  {
    const std::scoped_lock lock{mutex_};

    if (!sealed_) {
      throw std::logic_error{
              "captured-audio processor chain is not sealed"};
    }

    processor_count_value = processors_.size();

    ++statistics_.frames_received;
    statistics_.last_sequence = frame.sequence;
  }

  bool optional_failure_occurred{false};

  for (
    std::size_t index = 0U;
    index < processor_count_value;
    ++index)
  {
    CapturedAudioProcessor * processor{nullptr};
    std::string processor_name;

    {
      const std::scoped_lock lock{mutex_};

      ProcessorEntry & entry = processors_.at(index);

      processor = entry.processor;
      processor_name = entry.name;

      ++entry.statistics.invocations;
      entry.statistics.last_sequence = frame.sequence;

      ++statistics_.processor_invocations;
    }

    if (processor == nullptr) {
      const std::string error =
        "captured-audio processor is null: " +
        processor_name;

      const bool required_failure =
        record_processor_failure(
        index,
        frame.sequence,
        error);

      if (required_failure) {
        throw std::runtime_error{error};
      }

      optional_failure_occurred = true;
      continue;
    }

    try {
      processor->process(frame);

      record_processor_success(
        index,
        frame.sequence);
    } catch (const std::exception & exception) {
      const std::string error =
        "captured-audio processor '" +
        processor_name +
        "' failed: " +
        exception.what();

      const bool required_failure =
        record_processor_failure(
        index,
        frame.sequence,
        error);

      if (required_failure) {
        throw std::runtime_error{error};
      }

      optional_failure_occurred = true;
    } catch (...) {
      const std::string error =
        "captured-audio processor '" +
        processor_name +
        "' failed with an unknown exception";

      const bool required_failure =
        record_processor_failure(
        index,
        frame.sequence,
        error);

      if (required_failure) {
        throw std::runtime_error{error};
      }

      optional_failure_occurred = true;
    }
  }

  {
    const std::scoped_lock lock{mutex_};

    ++statistics_.frames_completed;

    if (optional_failure_occurred) {
      ++statistics_.frames_with_optional_failures;
    }
  }
}

CapturedAudioProcessorChainSnapshot
CapturedAudioProcessorChain::snapshot() const
{
  const std::scoped_lock lock{mutex_};

  CapturedAudioProcessorChainSnapshot snapshot;

  snapshot.sealed = sealed_;
  snapshot.processor_count = processors_.size();
  snapshot.statistics = statistics_;
  snapshot.last_error = last_error_;

  snapshot.processors.reserve(processors_.size());

  for (const ProcessorEntry & entry : processors_) {
    snapshot.processors.push_back(entry.statistics);
  }

  return snapshot;
}

void CapturedAudioProcessorChain::reset_statistics() noexcept
{
  const std::scoped_lock lock{mutex_};

  statistics_ =
    CapturedAudioProcessorChainStatistics{};

  last_error_.clear();

  for (ProcessorEntry & entry : processors_) {
    entry.statistics =
      CapturedAudioProcessorStatistics{};

    entry.statistics.name = entry.name;
    entry.statistics.required = entry.required;
  }
}

void CapturedAudioProcessorChain::
record_processor_success(
  const std::size_t index,
  const std::uint64_t sequence) noexcept
{
  const std::scoped_lock lock{mutex_};

  ProcessorEntry & entry = processors_.at(index);

  ++entry.statistics.successes;
  entry.statistics.last_sequence = sequence;
  entry.statistics.last_error.clear();

  ++statistics_.processor_successes;
}

bool CapturedAudioProcessorChain::
record_processor_failure(
  const std::size_t index,
  const std::uint64_t sequence,
  const std::string & error) noexcept
{
  const std::scoped_lock lock{mutex_};

  ProcessorEntry & entry = processors_.at(index);

  ++entry.statistics.failures;
  entry.statistics.last_sequence = sequence;
  entry.statistics.last_error = error;

  ++statistics_.processor_failures;

  last_error_ = error;

  if (entry.required) {
    ++statistics_.required_processor_failures;
    ++statistics_.frames_failed;

    return true;
  }

  ++statistics_.optional_processor_failures;

  return false;
}

}  // namespace savo_speech::audio
