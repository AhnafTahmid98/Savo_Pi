#ifndef SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_SOURCE_HPP_
#define SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_SOURCE_HPP_

#include <chrono>
#include <optional>

#include "savo_speech/session/completed_utterance.hpp"

namespace savo_speech::session
{

class CompletedUtteranceSource
{
public:
  virtual ~CompletedUtteranceSource() = default;

  [[nodiscard]] virtual std::optional<CompletedUtterance>
  wait_completed_for(
    std::chrono::milliseconds timeout) = 0;
};

}  // namespace savo_speech::session

#endif  // SAVO_SPEECH__SESSION__COMPLETED_UTTERANCE_SOURCE_HPP_
