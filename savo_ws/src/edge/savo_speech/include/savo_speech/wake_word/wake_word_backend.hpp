#ifndef SAVO_SPEECH__WAKE_WORD__WAKE_WORD_BACKEND_HPP_
#define SAVO_SPEECH__WAKE_WORD__WAKE_WORD_BACKEND_HPP_

#include <string>

#include "savo_speech/audio/audio_frame.hpp"

namespace savo_speech::wake_word
{

struct WakeWordBackendResult
{
  bool detected{false};

  std::string phrase{};
  double confidence{0.0};
};

class WakeWordBackend
{
public:
  virtual ~WakeWordBackend() = default;

  WakeWordBackend(const WakeWordBackend &) = delete;
  WakeWordBackend & operator=(
    const WakeWordBackend &) = delete;

  WakeWordBackend(WakeWordBackend &&) = delete;
  WakeWordBackend & operator=(
    WakeWordBackend &&) = delete;

  [[nodiscard]] virtual WakeWordBackendResult analyze(
    const audio::AudioFrame & frame) = 0;

  virtual void reset() noexcept = 0;

protected:
  WakeWordBackend() = default;
};

}  // namespace savo_speech::wake_word

#endif  // SAVO_SPEECH__WAKE_WORD__WAKE_WORD_BACKEND_HPP_
