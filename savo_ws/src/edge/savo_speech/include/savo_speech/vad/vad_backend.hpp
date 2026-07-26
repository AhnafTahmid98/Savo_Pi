#ifndef SAVO_SPEECH__VAD__VAD_BACKEND_HPP_
#define SAVO_SPEECH__VAD__VAD_BACKEND_HPP_

#include "savo_speech/audio/audio_frame.hpp"

namespace savo_speech::vad
{

struct VadBackendResult
{
  // Normalized speech likelihood or binary decision:
  //   0.0 = silence
  //   1.0 = speech
  double speech_score{0.0};
};

class VadBackend
{
public:
  virtual ~VadBackend() = default;

  VadBackend(const VadBackend &) = delete;
  VadBackend & operator=(const VadBackend &) = delete;

  VadBackend(VadBackend &&) = delete;
  VadBackend & operator=(VadBackend &&) = delete;

  [[nodiscard]] virtual VadBackendResult analyze(
    const audio::AudioFrame & frame) = 0;

  virtual void reset() noexcept = 0;

protected:
  VadBackend() = default;
};

}  // namespace savo_speech::vad

#endif  // SAVO_SPEECH__VAD__VAD_BACKEND_HPP_
