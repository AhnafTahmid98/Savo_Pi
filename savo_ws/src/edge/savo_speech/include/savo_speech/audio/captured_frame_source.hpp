#ifndef SAVO_SPEECH__AUDIO__CAPTURED_FRAME_SOURCE_HPP_
#define SAVO_SPEECH__AUDIO__CAPTURED_FRAME_SOURCE_HPP_

#include <chrono>
#include <optional>

#include "savo_speech/audio/audio_frame.hpp"

namespace savo_speech::audio
{

class CapturedFrameSource
{
public:
  virtual ~CapturedFrameSource() = default;

  CapturedFrameSource(const CapturedFrameSource &) = delete;
  CapturedFrameSource & operator=(
    const CapturedFrameSource &) = delete;

  CapturedFrameSource(CapturedFrameSource &&) = delete;
  CapturedFrameSource & operator=(
    CapturedFrameSource &&) = delete;

  [[nodiscard]] virtual std::optional<AudioFrame>
  wait_captured_frame_for(
    std::chrono::milliseconds timeout) = 0;

protected:
  CapturedFrameSource() = default;
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__CAPTURED_FRAME_SOURCE_HPP_
