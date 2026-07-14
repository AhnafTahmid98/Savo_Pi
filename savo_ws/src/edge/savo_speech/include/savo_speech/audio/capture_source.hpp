#ifndef SAVO_SPEECH__AUDIO__CAPTURE_SOURCE_HPP_
#define SAVO_SPEECH__AUDIO__CAPTURE_SOURCE_HPP_

#include "savo_speech/audio/audio_format.hpp"
#include "savo_speech/audio/audio_frame.hpp"

namespace savo_speech::audio
{

class CaptureSource
{
public:
  virtual ~CaptureSource() = default;

  CaptureSource(const CaptureSource &) = delete;
  CaptureSource & operator=(const CaptureSource &) = delete;

  CaptureSource(CaptureSource &&) = delete;
  CaptureSource & operator=(CaptureSource &&) = delete;

  virtual void open() = 0;
  virtual void close() noexcept = 0;

  [[nodiscard]] virtual bool is_open() const noexcept = 0;

  [[nodiscard]] virtual AudioFormat format() const noexcept = 0;

  [[nodiscard]] virtual AudioFrame read_frame() = 0;

protected:
  CaptureSource() = default;
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__CAPTURE_SOURCE_HPP_
