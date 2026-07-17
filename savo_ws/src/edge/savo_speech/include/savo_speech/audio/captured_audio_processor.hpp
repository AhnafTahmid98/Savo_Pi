#ifndef SAVO_SPEECH__AUDIO__CAPTURED_AUDIO_PROCESSOR_HPP_
#define SAVO_SPEECH__AUDIO__CAPTURED_AUDIO_PROCESSOR_HPP_

#include "savo_speech/audio/audio_frame.hpp"

namespace savo_speech::audio
{

class CapturedAudioProcessor
{
public:
  virtual ~CapturedAudioProcessor() = default;

  CapturedAudioProcessor(
    const CapturedAudioProcessor &) = delete;

  CapturedAudioProcessor & operator=(
    const CapturedAudioProcessor &) = delete;

  CapturedAudioProcessor(
    CapturedAudioProcessor &&) = delete;

  CapturedAudioProcessor & operator=(
    CapturedAudioProcessor &&) = delete;

  virtual void process(const AudioFrame & frame) = 0;

protected:
  CapturedAudioProcessor() = default;
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__CAPTURED_AUDIO_PROCESSOR_HPP_
