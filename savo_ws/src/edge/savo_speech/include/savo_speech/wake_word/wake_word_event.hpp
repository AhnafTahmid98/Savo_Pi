#ifndef SAVO_SPEECH__WAKE_WORD__WAKE_WORD_EVENT_HPP_
#define SAVO_SPEECH__WAKE_WORD__WAKE_WORD_EVENT_HPP_

#include <cstdint>
#include <string>

#include "savo_speech/audio/audio_frame.hpp"

namespace savo_speech::wake_word
{

struct WakeWordEvent
{
  using Clock = audio::AudioFrame::Clock;

  std::uint64_t event_id{0U};
  std::uint64_t frame_sequence{0U};

  Clock::time_point detected_at{};

  std::string phrase{};
  double confidence{0.0};
};

}  // namespace savo_speech::wake_word

#endif  // SAVO_SPEECH__WAKE_WORD__WAKE_WORD_EVENT_HPP_
