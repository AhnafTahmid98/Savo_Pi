#ifndef SAVO_SPEECH__AUDIO__CHANNEL_SELECTOR_HPP_
#define SAVO_SPEECH__AUDIO__CHANNEL_SELECTOR_HPP_

#include <cstdint>
#include <span>
#include <vector>

namespace savo_speech::audio
{

class ChannelSelector final
{
public:
  ChannelSelector() = delete;

  [[nodiscard]] static std::vector<std::int16_t> extract(
    std::span<const std::int16_t> interleaved_samples,
    std::uint16_t channel_count,
    std::uint16_t channel_index);
};

}  // namespace savo_speech::audio

#endif  // SAVO_SPEECH__AUDIO__CHANNEL_SELECTOR_HPP_
