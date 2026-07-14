#include "savo_speech/audio/channel_selector.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace savo_speech::audio
{

std::vector<std::int16_t> ChannelSelector::extract(
  const std::span<const std::int16_t> interleaved_samples,
  const std::uint16_t channel_count,
  const std::uint16_t channel_index)
{
  if (channel_count == 0U) {
    throw std::invalid_argument{
            "channel_count must be greater than zero"};
  }

  if (channel_index >= channel_count) {
    throw std::out_of_range{
            "channel_index must be less than channel_count"};
  }

  const std::size_t channels =
    static_cast<std::size_t>(channel_count);

  if (interleaved_samples.size() % channels != 0U) {
    throw std::invalid_argument{
            "interleaved sample count is not divisible by channel_count"};
  }

  const std::size_t output_sample_count =
    interleaved_samples.size() / channels;

  std::vector<std::int16_t> output;
  output.reserve(output_sample_count);

  for (
    std::size_t index =
      static_cast<std::size_t>(channel_index);
    index < interleaved_samples.size();
    index += channels)
  {
    output.push_back(interleaved_samples[index]);
  }

  return output;
}

}  // namespace savo_speech::audio
