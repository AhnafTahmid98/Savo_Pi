#ifndef SAVO_SPEECH__WAKE_WORD__WAKE_WORD_ASSET_RESOLVER_HPP_
#define SAVO_SPEECH__WAKE_WORD__WAKE_WORD_ASSET_RESOLVER_HPP_

#include <filesystem>
#include <string>
#include <string_view>

namespace savo_speech::wake_word
{

struct WakeWordAssetSelectionConfig
{
  std::filesystem::path package_share_directory{};

  std::string profile{"default"};

  std::filesystem::path dictionary_override{};
  std::filesystem::path keyword_file_override{};

  [[nodiscard]] bool is_valid() const noexcept;
};

struct WakeWordAssetPaths
{
  std::filesystem::path dictionary_path{};
  std::filesystem::path keyword_file_path{};
};

[[nodiscard]] bool is_supported_wake_word_profile(
  std::string_view profile) noexcept;

[[nodiscard]] WakeWordAssetPaths resolve_wake_word_assets(
  const WakeWordAssetSelectionConfig & config);

}  // namespace savo_speech::wake_word

#endif  // SAVO_SPEECH__WAKE_WORD__WAKE_WORD_ASSET_RESOLVER_HPP_
