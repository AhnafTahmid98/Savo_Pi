#include "savo_speech/wake_word/wake_word_asset_resolver.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>

namespace savo_speech::wake_word
{

namespace
{

[[nodiscard]] std::filesystem::path packaged_asset(
  const WakeWordAssetSelectionConfig & config,
  const std::string_view filename)
{
  if (config.package_share_directory.empty()) {
    throw std::invalid_argument{
            "package share directory is required for "
            "packaged wake-word assets"};
  }

  if (
    !std::filesystem::is_directory(
      config.package_share_directory))
  {
    throw std::invalid_argument{
            "package share directory does not exist: " +
            config.package_share_directory.string()};
  }

  return
    config.package_share_directory /
    "config" /
    "wake_word" /
    filename;
}

void validate_regular_file(
  const std::filesystem::path & path,
  const std::string_view asset_name)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::invalid_argument{
            std::string{asset_name} +
            " does not exist: " +
            path.string()};
  }
}

}  // namespace

bool is_supported_wake_word_profile(
  const std::string_view profile) noexcept
{
  return
    profile == "default" ||
    profile == "extended" ||
    profile == "custom";
}

bool WakeWordAssetSelectionConfig::is_valid()
const noexcept
{
  if (!is_supported_wake_word_profile(profile)) {
    return false;
  }

  if (
    profile == "custom" &&
    keyword_file_override.empty())
  {
    return false;
  }

  return true;
}

WakeWordAssetPaths resolve_wake_word_assets(
  const WakeWordAssetSelectionConfig & config)
{
  if (!config.is_valid()) {
    throw std::invalid_argument{
            "invalid wake-word asset selection configuration"};
  }

  WakeWordAssetPaths paths;

  if (!config.dictionary_override.empty()) {
    paths.dictionary_path =
      config.dictionary_override;
  } else {
    paths.dictionary_path =
      packaged_asset(
      config,
      "savo_wake.dict");
  }

  if (!config.keyword_file_override.empty()) {
    paths.keyword_file_path =
      config.keyword_file_override;
  } else if (config.profile == "default") {
    paths.keyword_file_path =
      packaged_asset(
      config,
      "savo_keywords_default.kws");
  } else if (config.profile == "extended") {
    paths.keyword_file_path =
      packaged_asset(
      config,
      "savo_keywords_extended.kws");
  } else {
    throw std::invalid_argument{
            "custom wake-word profile requires an "
            "explicit keyword file"};
  }

  validate_regular_file(
    paths.dictionary_path,
    "wake-word dictionary");

  validate_regular_file(
    paths.keyword_file_path,
    "wake-word keyword file");

  return paths;
}

}  // namespace savo_speech::wake_word
