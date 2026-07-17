#include <filesystem>
#include <fstream>
#include <string>

#include "gtest/gtest.h"

#include "savo_speech/wake_word/wake_word_asset_resolver.hpp"

namespace
{

class WakeWordAssetResolverTest :
  public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ =
      std::filesystem::temp_directory_path() /
      "savo_speech_wake_word_asset_resolver_test";

    std::filesystem::remove_all(root_);

    asset_directory_ =
      root_ / "config" / "wake_word";

    std::filesystem::create_directories(
      asset_directory_);

    dictionary_ =
      asset_directory_ / "savo_wake.dict";

    default_keywords_ =
      asset_directory_ /
      "savo_keywords_default.kws";

    extended_keywords_ =
      asset_directory_ /
      "savo_keywords_extended.kws";

    create_file(dictionary_);
    create_file(default_keywords_);
    create_file(extended_keywords_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_);
  }

  static void create_file(
    const std::filesystem::path & path)
  {
    std::ofstream stream{path};
    stream << "test\n";
  }

  [[nodiscard]]
  savo_speech::wake_word::
  WakeWordAssetSelectionConfig
  make_config() const
  {
    savo_speech::wake_word::
    WakeWordAssetSelectionConfig config;

    config.package_share_directory = root_;
    config.profile = "default";

    return config;
  }

  std::filesystem::path root_;
  std::filesystem::path asset_directory_;

  std::filesystem::path dictionary_;
  std::filesystem::path default_keywords_;
  std::filesystem::path extended_keywords_;
};

}  // namespace

TEST(
  WakeWordAssetResolver,
  RecognizesSupportedProfiles)
{
  using savo_speech::wake_word::
    is_supported_wake_word_profile;

  EXPECT_TRUE(
    is_supported_wake_word_profile("default"));

  EXPECT_TRUE(
    is_supported_wake_word_profile("extended"));

  EXPECT_TRUE(
    is_supported_wake_word_profile("custom"));

  EXPECT_FALSE(
    is_supported_wake_word_profile("unknown"));
}

TEST_F(
  WakeWordAssetResolverTest,
  ResolvesDefaultPackagedAssets)
{
  const auto paths =
    savo_speech::wake_word::
    resolve_wake_word_assets(make_config());

  EXPECT_EQ(paths.dictionary_path, dictionary_);
  EXPECT_EQ(
    paths.keyword_file_path,
    default_keywords_);
}

TEST_F(
  WakeWordAssetResolverTest,
  ResolvesExtendedPackagedAssets)
{
  auto config = make_config();
  config.profile = "extended";

  const auto paths =
    savo_speech::wake_word::
    resolve_wake_word_assets(config);

  EXPECT_EQ(paths.dictionary_path, dictionary_);
  EXPECT_EQ(
    paths.keyword_file_path,
    extended_keywords_);
}

TEST_F(
  WakeWordAssetResolverTest,
  CustomProfileRequiresKeywordOverride)
{
  auto config = make_config();
  config.profile = "custom";

  EXPECT_FALSE(config.is_valid());

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::wake_word::
      resolve_wake_word_assets(config)),
    std::invalid_argument);
}

TEST_F(
  WakeWordAssetResolverTest,
  ExplicitOverridesDoNotRequirePackageAssets)
{
  const auto custom_dictionary =
    root_ / "custom.dict";

  const auto custom_keywords =
    root_ / "custom.kws";

  create_file(custom_dictionary);
  create_file(custom_keywords);

  savo_speech::wake_word::
  WakeWordAssetSelectionConfig config;

  config.profile = "custom";
  config.dictionary_override =
    custom_dictionary;

  config.keyword_file_override =
    custom_keywords;

  const auto paths =
    savo_speech::wake_word::
    resolve_wake_word_assets(config);

  EXPECT_EQ(
    paths.dictionary_path,
    custom_dictionary);

  EXPECT_EQ(
    paths.keyword_file_path,
    custom_keywords);
}

TEST_F(
  WakeWordAssetResolverTest,
  RejectsMissingResolvedAsset)
{
  std::filesystem::remove(default_keywords_);

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::wake_word::
      resolve_wake_word_assets(make_config())),
    std::invalid_argument);
}
