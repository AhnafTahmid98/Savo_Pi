#include <filesystem>
#include <fstream>
#include <string>

#include "gtest/gtest.h"

#include "savo_speech/wake_word/pocketsphinx_wake_word_backend.hpp"

namespace
{

class PocketSphinxWakeWordBackendTest :
  public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ =
      std::filesystem::temp_directory_path() /
      "savo_speech_pocketsphinx_backend_test";

    std::filesystem::remove_all(root_);
    std::filesystem::create_directories(root_);

    model_directory_ = root_ / "model";
    dictionary_file_ = root_ / "dictionary.dict";
    keyword_file_ = root_ / "keywords.list";
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_);
  }

  [[nodiscard]]
  savo_speech::wake_word::
  PocketSphinxWakeWordBackendConfig
  make_config() const
  {
    savo_speech::wake_word::
    PocketSphinxWakeWordBackendConfig config;

    config.acoustic_model_path =
      model_directory_;

    config.dictionary_path =
      dictionary_file_;

    config.keyword_file_path =
      keyword_file_;

    config.search_name =
      "savo_wake_words";

    config.sample_rate_hz = 16000U;

    return config;
  }

  static void create_file(
    const std::filesystem::path & path,
    const std::string & contents = "test\n")
  {
    std::ofstream stream{path};
    stream << contents;
  }

  std::filesystem::path root_;
  std::filesystem::path model_directory_;
  std::filesystem::path dictionary_file_;
  std::filesystem::path keyword_file_;
};

}  // namespace

TEST_F(
  PocketSphinxWakeWordBackendTest,
  AcceptsStructurallyValidConfiguration)
{
  const auto config = make_config();

  EXPECT_TRUE(config.is_valid());
}

TEST_F(
  PocketSphinxWakeWordBackendTest,
  RejectsEmptySearchName)
{
  auto config = make_config();
  config.search_name.clear();

  EXPECT_FALSE(config.is_valid());
}

TEST_F(
  PocketSphinxWakeWordBackendTest,
  RejectsUnsupportedSampleRate)
{
  auto config = make_config();
  config.sample_rate_hz = 48000U;

  EXPECT_FALSE(config.is_valid());
}

TEST_F(
  PocketSphinxWakeWordBackendTest,
  RejectsMissingAcousticModelDirectory)
{
  create_file(dictionary_file_);
  create_file(keyword_file_);

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::wake_word::
      PocketSphinxWakeWordBackend{
        make_config()}),
    std::invalid_argument);
}

TEST_F(
  PocketSphinxWakeWordBackendTest,
  RejectsMissingDictionaryFile)
{
  std::filesystem::create_directories(
    model_directory_);

  create_file(keyword_file_);

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::wake_word::
      PocketSphinxWakeWordBackend{
        make_config()}),
    std::invalid_argument);
}

TEST_F(
  PocketSphinxWakeWordBackendTest,
  RejectsMissingKeywordFile)
{
  std::filesystem::create_directories(
    model_directory_);

  create_file(dictionary_file_);

  EXPECT_THROW(
    static_cast<void>(
      savo_speech::wake_word::
      PocketSphinxWakeWordBackend{
        make_config()}),
    std::invalid_argument);
}
