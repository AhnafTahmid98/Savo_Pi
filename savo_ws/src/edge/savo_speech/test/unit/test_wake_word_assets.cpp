#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "savo_speech/wake_word/pocketsphinx_wake_word_backend.hpp"

#ifndef SAVO_SPEECH_SOURCE_DIR
#error "SAVO_SPEECH_SOURCE_DIR must be defined"
#endif

namespace
{

struct KeywordEntry
{
  std::string phrase;
  double threshold{0.0};
};

[[nodiscard]] std::string trim(std::string value)
{
  const auto is_not_space = [](const unsigned char character) {
      return std::isspace(character) == 0;
    };

  value.erase(
    value.begin(),
    std::find_if(
      value.begin(),
      value.end(),
      is_not_space));

  value.erase(
    std::find_if(
      value.rbegin(),
      value.rend(),
      is_not_space).base(),
    value.end());

  return value;
}

[[nodiscard]] const std::filesystem::path & package_root()
{
  static const std::filesystem::path path{
    SAVO_SPEECH_SOURCE_DIR};

  return path;
}

[[nodiscard]] std::unordered_set<std::string>
load_dictionary(const std::filesystem::path & path)
{
  std::ifstream stream{path};

  if (!stream.is_open()) {
    throw std::runtime_error{
            "failed to open dictionary: " +
            path.string()};
  }

  std::unordered_set<std::string> words;
  std::string line;

  while (std::getline(stream, line)) {
    line = trim(std::move(line));

    if (
      line.empty() ||
      line.front() == '#')
    {
      continue;
    }

    std::istringstream parser{line};

    std::string word;
    parser >> word;

    const auto alternative_marker =
      word.find('(');

    if (alternative_marker != std::string::npos) {
      word.erase(alternative_marker);
    }

    words.insert(std::move(word));
  }

  return words;
}

[[nodiscard]] std::vector<KeywordEntry>
load_keyword_file(const std::filesystem::path & path)
{
  std::ifstream stream{path};

  if (!stream.is_open()) {
    throw std::runtime_error{
            "failed to open keyword file: " +
            path.string()};
  }

  std::vector<KeywordEntry> entries;
  std::string line;

  while (std::getline(stream, line)) {
    line = trim(std::move(line));

    if (
      line.empty() ||
      line.front() == '#')
    {
      continue;
    }

    const auto threshold_marker =
      line.rfind(" /");

    if (
      threshold_marker == std::string::npos ||
      line.back() != '/')
    {
      throw std::runtime_error{
              "invalid keyword line: " + line};
    }

    KeywordEntry entry;

    entry.phrase =
      trim(line.substr(0U, threshold_marker));

    const std::size_t threshold_begin =
      threshold_marker + 2U;

    const std::size_t threshold_length =
      line.size() - threshold_begin - 1U;

    entry.threshold = std::stod(
      line.substr(
        threshold_begin,
        threshold_length));

    if (
      entry.phrase.empty() ||
      entry.threshold <= 0.0 ||
      entry.threshold >= 1.0)
    {
      throw std::runtime_error{
              "invalid keyword entry: " + line};
    }

    entries.push_back(std::move(entry));
  }

  return entries;
}

void expect_all_words_in_dictionary(
  const std::vector<KeywordEntry> & entries,
  const std::unordered_set<std::string> & dictionary)
{
  for (const auto & entry : entries) {
    std::istringstream phrase_stream{entry.phrase};
    std::string word;

    while (phrase_stream >> word) {
      EXPECT_TRUE(dictionary.contains(word))
        << "Missing dictionary word '" << word
        << "' for phrase '" << entry.phrase << "'";
    }
  }
}

[[nodiscard]] bool contains_phrase(
  const std::vector<KeywordEntry> & entries,
  const std::string & phrase)
{
  return std::any_of(
    entries.begin(),
    entries.end(),
    [&phrase](const KeywordEntry & entry) {
      return entry.phrase == phrase;
    });
}

[[nodiscard]] std::optional<double> find_threshold(
  const std::vector<KeywordEntry> & entries,
  const std::string & phrase)
{
  const auto iterator = std::find_if(
    entries.begin(),
    entries.end(),
    [&phrase](const KeywordEntry & entry) {
      return entry.phrase == phrase;
    });

  if (iterator == entries.end()) {
    return std::nullopt;
  }

  return iterator->threshold;
}

[[nodiscard]] const std::filesystem::path &
dictionary_path()
{
  static const auto path =
    package_root() /
    "config/wake_word/savo_wake.dict";

  return path;
}

[[nodiscard]] std::filesystem::path default_keywords_path()
{
  return
    package_root() /
    "config/wake_word/savo_keywords_default.kws";
}

[[nodiscard]] std::filesystem::path extended_keywords_path()
{
  return
    package_root() /
    "config/wake_word/savo_keywords_extended.kws";
}

}  // namespace

TEST(
  WakeWordAssets,
  DictionaryContainsRequiredWords)
{
  const auto dictionary =
    load_dictionary(dictionary_path());

  const std::vector<std::string> required_words{
    "hey",
    "hi",
    "hello",
    "hei",
    "moi",
    "okay",
    "ok",
    "yo",
    "wake",
    "up",
    "robot",
    "savo",
    "sabo",
    "robo"};

  for (const auto & word : required_words) {
    EXPECT_TRUE(dictionary.contains(word))
      << "Missing required dictionary word: " << word;
  }
}

TEST(
  WakeWordAssets,
  DefaultKeywordFileIsValid)
{
  const auto dictionary =
    load_dictionary(dictionary_path());

  const auto entries =
    load_keyword_file(default_keywords_path());

  ASSERT_FALSE(entries.empty());

  expect_all_words_in_dictionary(
    entries,
    dictionary);

  std::unordered_set<std::string> unique_phrases;

  for (const auto & entry : entries) {
    EXPECT_TRUE(
      unique_phrases.insert(entry.phrase).second)
      << "Duplicate phrase: " << entry.phrase;
  }
}

TEST(
  WakeWordAssets,
  ExtendedKeywordFileIsValid)
{
  const auto dictionary =
    load_dictionary(dictionary_path());

  const auto entries =
    load_keyword_file(extended_keywords_path());

  ASSERT_FALSE(entries.empty());

  expect_all_words_in_dictionary(
    entries,
    dictionary);

  std::unordered_set<std::string> unique_phrases;

  for (const auto & entry : entries) {
    EXPECT_TRUE(
      unique_phrases.insert(entry.phrase).second)
      << "Duplicate phrase: " << entry.phrase;
  }
}

TEST(
  WakeWordAssets,
  DefaultContainsLockedWakePhrases)
{
  const auto entries =
    load_keyword_file(default_keywords_path());

  const std::vector<std::string> required_phrases{
    "hey savo",
    "hi savo",
    "hei savo",
    "moi savo",
    "robot savo",
    "savo robot",
    "savo"};

  for (const auto & phrase : required_phrases) {
    EXPECT_TRUE(contains_phrase(entries, phrase))
      << "Missing default phrase: " << phrase;
  }
}

TEST(
  WakeWordAssets,
  ExtendedContainsRequestedAliases)
{
  const auto entries =
    load_keyword_file(extended_keywords_path());

  const std::vector<std::string> required_aliases{
    "hey sabo",
    "hi sabo",
    "robot sabo",
    "sabo robot",
    "hei sabo",
    "moi sabo",
    "hey robo",
    "hi robo",
    "hei robo",
    "moi robo",
    "sabo",
    "robo"};

  for (const auto & phrase : required_aliases) {
    EXPECT_TRUE(contains_phrase(entries, phrase))
      << "Missing extended alias: " << phrase;
  }
}

TEST(
  WakeWordAssets,
  ShortAliasesUseStricterInitialThreshold)
{
  const auto entries =
    load_keyword_file(extended_keywords_path());

  const auto primary =
    find_threshold(entries, "hey savo");

  ASSERT_TRUE(primary.has_value());

  for (const std::string phrase :
    {"savo", "sabo", "robo"})
  {
    const auto threshold =
      find_threshold(entries, phrase);

    ASSERT_TRUE(threshold.has_value())
      << "Missing short alias: " << phrase;

    EXPECT_GT(threshold.value(), primary.value())
      << "Short alias is not stricter: " << phrase;
  }
}

TEST(
  WakeWordAssets,
  DecoderInitializesWithPackagedAssets)
{
  const std::filesystem::path acoustic_model{
    "/usr/share/pocketsphinx/model/en-us/en-us"};

  ASSERT_TRUE(
    std::filesystem::is_directory(acoustic_model));

  savo_speech::wake_word::
  PocketSphinxWakeWordBackendConfig config;

  config.acoustic_model_path =
    acoustic_model;

  config.dictionary_path =
    dictionary_path();

  config.keyword_file_path =
    default_keywords_path();

  config.search_name =
    "savo_asset_validation";

  config.sample_rate_hz = 16000U;
  config.suppress_decoder_log = true;

  savo_speech::wake_word::
  PocketSphinxWakeWordBackend backend{
    std::move(config)};

  const auto snapshot = backend.snapshot();

  EXPECT_TRUE(snapshot.initialized);
  EXPECT_TRUE(snapshot.stream_started);
  EXPECT_TRUE(snapshot.utterance_active);
  EXPECT_EQ(
    snapshot.active_search,
    "savo_asset_validation");
  EXPECT_TRUE(snapshot.last_error.empty());
}
