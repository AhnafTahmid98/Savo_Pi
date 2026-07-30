// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "savo_bridge/snapshot_writer.hpp"


namespace
{

using Duration = savo_bridge::TopicObservation::Duration;
using Snapshot = savo_bridge::TopicObservation::Snapshot;

class TemporaryDirectory final
{
public:
  TemporaryDirectory()
  {
    static std::atomic<std::uint64_t> counter{0U};

    const auto time_identifier =
      std::chrono::steady_clock::now()
      .time_since_epoch()
      .count();

    for (std::size_t attempt = 0U; attempt < 128U; ++attempt) {
      const std::uint64_t sequence =
        counter.fetch_add(1U, std::memory_order_relaxed);

      path_ = std::filesystem::temp_directory_path() /
        (
        "savo_bridge_snapshot_writer_" +
        std::to_string(time_identifier) +
        "_" +
        std::to_string(sequence));

      std::error_code error;

      if (std::filesystem::create_directory(path_, error)) {
        return;
      }

      if (error &&
        error != std::errc::file_exists)
      {
        throw std::system_error(error);
      }
    }

    throw std::runtime_error(
            "Unable to create temporary test directory");
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory & operator=(const TemporaryDirectory &) = delete;

  [[nodiscard]] const std::filesystem::path &
  path() const noexcept
  {
    return path_;
  }

private:
  std::filesystem::path path_;
};

[[nodiscard]] Duration seconds_duration(const int value)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::seconds(value));
}

[[nodiscard]] Duration milliseconds_duration(const int value)
{
  return std::chrono::duration_cast<Duration>(
    std::chrono::milliseconds(value));
}

[[nodiscard]] Snapshot make_snapshot(
  std::string topic_name,
  const savo_bridge::TopicRequirement requirement,
  const savo_bridge::TopicClassification classification)
{
  Snapshot snapshot;
  snapshot.topic_name = std::move(topic_name);
  snapshot.requirement = requirement;
  snapshot.classification = classification;
  snapshot.stale_after = seconds_duration(1);

  if (classification ==
    savo_bridge::TopicClassification::kNeverObserved)
  {
    return snapshot;
  }

  snapshot.observed = true;
  snapshot.accepted_observations = 1U;

  if (classification ==
    savo_bridge::TopicClassification::kFresh)
  {
    snapshot.age = milliseconds_duration(500);
  }

  if (classification ==
    savo_bridge::TopicClassification::kStale)
  {
    snapshot.age = seconds_duration(1);
  }

  return snapshot;
}

[[nodiscard]] std::string read_file(
  const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);

  if (!stream) {
    throw std::runtime_error("Unable to open test snapshot");
  }

  return std::string(
    std::istreambuf_iterator<char>(stream),
    std::istreambuf_iterator<char>());
}

[[nodiscard]] std::size_t temporary_file_count(
  const std::filesystem::path & directory,
  const std::string & target_filename)
{
  const std::string prefix =
    "." + target_filename + ".tmp.";

  std::size_t count = 0U;

  for (const auto & entry :
    std::filesystem::directory_iterator(directory))
  {
    const std::string filename =
      entry.path().filename().string();

    if (filename.rfind(prefix, 0U) == 0U) {
      ++count;
    }
  }

  return count;
}

TEST(SavoBridgeSnapshotWriter, CanonicalAtomicPublicationContract)
{
  const Snapshot required_fresh = make_snapshot(
    "/z_required",
    savo_bridge::TopicRequirement::kRequired,
    savo_bridge::TopicClassification::kFresh);

  const Snapshot optional_stale = make_snapshot(
    "/a_optional",
    savo_bridge::TopicRequirement::kOptional,
    savo_bridge::TopicClassification::kStale);

  savo_bridge::SnapshotDocument document;
  document.sequence = 42U;
  document.topics = {required_fresh, optional_stale};

  savo_bridge::SnapshotDocument reordered = document;
  reordered.topics = {optional_stale, required_fresh};

  const std::string serialized =
    savo_bridge::serialize_snapshot(document);

  EXPECT_EQ(
    serialized,
    savo_bridge::serialize_snapshot(reordered));

  const nlohmann::json parsed =
    nlohmann::json::parse(serialized);

  EXPECT_EQ(parsed.at("schema_name"), "savo_bridge_snapshot");
  EXPECT_EQ(parsed.at("schema_version"), 2);
  EXPECT_EQ(parsed.at("snapshot_sequence"), 42);

  const auto & bridge = parsed.at("bridge");
  EXPECT_EQ(bridge.at("owner"), "savo_bridge");
  EXPECT_TRUE(bridge.at("read_only").get<bool>());
  EXPECT_FALSE(bridge.at("commands_enabled").get<bool>());
  EXPECT_FALSE(bridge.at("bridge_ready").get<bool>());
  EXPECT_TRUE(bridge.at("block_navigation").get<bool>());
  EXPECT_TRUE(bridge.contains("dispatcher"));

  const auto & health = parsed.at("health");
  EXPECT_EQ(health.at("status"), "degraded");
  EXPECT_EQ(
    health.at("reason"),
    "optional_topic_unavailable");
  EXPECT_TRUE(health.at("required_topics_ready").get<bool>());
  EXPECT_FALSE(health.at("all_topics_fresh").get<bool>());

  const auto & topics = parsed.at("topics");
  ASSERT_EQ(topics.size(), 2U);
  EXPECT_EQ(topics.at(0).at("topic_name"), "/a_optional");
  EXPECT_EQ(topics.at(1).at("topic_name"), "/z_required");

  savo_bridge::SnapshotDocument empty_document;
  empty_document.sequence = 1U;

  const std::string empty_serialized =
    savo_bridge::serialize_snapshot(empty_document);

  EXPECT_NE(
    empty_serialized.find("\"status\":\"unknown\""),
    std::string::npos);

  EXPECT_NE(
    empty_serialized.find("\"reason\":\"no_topics\""),
    std::string::npos);

  savo_bridge::SnapshotDocument duplicate_document = document;
  duplicate_document.topics = {
    required_fresh,
    required_fresh,
  };

  EXPECT_THROW(
    savo_bridge::serialize_snapshot(duplicate_document),
    std::invalid_argument);

  EXPECT_THROW(
    (void)savo_bridge::AtomicSnapshotWriter(
      std::filesystem::path{}),
    std::invalid_argument);

  EXPECT_THROW(
    (void)savo_bridge::AtomicSnapshotWriter(
      std::filesystem::path(".")),
    std::invalid_argument);

  const TemporaryDirectory temporary_directory;

  const std::filesystem::path target =
    temporary_directory.path() / "snapshot.json";

  const savo_bridge::AtomicSnapshotWriter writer(target);

  EXPECT_EQ(writer.target_path(), target);

  const auto first_result = writer.write(document);

  EXPECT_FALSE(first_result.replaced_existing);
  EXPECT_EQ(first_result.bytes_written, serialized.size());
  EXPECT_EQ(read_file(target), serialized);

  EXPECT_EQ(
    temporary_file_count(
      temporary_directory.path(),
      target.filename().string()),
    0U);

  document.sequence = 43U;

  const std::string replacement =
    savo_bridge::serialize_snapshot(document);

  const auto second_result = writer.write(document);

  EXPECT_TRUE(second_result.replaced_existing);
  EXPECT_EQ(second_result.bytes_written, replacement.size());
  EXPECT_EQ(read_file(target), replacement);

  EXPECT_EQ(
    temporary_file_count(
      temporary_directory.path(),
      target.filename().string()),
    0U);

  const std::string preserved = read_file(target);

  EXPECT_THROW(
    (void)writer.write(duplicate_document),
    std::invalid_argument);

  EXPECT_EQ(read_file(target), preserved);

  EXPECT_EQ(
    temporary_file_count(
      temporary_directory.path(),
      target.filename().string()),
    0U);

  const std::filesystem::path missing_parent_target =
    temporary_directory.path() /
    "missing" /
    "snapshot.json";

  const savo_bridge::AtomicSnapshotWriter missing_parent_writer(
    missing_parent_target);

  EXPECT_THROW(
    (void)missing_parent_writer.write(document),
    std::system_error);

  EXPECT_FALSE(
    std::filesystem::exists(missing_parent_target));

  const std::filesystem::path directory_target =
    temporary_directory.path() / "directory_target";

  ASSERT_TRUE(
    std::filesystem::create_directory(directory_target));

  const savo_bridge::AtomicSnapshotWriter directory_writer(
    directory_target);

  EXPECT_THROW(
    (void)directory_writer.write(document),
    std::invalid_argument);

  EXPECT_TRUE(std::filesystem::is_directory(directory_target));
}

}  // namespace
