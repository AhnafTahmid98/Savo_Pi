#include "savo_mapping/saved_map_contract.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{

namespace fs = std::filesystem;

fs::path unique_root()
{
  return fs::temp_directory_path() /
         (
           "savo_saved_map_test_" +
           std::to_string(
             std::chrono::steady_clock::now()
             .time_since_epoch().count()));
}

void write_file(
  const fs::path & path,
  const std::string & content)
{
  std::ofstream output{
    path,
    std::ios::binary};

  output << content;
}

fs::path create_valid_session(
  const std::string & map_id)
{
  const fs::path root =
    unique_root();

  const fs::path session =
    root / map_id;

  fs::create_directories(session);

  const fs::path base =
    session / map_id;

  write_file(
    base.string() + ".pgm",
    "P5\n1 1\n255\nx");

  write_file(
    base.string() + ".posegraph",
    "posegraph");

  write_file(
    base.string() + ".data",
    "data");

  write_file(
    base.string() + ".yaml",
    "image: " + map_id + ".pgm\n"
    "mode: trinary\n"
    "resolution: 0.05\n"
    "origin: [0.0, 0.0, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n");

  write_file(
    session / "manifest.yaml",
    "schema_version: 1\n"
    "map_id: \"" + map_id + "\"\n"
    "frame_id: \"map\"\n"
    "session_directory: \"" +
      session.generic_string() + "\"\n"
    "occupancy_grid:\n"
    "  yaml: \"" +
      (base.string() + ".yaml") + "\"\n"
    "  image: \"" +
      (base.string() + ".pgm") + "\"\n"
    "pose_graph:\n"
    "  posegraph: \"" +
      (base.string() + ".posegraph") +
      "\"\n"
    "  data: \"" +
      (base.string() + ".data") + "\"\n"
    "map_quality:\n"
    "  structurally_valid: true\n"
    "  evaluated: false\n"
    "navigation_handoff_ready: false\n");

  return session;
}

}  // namespace

TEST(SavedMapContract, AcceptsCompleteSession)
{
  const fs::path session =
    create_valid_session(
      "campus_map");

  const auto result =
    savo_mapping::session::
    verify_saved_map_session(
      session,
      "campus_map",
      "map");

  EXPECT_TRUE(result.valid);
  EXPECT_EQ(
    result.reason,
    "saved_map_valid");

  EXPECT_GT(
    result.posegraph_bytes,
    0u);

  fs::remove_all(
    session.parent_path());
}

TEST(SavedMapContract, RejectsMissingData)
{
  const fs::path session =
    create_valid_session(
      "campus_map");

  fs::remove(
    session /
    "campus_map.data");

  const auto result =
    savo_mapping::session::
    verify_saved_map_session(
      session,
      "campus_map",
      "map");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.reason,
    "posegraph_data_missing_or_empty");

  fs::remove_all(
    session.parent_path());
}

TEST(SavedMapContract, RejectsMapIdMismatch)
{
  const fs::path session =
    create_valid_session(
      "campus_map");

  const auto result =
    savo_mapping::session::
    verify_saved_map_session(
      session,
      "different_map",
      "map");

  EXPECT_FALSE(result.valid);

  fs::remove_all(
    session.parent_path());
}

TEST(SavedMapContract, RejectsWrongFrame)
{
  const fs::path session =
    create_valid_session(
      "campus_map");

  const auto result =
    savo_mapping::session::
    verify_saved_map_session(
      session,
      "campus_map",
      "world");

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.reason,
    "manifest_frame_id_mismatch");

  fs::remove_all(
    session.parent_path());
}
