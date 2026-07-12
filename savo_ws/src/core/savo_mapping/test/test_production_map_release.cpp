#include "savo_mapping/production_map_release.hpp"
#include "savo_mapping/saved_map_contract.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{

namespace fs = std::filesystem;

void write_text(
  const fs::path & path,
  const std::string & text)
{
  std::ofstream output{path};
  ASSERT_TRUE(output.is_open());

  output << text;
  output.close();
}

fs::path create_source_session(
  const fs::path & root,
  const std::string & map_id)
{
  const fs::path session =
    root / map_id;

  fs::create_directories(session);

  const fs::path yaml =
    session / (map_id + ".yaml");

  const fs::path image =
    session / (map_id + ".pgm");

  const fs::path posegraph =
    session / (map_id + ".posegraph");

  const fs::path data =
    session / (map_id + ".data");

  const fs::path quality =
    session / "quality_report.yaml";

  const fs::path manifest =
    session / "manifest.yaml";

  write_text(
    image,
    "P2\n2 2\n255\n0 255 205 254\n");

  write_text(
    yaml,
    "image: " + image.string() + "\n"
    "resolution: 0.05\n"
    "origin: [0.0, 0.0, 0.0]\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.196\n");

  write_text(
    posegraph,
    "posegraph");

  write_text(
    data,
    "dataset");

  write_text(
    quality,
    "schema_version: 1\n"
    "map_id: " + map_id + "\n"
    "frame_id: map\n"
    "passed: true\n"
    "reason: quality_passed\n");

  write_text(
    manifest,
    "schema_version: 1\n"
    "map_id: " + map_id + "\n"
    "frame_id: map\n"
    "created_unix_ns: 1\n"
    "session_directory: " +
      session.string() + "\n"
    "occupancy_grid:\n"
    "  yaml: " + yaml.string() + "\n"
    "  image: " + image.string() + "\n"
    "pose_graph:\n"
    "  posegraph: " +
      posegraph.string() + "\n"
    "  data: " + data.string() + "\n"
    "map_quality:\n"
    "  structurally_valid: true\n"
    "  evaluated: true\n"
    "  passed: true\n"
    "  report: " +
      quality.string() + "\n"
    "navigation_handoff_ready: true\n"
    "location_link:\n"
    "  map_id: " + map_id + "\n"
    "  registered: false\n"
    "navigation_handoff:\n"
    "  contract_version: 1\n"
    "  map_id: " + map_id + "\n"
    "  frame_id: map\n"
    "  approved: true\n"
    "  reason: operator_approved\n"
    "  map_yaml: " +
      yaml.string() + "\n"
    "  quality_report: " +
      quality.string() + "\n");

  return session;
}

}  // namespace

TEST(ProductionMapRelease, ValidReleaseIdentifiers)
{
  EXPECT_TRUE(
    savo_mapping::release::
    valid_release_id("savonia_main_v1"));

  EXPECT_TRUE(
    savo_mapping::release::
    valid_release_id("map-2026.07.11"));
}

TEST(ProductionMapRelease, InvalidReleaseIdentifiers)
{
  EXPECT_FALSE(
    savo_mapping::release::
    valid_release_id(""));

  EXPECT_FALSE(
    savo_mapping::release::
    valid_release_id("../release"));

  EXPECT_FALSE(
    savo_mapping::release::
    valid_release_id("/absolute"));

  EXPECT_FALSE(
    savo_mapping::release::
    valid_release_id("release name"));
}

TEST(ProductionMapRelease, CreateVerifyPromoteDeactivate)
{
  const fs::path root =
    fs::temp_directory_path() /
    (
      "savo_mapping_release_test_" +
      std::to_string(
        std::chrono::steady_clock::now()
        .time_since_epoch()
        .count()));

  const fs::path sessions =
    root / "sessions";

  const fs::path production =
    root / "production";

  const std::string map_id =
    "test_map";

  const fs::path session =
    create_source_session(
    sessions,
    map_id);

  const auto source =
    savo_mapping::session::
    verify_saved_map_session(
      session,
      map_id,
      "map");

  ASSERT_TRUE(source.valid)
    << source.reason;

  const auto created =
    savo_mapping::release::
    create_release(
      source,
      production,
      "test_map_v1",
      false);

  ASSERT_TRUE(created.valid)
    << created.reason;

  EXPECT_TRUE(
    fs::is_regular_file(
      created.release_manifest));

  EXPECT_TRUE(
    fs::is_regular_file(
      production / "catalog.yaml"));

  const auto verified =
    savo_mapping::release::
    verify_release(
      production,
      "test_map_v1");

  ASSERT_TRUE(verified.valid)
    << verified.reason;

  const auto active =
    savo_mapping::release::
    promote_release(
      production,
      "test_map_v1");

  EXPECT_TRUE(active.active);
  EXPECT_EQ(
    active.release_id,
    "test_map_v1");

  const auto deactivated =
    savo_mapping::release::
    deactivate_active_map(
      production);

  EXPECT_FALSE(deactivated.active);
  EXPECT_EQ(
    deactivated.reason,
    "operator_deactivated");

  fs::remove_all(root);
}
