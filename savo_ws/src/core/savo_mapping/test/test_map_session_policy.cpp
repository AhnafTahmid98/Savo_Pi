#include "savo_mapping/map_session_policy.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{

namespace fs = std::filesystem;

fs::path unique_test_root()
{
  const auto value =
    std::chrono::steady_clock::now()
    .time_since_epoch().count();

  return fs::temp_directory_path() /
         (
           "savo_mapping_session_test_" +
           std::to_string(value));
}

}  // namespace

TEST(MapSessionPolicy, AcceptsStableMapIds)
{
  EXPECT_TRUE(
    savo_mapping::session::
    validate_map_id(
      "campus_floor_2").empty());

  EXPECT_TRUE(
    savo_mapping::session::
    validate_map_id(
      "a201_map").empty());
}

TEST(MapSessionPolicy, RejectsUnsafeMapIds)
{
  EXPECT_EQ(
    savo_mapping::session::
    validate_map_id(""),
    "map_id_is_empty");

  EXPECT_FALSE(
    savo_mapping::session::
    validate_map_id("../map").empty());

  EXPECT_FALSE(
    savo_mapping::session::
    validate_map_id("CampusMap").empty());

  EXPECT_FALSE(
    savo_mapping::session::
    validate_map_id("campus-map").empty());

  EXPECT_FALSE(
    savo_mapping::session::
    validate_map_id("a").empty());
}

TEST(MapSessionPolicy, RequiresAbsoluteOutputRoot)
{
  EXPECT_EQ(
    savo_mapping::session::
    validate_output_root(
      fs::path{"relative/maps"}),
    "output_root_must_be_absolute");

  EXPECT_TRUE(
    savo_mapping::session::
    validate_output_root(
      fs::path{"/tmp/savo_maps"})
    .empty());
}

TEST(MapSessionPolicy, BuildsExpectedArtifacts)
{
  const auto paths =
    savo_mapping::session::
    build_session_paths(
      "/tmp/savo_maps",
      "campus_floor_2");

  EXPECT_EQ(
    paths.session_directory,
    fs::path{
      "/tmp/savo_maps/campus_floor_2"});

  EXPECT_EQ(
    paths.artifacts.grid_yaml,
    fs::path{
      "/tmp/savo_maps/campus_floor_2/"
      "campus_floor_2.yaml"});

  EXPECT_EQ(
    paths.artifacts.posegraph,
    fs::path{
      "/tmp/savo_maps/campus_floor_2/"
      "campus_floor_2.posegraph"});

  EXPECT_EQ(
    paths.artifacts.posegraph_data,
    fs::path{
      "/tmp/savo_maps/campus_floor_2/"
      "campus_floor_2.data"});
}

TEST(MapSessionPolicy, EnforcesReadiness)
{
  savo_mapping::session::SaveReadiness readiness;

  EXPECT_EQ(
    savo_mapping::session::
    validate_save_readiness(
      readiness,
      true,
      true,
      true),
    "mapping_is_not_ready");

  readiness.mapping_ready = true;

  EXPECT_EQ(
    savo_mapping::session::
    validate_save_readiness(
      readiness,
      true,
      true,
      true),
    "slam_toolbox_is_not_active");

  readiness.slam_active = true;

  EXPECT_EQ(
    savo_mapping::session::
    validate_save_readiness(
      readiness,
      true,
      true,
      true),
    "map_is_not_structurally_valid");

  readiness.map_structurally_valid = true;

  EXPECT_TRUE(
    savo_mapping::session::
    validate_save_readiness(
      readiness,
      true,
      true,
      true).empty());
}

TEST(MapSessionPolicy, DetectsExistingSession)
{
  const fs::path root =
    unique_test_root();

  const auto paths =
    savo_mapping::session::
    build_session_paths(
      root,
      "campus_map");

  EXPECT_FALSE(
    savo_mapping::session::
    session_target_exists(paths));

  fs::create_directories(
    paths.session_directory);

  EXPECT_TRUE(
    savo_mapping::session::
    session_target_exists(paths));

  fs::remove_all(root);
}

TEST(MapSessionPolicy, ValidatesRequiredArtifacts)
{
  const fs::path root =
    unique_test_root();

  fs::create_directories(root);

  const auto artifacts =
    savo_mapping::session::
    build_artifact_paths(
      root / "test_map");

  EXPECT_EQ(
    savo_mapping::session::
    missing_required_artifacts(
      artifacts).size(),
    4u);

  std::ofstream{
    artifacts.grid_yaml}.put('\n');

  std::ofstream{
    artifacts.grid_pgm}.put('\n');

  std::ofstream{
    artifacts.posegraph}.put('\n');

  std::ofstream{
    artifacts.posegraph_data}.put('\n');

  EXPECT_TRUE(
    savo_mapping::session::
    missing_required_artifacts(
      artifacts).empty());

  fs::remove_all(root);
}
