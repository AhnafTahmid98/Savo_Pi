#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace savo_mapping::session
{

struct SaveReadiness
{
  bool mapping_ready{false};
  bool slam_active{false};
  bool map_structurally_valid{false};
};

struct MapArtifactPaths
{
  std::filesystem::path base;

  std::filesystem::path grid_yaml;
  std::filesystem::path grid_pgm;
  std::filesystem::path grid_png;

  std::filesystem::path posegraph;
  std::filesystem::path posegraph_data;
};

struct MapSessionPaths
{
  std::filesystem::path output_root;
  std::filesystem::path session_directory;
  std::filesystem::path manifest;

  MapArtifactPaths artifacts;
};

std::string validate_map_id(
  std::string_view map_id);

std::filesystem::path expand_user_path(
  std::string_view path);

std::string validate_output_root(
  const std::filesystem::path & output_root);

MapArtifactPaths build_artifact_paths(
  const std::filesystem::path & base);

MapSessionPaths build_session_paths(
  const std::filesystem::path & output_root,
  std::string_view map_id);

std::string validate_save_readiness(
  const SaveReadiness & readiness,
  bool require_mapping_ready,
  bool require_slam_active,
  bool require_structurally_valid_map);

bool session_target_exists(
  const MapSessionPaths & paths);

std::vector<std::filesystem::path>
missing_required_artifacts(
  const MapArtifactPaths & paths);

}  // namespace savo_mapping::session
