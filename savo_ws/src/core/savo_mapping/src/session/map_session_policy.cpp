#include "savo_mapping/map_session_policy.hpp"

#include <cstdlib>
#include <regex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace savo_mapping::session
{

namespace fs = std::filesystem;

std::string validate_map_id(
  std::string_view map_id)
{
  if (map_id.empty()) {
    return "map_id_is_empty";
  }

  if (map_id.size() > 64) {
    return "map_id_is_too_long";
  }

  static const std::regex allowed{
    "^[a-z][a-z0-9_]{1,63}$"};

  if (!std::regex_match(
      std::string{map_id},
      allowed))
  {
    return
      "map_id_must_start_with_lowercase_and_use_"
      "only_lowercase_letters_numbers_underscores";
  }

  return {};
}

fs::path expand_user_path(
  std::string_view raw_path)
{
  std::string value{raw_path};

  if (value.empty() || value.front() != '~') {
    return fs::path{value};
  }

  const char * home = std::getenv("HOME");

  if (home == nullptr || std::string{home}.empty()) {
    throw std::runtime_error(
            "HOME_is_not_available");
  }

  if (value == "~") {
    return fs::path{home};
  }

  if (value.rfind("~/", 0) == 0) {
    return fs::path{home} / value.substr(2);
  }

  throw std::invalid_argument(
          "only_current_user_tilde_expansion_is_supported");
}

std::string validate_output_root(
  const fs::path & output_root)
{
  if (output_root.empty()) {
    return "output_root_is_empty";
  }

  if (!output_root.is_absolute()) {
    return "output_root_must_be_absolute";
  }

  const std::string normalized =
    output_root.lexically_normal().generic_string();

  static const std::regex allowed{
    "^[A-Za-z0-9_./-]+$"};

  if (!std::regex_match(normalized, allowed)) {
    return
      "output_root_contains_unsupported_characters";
  }

  for (const auto & component :
       output_root.lexically_normal())
  {
    if (component == "..") {
      return "output_root_must_not_contain_parent_traversal";
    }
  }

  return {};
}

MapArtifactPaths build_artifact_paths(
  const fs::path & base)
{
  MapArtifactPaths paths;
  paths.base = base;

  paths.grid_yaml = base;
  paths.grid_yaml += ".yaml";

  paths.grid_pgm = base;
  paths.grid_pgm += ".pgm";

  paths.grid_png = base;
  paths.grid_png += ".png";

  paths.posegraph = base;
  paths.posegraph += ".posegraph";

  paths.posegraph_data = base;
  paths.posegraph_data += ".data";

  return paths;
}

MapSessionPaths build_session_paths(
  const fs::path & output_root,
  std::string_view map_id)
{
  const std::string map_id_error =
    validate_map_id(map_id);

  if (!map_id_error.empty()) {
    throw std::invalid_argument(map_id_error);
  }

  const fs::path normalized_root =
    output_root.lexically_normal();

  MapSessionPaths paths;

  paths.output_root = normalized_root;

  paths.session_directory =
    normalized_root / std::string{map_id};

  paths.manifest =
    paths.session_directory / "manifest.yaml";

  paths.artifacts = build_artifact_paths(
    paths.session_directory /
    std::string{map_id});

  return paths;
}

std::string validate_save_readiness(
  const SaveReadiness & readiness,
  bool require_mapping_ready,
  bool require_slam_active,
  bool require_structurally_valid_map)
{
  if (require_mapping_ready &&
      !readiness.mapping_ready)
  {
    return "mapping_is_not_ready";
  }

  if (require_slam_active &&
      !readiness.slam_active)
  {
    return "slam_toolbox_is_not_active";
  }

  if (require_structurally_valid_map &&
      !readiness.map_structurally_valid)
  {
    return "map_is_not_structurally_valid";
  }

  return {};
}

bool session_target_exists(
  const MapSessionPaths & paths)
{
  std::error_code error;

  const bool exists =
    fs::exists(paths.session_directory, error);

  return !error && exists;
}

std::vector<fs::path>
missing_required_artifacts(
  const MapArtifactPaths & paths)
{
  std::vector<fs::path> missing;

  if (!fs::is_regular_file(paths.grid_yaml)) {
    missing.push_back(paths.grid_yaml);
  }

  const bool image_exists =
    fs::is_regular_file(paths.grid_pgm) ||
    fs::is_regular_file(paths.grid_png);

  if (!image_exists) {
    missing.push_back(paths.grid_pgm);
  }

  if (!fs::is_regular_file(paths.posegraph)) {
    missing.push_back(paths.posegraph);
  }

  if (!fs::is_regular_file(paths.posegraph_data)) {
    missing.push_back(paths.posegraph_data);
  }

  return missing;
}

}  // namespace savo_mapping::session
