#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

namespace savo_mapping::session
{

struct SavedMapPaths
{
  std::filesystem::path session_directory;
  std::filesystem::path manifest;
  std::filesystem::path artifact_base;

  std::filesystem::path grid_yaml;
  std::filesystem::path grid_image;

  std::filesystem::path posegraph;
  std::filesystem::path posegraph_data;
};

struct SavedMapVerification
{
  bool valid{false};
  std::string reason{"not_evaluated"};

  std::string map_id;
  std::string frame_id;
  int schema_version{0};

  SavedMapPaths paths;

  std::uintmax_t grid_yaml_bytes{0};
  std::uintmax_t grid_image_bytes{0};
  std::uintmax_t posegraph_bytes{0};
  std::uintmax_t posegraph_data_bytes{0};
};

SavedMapVerification verify_saved_map_session(
  const std::filesystem::path & session_directory,
  std::string_view map_id,
  std::string_view expected_frame);

}  // namespace savo_mapping::session
