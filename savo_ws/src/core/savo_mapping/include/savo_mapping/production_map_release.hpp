#pragma once

#include "savo_mapping/saved_map_contract.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace savo_mapping::release
{

inline constexpr char kReleaseStateTopic[] =
  "/savo_mapping/map_release/state";

inline constexpr char kReleaseResultTopic[] =
  "/savo_mapping/map_release/result";

inline constexpr char kMapCatalogTopic[] =
  "/savo_mapping/map_catalog";

inline constexpr char kActiveMapTopic[] =
  "/savo_mapping/active_map";

inline constexpr char kCreateReleaseService[] =
  "/savo_mapping/map_release/create";

inline constexpr char kVerifyReleaseService[] =
  "/savo_mapping/map_release/verify";

inline constexpr char kPromoteReleaseService[] =
  "/savo_mapping/map_release/promote";

inline constexpr char kDeactivateReleaseService[] =
  "/savo_mapping/map_release/deactivate";

struct ArtifactRecord
{
  std::string role;
  std::filesystem::path path;

  std::uintmax_t size_bytes{0};
  std::string sha256;
};

struct ReleaseRecord
{
  bool valid{false};

  int schema_version{1};

  std::string reason{"not_verified"};
  std::string release_id;
  std::string map_id;
  std::string frame_id{"map"};

  std::uint64_t created_unix_ns{0};

  std::filesystem::path release_directory;
  std::filesystem::path release_manifest;

  std::filesystem::path map_yaml;
  std::filesystem::path map_image;
  std::filesystem::path posegraph;
  std::filesystem::path data;
  std::filesystem::path quality_report;
  std::filesystem::path source_manifest;

  std::vector<ArtifactRecord> artifacts;
};

struct ActiveMapContract
{
  int schema_version{1};

  bool active{false};

  std::string reason{"no_active_release"};
  std::string release_id;
  std::string map_id;
  std::string frame_id{"map"};

  std::uint64_t changed_unix_ns{0};

  std::filesystem::path release_directory;
  std::filesystem::path release_manifest;
  std::filesystem::path map_yaml;
  std::filesystem::path quality_report;

  std::string release_manifest_sha256;
};

bool valid_release_id(
  const std::string & release_id);

ReleaseRecord create_release(
  const session::SavedMapVerification & source,
  const std::filesystem::path & production_root,
  const std::string & release_id,
  bool make_read_only);

ReleaseRecord verify_release(
  const std::filesystem::path & production_root,
  const std::string & release_id);

ActiveMapContract promote_release(
  const std::filesystem::path & production_root,
  const std::string & release_id);

ActiveMapContract deactivate_active_map(
  const std::filesystem::path & production_root);

ActiveMapContract read_active_map(
  const std::filesystem::path & production_root);

std::string release_record_to_json(
  const ReleaseRecord & release);

std::string active_map_to_json(
  const ActiveMapContract & active);

std::string catalog_to_json(
  const std::filesystem::path & production_root);

}  // namespace savo_mapping::release
