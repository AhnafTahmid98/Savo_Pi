#include "savo_mapping/production_map_release.hpp"

#include "savo_mapping/saved_map_quality.hpp"

#include <openssl/evp.h>

#include <yaml-cpp/yaml.h>

#include <array>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace savo_mapping::release
{

namespace
{

namespace fs = std::filesystem;

constexpr int kReleaseSchemaVersion = 1;

std::uint64_t unix_now_ns()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<
      std::chrono::nanoseconds>(
      std::chrono::system_clock::now()
      .time_since_epoch())
    .count());
}

std::string escape_json(
  const std::string & value)
{
  std::ostringstream output;

  for (const char character : value) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;

      case '"':
        output << "\\\"";
        break;

      case '\n':
        output << "\\n";
        break;

      case '\r':
        output << "\\r";
        break;

      case '\t':
        output << "\\t";
        break;

      default:
        output << character;
        break;
    }
  }

  return output.str();
}

void write_yaml_atomic(
  const fs::path & path,
  const YAML::Node & node)
{
  fs::create_directories(
    path.parent_path());

  const fs::path temporary =
    path.string() +
    ".tmp." +
    std::to_string(unix_now_ns());

  {
    std::ofstream output{temporary};

    if (!output.is_open()) {
      throw std::runtime_error(
              "cannot_open_temporary_yaml:" +
              temporary.string());
    }

    output << node << '\n';
    output.flush();

    if (!output.good()) {
      throw std::runtime_error(
              "cannot_write_temporary_yaml:" +
              temporary.string());
    }
  }

  std::error_code error;

  fs::rename(
    temporary,
    path,
    error);

  if (error) {
    fs::remove(temporary);

    throw std::runtime_error(
            "atomic_yaml_rename_failed:" +
            error.message());
  }
}

std::string sha256_file(
  const fs::path & path)
{
  std::ifstream input{
    path,
    std::ios::binary};

  if (!input.is_open()) {
    throw std::runtime_error(
            "cannot_open_for_hash:" +
            path.string());
  }

  using ContextPtr =
    std::unique_ptr<
      EVP_MD_CTX,
      decltype(&EVP_MD_CTX_free)>;

  ContextPtr context{
    EVP_MD_CTX_new(),
    EVP_MD_CTX_free};

  if (!context) {
    throw std::runtime_error(
            "sha256_context_creation_failed");
  }

  if (EVP_DigestInit_ex(
      context.get(),
      EVP_sha256(),
      nullptr) != 1)
  {
    throw std::runtime_error(
            "sha256_initialization_failed");
  }

  std::array<char, 65536> buffer{};

  while (input.good()) {
    input.read(
      buffer.data(),
      static_cast<std::streamsize>(
        buffer.size()));

    const std::streamsize count =
      input.gcount();

    if (count > 0) {
      if (EVP_DigestUpdate(
          context.get(),
          buffer.data(),
          static_cast<std::size_t>(
            count)) != 1)
      {
        throw std::runtime_error(
                "sha256_update_failed");
      }
    }
  }

  if (!input.eof()) {
    throw std::runtime_error(
            "sha256_file_read_failed:" +
            path.string());
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE>
    digest{};

  unsigned int digest_size = 0U;

  if (EVP_DigestFinal_ex(
      context.get(),
      digest.data(),
      &digest_size) != 1)
  {
    throw std::runtime_error(
            "sha256_finalization_failed");
  }

  std::ostringstream output;

  output
    << std::hex
    << std::setfill('0');

  for (unsigned int index = 0U;
    index < digest_size;
    ++index)
  {
    output
      << std::setw(2)
      << static_cast<unsigned int>(
           digest[index]);
  }

  return output.str();
}

ArtifactRecord make_artifact(
  const std::string & role,
  const fs::path & path)
{
  if (!fs::is_regular_file(path)) {
    throw std::runtime_error(
            "release_artifact_missing:" +
            role);
  }

  ArtifactRecord artifact;

  artifact.role = role;
  artifact.path = path;
  artifact.size_bytes =
    fs::file_size(path);

  artifact.sha256 =
    sha256_file(path);

  return artifact;
}

void copy_required_file(
  const fs::path & source,
  const fs::path & destination)
{
  if (!fs::is_regular_file(source) ||
      fs::file_size(source) == 0U)
  {
    throw std::runtime_error(
            "source_artifact_missing_or_empty:" +
            source.string());
  }

  std::error_code error;

  fs::copy_file(
    source,
    destination,
    fs::copy_options::none,
    error);

  if (error) {
    throw std::runtime_error(
            "artifact_copy_failed:" +
            error.message());
  }
}

fs::path resolve_release_path(
  const fs::path & release_directory,
  const std::string & value)
{
  fs::path candidate{value};

  if (candidate.is_absolute()) {
    throw std::runtime_error(
            "absolute_release_artifact_path");
  }

  const fs::path root =
    fs::weakly_canonical(
    release_directory);

  const fs::path resolved =
    fs::weakly_canonical(
    release_directory /
    candidate);

  const fs::path relative =
    resolved.lexically_relative(root);

  if (relative.empty() ||
      relative.begin() ==
        relative.end() ||
      *relative.begin() == "..")
  {
    throw std::runtime_error(
            "release_artifact_path_escape");
  }

  return resolved;
}

void validate_source_approval(
  const session::SavedMapVerification & source)
{
  if (!source.valid) {
    throw std::runtime_error(
            "source_session_not_verified:" +
            source.reason);
  }

  const YAML::Node manifest =
    YAML::LoadFile(
    source.paths.manifest.string());

  if (!manifest[
      "navigation_handoff_ready"] ||
      !manifest[
      "navigation_handoff_ready"].as<bool>())
  {
    throw std::runtime_error(
            "navigation_handoff_not_ready");
  }

  const YAML::Node quality =
    manifest["map_quality"];

  if (!quality ||
      !quality["evaluated"] ||
      !quality["evaluated"].as<bool>() ||
      !quality["passed"] ||
      !quality["passed"].as<bool>())
  {
    throw std::runtime_error(
            "map_quality_not_passed");
  }

  const YAML::Node handoff =
    manifest["navigation_handoff"];

  if (!handoff ||
      !handoff["approved"] ||
      !handoff["approved"].as<bool>())
  {
    throw std::runtime_error(
            "navigation_handoff_not_approved");
  }

  if (!handoff["map_id"] ||
      handoff["map_id"].as<std::string>() !=
        source.map_id)
  {
    throw std::runtime_error(
            "navigation_handoff_map_id_mismatch");
  }

  if (!handoff["quality_report"]) {
    throw std::runtime_error(
            "quality_report_path_missing");
  }

  const fs::path quality_report =
    handoff["quality_report"]
    .as<std::string>();

  if (!fs::is_regular_file(
      quality_report))
  {
    throw std::runtime_error(
            "quality_report_missing");
  }

  const YAML::Node report =
    YAML::LoadFile(
    quality_report.string());

  if (!report["passed"] ||
      !report["passed"].as<bool>() ||
      !report["map_id"] ||
      report["map_id"].as<std::string>() !=
        source.map_id)
  {
    throw std::runtime_error(
            "quality_report_not_approved");
  }
}

fs::path source_quality_report(
  const session::SavedMapVerification & source)
{
  const YAML::Node manifest =
    YAML::LoadFile(
    source.paths.manifest.string());

  const YAML::Node handoff =
    manifest["navigation_handoff"];

  return fs::path{
    handoff["quality_report"]
    .as<std::string>()};
}

YAML::Node artifact_to_yaml(
  const ArtifactRecord & artifact,
  const fs::path & release_directory)
{
  YAML::Node node;

  node["role"] = artifact.role;

  node["path"] =
    artifact.path
    .lexically_relative(
      release_directory)
    .string();

  node["size_bytes"] =
    artifact.size_bytes;

  node["sha256"] =
    artifact.sha256;

  return node;
}

void append_catalog_entry(
  const fs::path & production_root,
  const ReleaseRecord & release)
{
  const fs::path catalog_path =
    production_root /
    "catalog.yaml";

  YAML::Node catalog;

  if (fs::is_regular_file(catalog_path)) {
    catalog =
      YAML::LoadFile(
      catalog_path.string());
  } else {
    catalog["schema_version"] = 1;
    catalog["releases"] =
      YAML::Node(
      YAML::NodeType::Sequence);
  }

  if (!catalog["releases"]) {
    catalog["releases"] =
      YAML::Node(
      YAML::NodeType::Sequence);
  }

  for (const auto & entry :
    catalog["releases"])
  {
    if (entry["release_id"] &&
        entry["release_id"]
        .as<std::string>() ==
        release.release_id)
    {
      throw std::runtime_error(
              "release_already_cataloged");
    }
  }

  YAML::Node entry;

  entry["release_id"] =
    release.release_id;

  entry["map_id"] =
    release.map_id;

  entry["frame_id"] =
    release.frame_id;

  entry["created_unix_ns"] =
    release.created_unix_ns;

  entry["release_directory"] =
    release.release_directory.string();

  entry["release_manifest"] =
    release.release_manifest.string();

  entry["release_manifest_sha256"] =
    sha256_file(
    release.release_manifest);

  catalog["releases"].push_back(entry);

  write_yaml_atomic(
    catalog_path,
    catalog);
}

void make_release_read_only(
  const fs::path & release_directory)
{
  const fs::perms file_permissions =
    fs::perms::owner_read |
    fs::perms::group_read |
    fs::perms::others_read;

  const fs::perms directory_permissions =
    fs::perms::owner_read |
    fs::perms::owner_exec |
    fs::perms::group_read |
    fs::perms::group_exec |
    fs::perms::others_read |
    fs::perms::others_exec;

  for (const auto & entry :
    fs::recursive_directory_iterator(
      release_directory))
  {
    if (entry.is_regular_file()) {
      fs::permissions(
        entry.path(),
        file_permissions,
        fs::perm_options::replace);
    } else if (entry.is_directory()) {
      fs::permissions(
        entry.path(),
        directory_permissions,
        fs::perm_options::replace);
    }
  }

  fs::permissions(
    release_directory,
    directory_permissions,
    fs::perm_options::replace);
}

}  // namespace

bool valid_release_id(
  const std::string & release_id)
{
  if (release_id.empty() ||
      release_id.size() > 64U)
  {
    return false;
  }

  const auto valid_character =
    [](const unsigned char character)
    {
      return
        std::isalnum(character) != 0 ||
        character == '-' ||
        character == '_' ||
        character == '.';
    };

  if (std::isalnum(
      static_cast<unsigned char>(
        release_id.front())) == 0)
  {
    return false;
  }

  for (const unsigned char character :
    release_id)
  {
    if (!valid_character(character)) {
      return false;
    }
  }

  return
    release_id.find("..") ==
    std::string::npos;
}

ReleaseRecord create_release(
  const session::SavedMapVerification & source,
  const fs::path & production_root,
  const std::string & release_id,
  const bool make_read_only)
{
  validate_source_approval(source);

  if (!valid_release_id(release_id)) {
    throw std::runtime_error(
            "invalid_release_id");
  }

  const fs::path releases_root =
    production_root /
    "releases";

  const fs::path final_directory =
    releases_root /
    release_id;

  if (fs::exists(final_directory)) {
    throw std::runtime_error(
            "release_already_exists");
  }

  fs::create_directories(
    releases_root);

  const fs::path staging =
    releases_root /
    (
      "." +
      release_id +
      ".staging." +
      std::to_string(unix_now_ns()));

  fs::create_directories(staging);

  try {
    const fs::path release_map_yaml =
      staging /
      source.paths.grid_yaml.filename();

    const fs::path release_map_image =
      staging /
      source.paths.grid_image.filename();

    const fs::path release_posegraph =
      staging /
      source.paths.posegraph.filename();

    const fs::path release_data =
      staging /
      source.paths.posegraph_data.filename();

    const fs::path release_quality =
      staging /
      "quality_report.yaml";

    const fs::path release_source_manifest =
      staging /
      "source_manifest.yaml";

    copy_required_file(
      source.paths.grid_image,
      release_map_image);

    copy_required_file(
      source.paths.posegraph,
      release_posegraph);

    copy_required_file(
      source.paths.posegraph_data,
      release_data);

    copy_required_file(
      source_quality_report(source),
      release_quality);

    copy_required_file(
      source.paths.manifest,
      release_source_manifest);

    YAML::Node map_yaml =
      YAML::LoadFile(
      source.paths.grid_yaml.string());

    map_yaml["image"] =
      release_map_image
      .filename()
      .string();

    write_yaml_atomic(
      release_map_yaml,
      map_yaml);

    std::vector<ArtifactRecord> artifacts{
      make_artifact(
        "map_yaml",
        release_map_yaml),

      make_artifact(
        "map_image",
        release_map_image),

      make_artifact(
        "posegraph",
        release_posegraph),

      make_artifact(
        "data",
        release_data),

      make_artifact(
        "quality_report",
        release_quality),

      make_artifact(
        "source_manifest",
        release_source_manifest),
    };

    const std::uint64_t created =
      unix_now_ns();

    YAML::Node manifest;

    manifest["schema_version"] =
      kReleaseSchemaVersion;

    manifest["release_id"] =
      release_id;

    manifest["map_id"] =
      source.map_id;

    manifest["frame_id"] =
      source.frame_id;

    manifest["created_unix_ns"] =
      created;

    manifest["immutable"] = true;

    manifest["source"]["session_directory"] =
      source.paths.session_directory
      .string();

    manifest["source"]["manifest"] =
      source.paths.manifest.string();

    manifest["quality"]["passed"] =
      true;

    manifest["quality"]["operator_approved"] =
      true;

    manifest["navigation"]["eligible"] =
      true;

    manifest["artifacts"] =
      YAML::Node(
      YAML::NodeType::Sequence);

    for (const auto & artifact :
      artifacts)
    {
      manifest["artifacts"].push_back(
        artifact_to_yaml(
          artifact,
          staging));
    }

    const fs::path release_manifest =
      staging /
      "release_manifest.yaml";

    write_yaml_atomic(
      release_manifest,
      manifest);

    std::error_code rename_error;

    fs::rename(
      staging,
      final_directory,
      rename_error);

    if (rename_error) {
      throw std::runtime_error(
              "release_commit_failed:" +
              rename_error.message());
    }

    ReleaseRecord release =
      verify_release(
      production_root,
      release_id);

    if (!release.valid) {
      fs::remove_all(final_directory);

      throw std::runtime_error(
              "release_post_commit_verification_failed:" +
              release.reason);
    }

    try {
      append_catalog_entry(
        production_root,
        release);
    } catch (...) {
      fs::remove_all(final_directory);
      throw;
    }

    if (make_read_only) {
      make_release_read_only(
        final_directory);
    }

    return verify_release(
      production_root,
      release_id);
  } catch (...) {
    std::error_code cleanup_error;
    fs::remove_all(
      staging,
      cleanup_error);

    throw;
  }
}

ReleaseRecord verify_release(
  const fs::path & production_root,
  const std::string & release_id)
{
  ReleaseRecord release;

  release.release_id = release_id;

  if (!valid_release_id(release_id)) {
    release.reason =
      "invalid_release_id";

    return release;
  }

  release.release_directory =
    production_root /
    "releases" /
    release_id;

  release.release_manifest =
    release.release_directory /
    "release_manifest.yaml";

  if (!fs::is_regular_file(
      release.release_manifest))
  {
    release.reason =
      "release_manifest_missing";

    return release;
  }

  try {
    const YAML::Node manifest =
      YAML::LoadFile(
      release.release_manifest.string());

    if (!manifest["schema_version"] ||
        manifest["schema_version"].as<int>() !=
          kReleaseSchemaVersion)
    {
      release.reason =
        "release_schema_mismatch";

      return release;
    }

    release.schema_version =
      manifest["schema_version"].as<int>();

    if (!manifest["release_id"] ||
        manifest["release_id"].as<std::string>() !=
          release_id)
    {
      release.reason =
        "release_id_mismatch";

      return release;
    }

    if (!manifest["immutable"] ||
        !manifest["immutable"].as<bool>())
    {
      release.reason =
        "release_not_immutable";

      return release;
    }

    release.map_id =
      manifest["map_id"].as<std::string>();

    release.frame_id =
      manifest["frame_id"].as<std::string>();

    release.created_unix_ns =
      manifest["created_unix_ns"]
      .as<std::uint64_t>();

    const YAML::Node artifacts =
      manifest["artifacts"];

    if (!artifacts ||
        !artifacts.IsSequence())
    {
      release.reason =
        "release_artifacts_missing";

      return release;
    }

    for (const auto & node :
      artifacts)
    {
      ArtifactRecord artifact;

      artifact.role =
        node["role"].as<std::string>();

      artifact.path =
        resolve_release_path(
        release.release_directory,
        node["path"].as<std::string>());

      artifact.size_bytes =
        node["size_bytes"]
        .as<std::uintmax_t>();

      artifact.sha256 =
        node["sha256"]
        .as<std::string>();

      if (!fs::is_regular_file(
          artifact.path))
      {
        release.reason =
          "release_artifact_missing:" +
          artifact.role;

        return release;
      }

      if (fs::file_size(
          artifact.path) !=
          artifact.size_bytes)
      {
        release.reason =
          "release_artifact_size_mismatch:" +
          artifact.role;

        return release;
      }

      if (sha256_file(
          artifact.path) !=
          artifact.sha256)
      {
        release.reason =
          "release_artifact_hash_mismatch:" +
          artifact.role;

        return release;
      }

      if (artifact.role == "map_yaml") {
        release.map_yaml =
          artifact.path;
      } else if (
        artifact.role == "map_image")
      {
        release.map_image =
          artifact.path;
      } else if (
        artifact.role == "posegraph")
      {
        release.posegraph =
          artifact.path;
      } else if (
        artifact.role == "data")
      {
        release.data =
          artifact.path;
      } else if (
        artifact.role == "quality_report")
      {
        release.quality_report =
          artifact.path;
      } else if (
        artifact.role == "source_manifest")
      {
        release.source_manifest =
          artifact.path;
      }

      release.artifacts.push_back(
        artifact);
    }

    if (release.map_yaml.empty() ||
        release.map_image.empty() ||
        release.posegraph.empty() ||
        release.data.empty() ||
        release.quality_report.empty() ||
        release.source_manifest.empty())
    {
      release.reason =
        "required_release_artifact_missing";

      return release;
    }

    const YAML::Node map_yaml =
      YAML::LoadFile(
      release.map_yaml.string());

    if (!map_yaml["image"] ||
        map_yaml["image"].as<std::string>() !=
          release.map_image
          .filename()
          .string())
    {
      release.reason =
        "release_map_image_contract_invalid";

      return release;
    }

    const YAML::Node quality =
      YAML::LoadFile(
      release.quality_report.string());

    if (!quality["passed"] ||
        !quality["passed"].as<bool>() ||
        !quality["map_id"] ||
        quality["map_id"].as<std::string>() !=
          release.map_id)
    {
      release.reason =
        "release_quality_contract_invalid";

      return release;
    }

    release.valid = true;
    release.reason =
      "release_verified";

    return release;
  } catch (const std::exception & exception) {
    release.reason =
      std::string{
      "release_verification_error:"} +
      exception.what();

    return release;
  }
}

ActiveMapContract promote_release(
  const fs::path & production_root,
  const std::string & release_id)
{
  const ReleaseRecord release =
    verify_release(
    production_root,
    release_id);

  if (!release.valid) {
    throw std::runtime_error(
            "release_verification_failed:" +
            release.reason);
  }

  ActiveMapContract active;

  active.active = true;
  active.reason =
    "operator_promoted";

  active.release_id =
    release.release_id;

  active.map_id =
    release.map_id;

  active.frame_id =
    release.frame_id;

  active.changed_unix_ns =
    unix_now_ns();

  active.release_directory =
    release.release_directory;

  active.release_manifest =
    release.release_manifest;

  active.map_yaml =
    release.map_yaml;

  active.quality_report =
    release.quality_report;

  active.release_manifest_sha256 =
    sha256_file(
    release.release_manifest);

  YAML::Node node;

  node["schema_version"] = 1;
  node["active"] = true;
  node["reason"] = active.reason;
  node["changed_unix_ns"] =
    active.changed_unix_ns;

  node["release_id"] =
    active.release_id;

  node["map_id"] =
    active.map_id;

  node["frame_id"] =
    active.frame_id;

  node["release_directory"] =
    active.release_directory.string();

  node["release_manifest"] =
    active.release_manifest.string();

  node["release_manifest_sha256"] =
    active.release_manifest_sha256;

  node["map_yaml"] =
    active.map_yaml.string();

  node["quality_report"] =
    active.quality_report.string();

  write_yaml_atomic(
    production_root /
    "active_map.yaml",
    node);

  return read_active_map(
    production_root);
}

ActiveMapContract deactivate_active_map(
  const fs::path & production_root)
{
  ActiveMapContract previous =
    read_active_map(
    production_root);

  YAML::Node node;

  node["schema_version"] = 1;
  node["active"] = false;
  node["reason"] =
    "operator_deactivated";

  node["changed_unix_ns"] =
    unix_now_ns();

  if (!previous.release_id.empty()) {
    node["previous_release_id"] =
      previous.release_id;

    node["previous_map_id"] =
      previous.map_id;
  }

  write_yaml_atomic(
    production_root /
    "active_map.yaml",
    node);

  return read_active_map(
    production_root);
}

ActiveMapContract read_active_map(
  const fs::path & production_root)
{
  ActiveMapContract active;

  const fs::path path =
    production_root /
    "active_map.yaml";

  if (!fs::is_regular_file(path)) {
    active.reason =
      "no_active_release";

    return active;
  }

  try {
    const YAML::Node node =
      YAML::LoadFile(
      path.string());

    active.schema_version =
      node["schema_version"]
      .as<int>();

    active.active =
      node["active"]
      .as<bool>();

    active.reason =
      node["reason"]
      .as<std::string>();

    active.changed_unix_ns =
      node["changed_unix_ns"]
      .as<std::uint64_t>();

    if (!active.active) {
      return active;
    }

    active.release_id =
      node["release_id"]
      .as<std::string>();

    active.map_id =
      node["map_id"]
      .as<std::string>();

    active.frame_id =
      node["frame_id"]
      .as<std::string>();

    active.release_directory =
      node["release_directory"]
      .as<std::string>();

    active.release_manifest =
      node["release_manifest"]
      .as<std::string>();

    active.release_manifest_sha256 =
      node["release_manifest_sha256"]
      .as<std::string>();

    active.map_yaml =
      node["map_yaml"]
      .as<std::string>();

    active.quality_report =
      node["quality_report"]
      .as<std::string>();

    const ReleaseRecord release =
      verify_release(
      production_root,
      active.release_id);

    if (!release.valid) {
      active.active = false;
      active.reason =
        "active_release_verification_failed:" +
        release.reason;

      return active;
    }

    if (sha256_file(
        active.release_manifest) !=
        active.release_manifest_sha256)
    {
      active.active = false;
      active.reason =
        "active_manifest_hash_mismatch";

      return active;
    }

    active.reason =
      "active_release_verified";

    return active;
  } catch (const std::exception & exception) {
    active.active = false;

    active.reason =
      std::string{
      "active_map_contract_error:"} +
      exception.what();

    return active;
  }
}

std::string release_record_to_json(
  const ReleaseRecord & release)
{
  std::ostringstream output;

  output
    << std::boolalpha
    << "{"
    << "\"valid\":"
    << release.valid
    << ",\"reason\":\""
    << escape_json(release.reason)
    << "\",\"release_id\":\""
    << escape_json(release.release_id)
    << "\",\"map_id\":\""
    << escape_json(release.map_id)
    << "\",\"frame_id\":\""
    << escape_json(release.frame_id)
    << "\",\"release_directory\":\""
    << escape_json(
         release.release_directory.string())
    << "\",\"release_manifest\":\""
    << escape_json(
         release.release_manifest.string())
    << "\",\"map_yaml\":\""
    << escape_json(
         release.map_yaml.string())
    << "\"}";

  return output.str();
}

std::string active_map_to_json(
  const ActiveMapContract & active)
{
  std::ostringstream output;

  output
    << std::boolalpha
    << "{"
    << "\"schema_version\":"
    << active.schema_version
    << ",\"active\":"
    << active.active
    << ",\"reason\":\""
    << escape_json(active.reason)
    << "\",\"release_id\":\""
    << escape_json(active.release_id)
    << "\",\"map_id\":\""
    << escape_json(active.map_id)
    << "\",\"frame_id\":\""
    << escape_json(active.frame_id)
    << "\",\"release_manifest\":\""
    << escape_json(
         active.release_manifest.string())
    << "\",\"map_yaml\":\""
    << escape_json(
         active.map_yaml.string())
    << "\",\"quality_report\":\""
    << escape_json(
         active.quality_report.string())
    << "\"}";

  return output.str();
}

std::string catalog_to_json(
  const fs::path & production_root)
{
  const fs::path catalog_path =
    production_root /
    "catalog.yaml";

  std::ostringstream output;

  output
    << "{\"schema_version\":1,"
    << "\"production_root\":\""
    << escape_json(
         production_root.string())
    << "\",\"releases\":[";

  if (fs::is_regular_file(catalog_path)) {
    const YAML::Node catalog =
      YAML::LoadFile(
      catalog_path.string());

    const YAML::Node releases =
      catalog["releases"];

    bool first = true;

    if (releases &&
        releases.IsSequence())
    {
      for (const auto & entry :
        releases)
      {
        if (!first) {
          output << ',';
        }

        first = false;

        output
          << "{"
          << "\"release_id\":\""
          << escape_json(
               entry["release_id"]
               .as<std::string>())
          << "\",\"map_id\":\""
          << escape_json(
               entry["map_id"]
               .as<std::string>())
          << "\",\"frame_id\":\""
          << escape_json(
               entry["frame_id"]
               .as<std::string>())
          << "\",\"release_manifest\":\""
          << escape_json(
               entry["release_manifest"]
               .as<std::string>())
          << "\"}";
      }
    }
  }

  output << "]}";

  return output.str();
}

}  // namespace savo_mapping::release
