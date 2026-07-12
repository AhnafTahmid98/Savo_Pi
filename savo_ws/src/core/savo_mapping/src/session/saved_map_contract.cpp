#include "savo_mapping/saved_map_contract.hpp"

#include "savo_mapping/map_session_policy.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <string>
#include <string_view>

namespace savo_mapping::session
{

namespace
{

namespace fs = std::filesystem;

SavedMapVerification fail(
  SavedMapVerification result,
  std::string reason)
{
  result.valid = false;
  result.reason = std::move(reason);
  return result;
}

bool is_nonempty_regular_file(
  const fs::path & path)
{
  std::error_code error;

  if (!fs::is_regular_file(path, error) ||
      error)
  {
    return false;
  }

  return fs::file_size(path, error) > 0 &&
         !error;
}

fs::path resolve_path(
  const fs::path & parent,
  const std::string & value)
{
  fs::path path{value};

  if (path.is_relative()) {
    path = parent / path;
  }

  return fs::weakly_canonical(path);
}

bool path_is_within(
  const fs::path & parent,
  const fs::path & child)
{
  const fs::path relative =
    child.lexically_relative(parent);

  if (relative.empty()) {
    return child == parent;
  }

  if (relative.is_absolute()) {
    return false;
  }

  for (const auto & component : relative) {
    if (component == "..") {
      return false;
    }
  }

  return true;
}

bool paths_match(
  const fs::path & left,
  const fs::path & right)
{
  return fs::weakly_canonical(left) ==
         fs::weakly_canonical(right);
}

std::string required_string(
  const YAML::Node & node,
  const std::string & field)
{
  if (!node[field] ||
      !node[field].IsScalar())
  {
    throw std::runtime_error(
            "missing_or_invalid_field:" + field);
  }

  return node[field].as<std::string>();
}

}  // namespace

SavedMapVerification verify_saved_map_session(
  const fs::path & raw_session_directory,
  std::string_view map_id,
  std::string_view expected_frame)
{
  SavedMapVerification result;
  result.map_id = std::string{map_id};

  const std::string map_id_error =
    validate_map_id(map_id);

  if (!map_id_error.empty()) {
    return fail(
      std::move(result),
      map_id_error);
  }

  try {
    fs::path session_directory =
      expand_user_path(
      raw_session_directory.generic_string());

    session_directory =
      fs::absolute(session_directory)
      .lexically_normal();

    if (!fs::is_directory(session_directory)) {
      return fail(
        std::move(result),
        "session_directory_not_found");
    }

    session_directory =
      fs::weakly_canonical(
      session_directory);

    const std::string map_id_string{
      map_id};

    result.paths.session_directory =
      session_directory;

    result.paths.manifest =
      session_directory / "manifest.yaml";

    result.paths.artifact_base =
      session_directory / map_id_string;

    result.paths.grid_yaml =
      result.paths.artifact_base;
    result.paths.grid_yaml += ".yaml";

    result.paths.posegraph =
      result.paths.artifact_base;
    result.paths.posegraph += ".posegraph";

    result.paths.posegraph_data =
      result.paths.artifact_base;
    result.paths.posegraph_data += ".data";

    if (!is_nonempty_regular_file(
        result.paths.manifest))
    {
      return fail(
        std::move(result),
        "manifest_missing_or_empty");
    }

    if (!is_nonempty_regular_file(
        result.paths.grid_yaml))
    {
      return fail(
        std::move(result),
        "grid_yaml_missing_or_empty");
    }

    if (!is_nonempty_regular_file(
        result.paths.posegraph))
    {
      return fail(
        std::move(result),
        "posegraph_missing_or_empty");
    }

    if (!is_nonempty_regular_file(
        result.paths.posegraph_data))
    {
      return fail(
        std::move(result),
        "posegraph_data_missing_or_empty");
    }

    const YAML::Node manifest =
      YAML::LoadFile(
      result.paths.manifest.string());

    if (!manifest["schema_version"]) {
      return fail(
        std::move(result),
        "manifest_schema_version_missing");
    }

    result.schema_version =
      manifest["schema_version"].as<int>();

    if (result.schema_version != 1) {
      return fail(
        std::move(result),
        "unsupported_manifest_schema_version");
    }

    const std::string manifest_map_id =
      required_string(
      manifest,
      "map_id");

    if (manifest_map_id != map_id) {
      return fail(
        std::move(result),
        "manifest_map_id_mismatch");
    }

    result.frame_id =
      required_string(
      manifest,
      "frame_id");

    if (result.frame_id != expected_frame) {
      return fail(
        std::move(result),
        "manifest_frame_id_mismatch");
    }

    if (!manifest["map_quality"] ||
        !manifest["map_quality"]
          ["structurally_valid"] ||
        !manifest["map_quality"]
          ["structurally_valid"].as<bool>())
    {
      return fail(
        std::move(result),
        "manifest_map_not_structurally_valid");
    }

    const fs::path manifest_directory =
      resolve_path(
      result.paths.manifest.parent_path(),
      required_string(
        manifest,
        "session_directory"));

    if (!paths_match(
        manifest_directory,
        session_directory))
    {
      return fail(
        std::move(result),
        "manifest_session_directory_mismatch");
    }

    if (!manifest["occupancy_grid"] ||
        !manifest["pose_graph"])
    {
      return fail(
        std::move(result),
        "manifest_artifact_sections_missing");
    }

    const fs::path manifest_grid_yaml =
      resolve_path(
      result.paths.manifest.parent_path(),
      required_string(
        manifest["occupancy_grid"],
        "yaml"));

    if (!paths_match(
        manifest_grid_yaml,
        result.paths.grid_yaml))
    {
      return fail(
        std::move(result),
        "manifest_grid_yaml_mismatch");
    }

    const fs::path manifest_posegraph =
      resolve_path(
      result.paths.manifest.parent_path(),
      required_string(
        manifest["pose_graph"],
        "posegraph"));

    if (!paths_match(
        manifest_posegraph,
        result.paths.posegraph))
    {
      return fail(
        std::move(result),
        "manifest_posegraph_mismatch");
    }

    const fs::path manifest_posegraph_data =
      resolve_path(
      result.paths.manifest.parent_path(),
      required_string(
        manifest["pose_graph"],
        "data"));

    if (!paths_match(
        manifest_posegraph_data,
        result.paths.posegraph_data))
    {
      return fail(
        std::move(result),
        "manifest_posegraph_data_mismatch");
    }

    const YAML::Node grid_yaml =
      YAML::LoadFile(
      result.paths.grid_yaml.string());

    const std::string image_value =
      required_string(
      grid_yaml,
      "image");

    result.paths.grid_image =
      resolve_path(
      result.paths.grid_yaml.parent_path(),
      image_value);

    if (!path_is_within(
        session_directory,
        result.paths.grid_image))
    {
      return fail(
        std::move(result),
        "grid_image_outside_session");
    }

    if (result.paths.grid_image.stem() !=
        map_id_string)
    {
      return fail(
        std::move(result),
        "grid_image_map_id_mismatch");
    }

    const std::string extension =
      result.paths.grid_image.extension()
      .string();

    if (extension != ".pgm" &&
        extension != ".png")
    {
      return fail(
        std::move(result),
        "grid_image_extension_unsupported");
    }

    if (!is_nonempty_regular_file(
        result.paths.grid_image))
    {
      return fail(
        std::move(result),
        "grid_image_missing_or_empty");
    }

    const fs::path manifest_grid_image =
      resolve_path(
      result.paths.manifest.parent_path(),
      required_string(
        manifest["occupancy_grid"],
        "image"));

    if (!paths_match(
        manifest_grid_image,
        result.paths.grid_image))
    {
      return fail(
        std::move(result),
        "manifest_grid_image_mismatch");
    }

    if (!grid_yaml["resolution"] ||
        grid_yaml["resolution"].as<double>() <=
        0.0)
    {
      return fail(
        std::move(result),
        "grid_resolution_invalid");
    }

    if (!grid_yaml["origin"] ||
        !grid_yaml["origin"].IsSequence() ||
        grid_yaml["origin"].size() != 3)
    {
      return fail(
        std::move(result),
        "grid_origin_invalid");
    }

    result.grid_yaml_bytes =
      fs::file_size(
      result.paths.grid_yaml);

    result.grid_image_bytes =
      fs::file_size(
      result.paths.grid_image);

    result.posegraph_bytes =
      fs::file_size(
      result.paths.posegraph);

    result.posegraph_data_bytes =
      fs::file_size(
      result.paths.posegraph_data);

    result.valid = true;
    result.reason = "saved_map_valid";

    return result;
  } catch (const YAML::Exception & exception) {
    return fail(
      std::move(result),
      "yaml_parse_failed:" +
      std::string{exception.what()});
  } catch (const std::exception & exception) {
    return fail(
      std::move(result),
      "verification_failed:" +
      std::string{exception.what()});
  }
}

}  // namespace savo_mapping::session
