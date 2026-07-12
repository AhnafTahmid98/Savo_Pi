#include "savo_mapping/saved_map_quality.hpp"

#include <nav2_map_server/map_io.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace savo_mapping::quality
{

namespace
{

namespace fs = std::filesystem;

std::uint64_t unix_now_ns()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<
      std::chrono::nanoseconds>(
      std::chrono::system_clock::now()
      .time_since_epoch())
    .count());
}

double safe_ratio(
  const std::size_t numerator,
  const std::size_t denominator)
{
  if (denominator == 0U) {
    return 0.0;
  }

  return
    static_cast<double>(numerator) /
    static_cast<double>(denominator);
}

void append_failure(
  QualityEvaluation & evaluation,
  const bool condition,
  const std::string & reason)
{
  if (!condition) {
    evaluation.failed_checks.push_back(reason);
  }
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

    output << node;
    output << '\n';
    output.flush();

    if (!output.good()) {
      throw std::runtime_error(
              "cannot_write_temporary_yaml:" +
              temporary.string());
    }
  }

  std::error_code error;
  fs::rename(temporary, path, error);

  if (error) {
    fs::remove(temporary);

    throw std::runtime_error(
            "atomic_yaml_rename_failed:" +
            error.message());
  }
}

fs::path resolve_manifest_path(
  const fs::path & manifest,
  const std::string & value)
{
  fs::path path{value};

  if (path.is_relative()) {
    path = manifest.parent_path() / path;
  }

  return fs::weakly_canonical(path);
}

std::string map_load_failure_reason(
  const nav2_map_server::LOAD_MAP_STATUS status)
{
  switch (status) {
    case nav2_map_server::LOAD_MAP_SUCCESS:
      return "map_load_success";

    case nav2_map_server::MAP_DOES_NOT_EXIST:
      return "map_does_not_exist";

    case nav2_map_server::INVALID_MAP_METADATA:
      return "invalid_map_metadata";

    case nav2_map_server::INVALID_MAP_DATA:
      return "invalid_map_data";
  }

  return "unknown_map_load_failure";
}

std::size_t calculate_largest_free_component(
  const std::size_t width,
  const std::size_t height,
  const std::vector<std::int8_t> & data)
{
  if (width == 0U ||
      height == 0U ||
      data.size() != width * height)
  {
    return 0U;
  }

  std::vector<bool> visited(
    data.size(),
    false);

  std::size_t largest_component = 0U;

  for (std::size_t start = 0U;
    start < data.size();
    ++start)
  {
    if (visited[start] ||
        data[start] != 0)
    {
      continue;
    }

    std::deque<std::size_t> pending;
    pending.push_back(start);
    visited[start] = true;

    std::size_t component_size = 0U;

    while (!pending.empty()) {
      const std::size_t index =
        pending.front();

      pending.pop_front();
      ++component_size;

      const std::size_t x =
        index % width;

      const std::size_t y =
        index / width;

      const auto try_add =
        [&](const std::size_t neighbour)
        {
          if (!visited[neighbour] &&
              data[neighbour] == 0)
          {
            visited[neighbour] = true;
            pending.push_back(neighbour);
          }
        };

      if (x > 0U) {
        try_add(index - 1U);
      }

      if (x + 1U < width) {
        try_add(index + 1U);
      }

      if (y > 0U) {
        try_add(index - width);
      }

      if (y + 1U < height) {
        try_add(index + width);
      }
    }

    largest_component =
      std::max(
      largest_component,
      component_size);
  }

  return largest_component;
}

YAML::Node make_quality_report(
  const QualityEvaluation & evaluation)
{
  YAML::Node report;

  report["schema_version"] = 1;
  report["map_id"] = evaluation.map_id;
  report["frame_id"] = evaluation.frame_id;
  report["evaluated_unix_ns"] = unix_now_ns();
  report["passed"] = evaluation.passed;
  report["reason"] = evaluation.reason;

  auto policy = report["policy"];

  policy["min_width_cells"] =
    evaluation.policy.min_width_cells;

  policy["min_height_cells"] =
    evaluation.policy.min_height_cells;

  policy["min_known_cells"] =
    evaluation.policy.min_known_cells;

  policy["min_free_cells"] =
    evaluation.policy.min_free_cells;

  policy["min_occupied_cells"] =
    evaluation.policy.min_occupied_cells;

  policy["min_resolution_m"] =
    evaluation.policy.min_resolution_m;

  policy["max_resolution_m"] =
    evaluation.policy.max_resolution_m;

  policy["min_known_ratio"] =
    evaluation.policy.min_known_ratio;

  policy[
    "min_largest_free_component_ratio"] =
    evaluation.policy
    .min_largest_free_component_ratio;

  auto metrics = report["metrics"];

  metrics["width"] =
    evaluation.metrics.width;

  metrics["height"] =
    evaluation.metrics.height;

  metrics["resolution_m"] =
    evaluation.metrics.resolution_m;

  metrics["total_cells"] =
    evaluation.metrics.total_cells;

  metrics["known_cells"] =
    evaluation.metrics.known_cells;

  metrics["unknown_cells"] =
    evaluation.metrics.unknown_cells;

  metrics["free_cells"] =
    evaluation.metrics.free_cells;

  metrics["occupied_cells"] =
    evaluation.metrics.occupied_cells;

  metrics["known_ratio"] =
    evaluation.metrics.known_ratio;

  metrics["unknown_ratio"] =
    evaluation.metrics.unknown_ratio;

  metrics["free_ratio"] =
    evaluation.metrics.free_ratio;

  metrics["occupied_ratio_of_known"] =
    evaluation.metrics
    .occupied_ratio_of_known;

  metrics[
    "largest_free_component_cells"] =
    evaluation.metrics
    .largest_free_component_cells;

  metrics[
    "largest_free_component_ratio"] =
    evaluation.metrics
    .largest_free_component_ratio;

  metrics["mapped_area_m2"] =
    evaluation.metrics.mapped_area_m2;

  metrics["free_area_m2"] =
    evaluation.metrics.free_area_m2;

  auto failed_checks =
    report["failed_checks"];

  for (const auto & check :
    evaluation.failed_checks)
  {
    failed_checks.push_back(check);
  }

  return report;
}

}  // namespace

QualityEvaluation evaluate_occupancy_grid(
  const std::string & map_id,
  const std::string & frame_id,
  const nav_msgs::msg::OccupancyGrid & grid,
  const QualityPolicy & policy)
{
  QualityEvaluation evaluation;

  evaluation.evaluated = true;
  evaluation.map_id = map_id;
  evaluation.frame_id = frame_id;
  evaluation.policy = policy;

  auto & metrics = evaluation.metrics;

  metrics.width = grid.info.width;
  metrics.height = grid.info.height;
  metrics.resolution_m =
    grid.info.resolution;

  metrics.total_cells =
    metrics.width * metrics.height;

  if (grid.data.size() !=
      metrics.total_cells)
  {
    evaluation.reason =
      "quality_failed";

    evaluation.failed_checks.push_back(
      "occupancy_data_size_mismatch");

    return evaluation;
  }

  for (const std::int8_t value :
    grid.data)
  {
    if (value < 0) {
      ++metrics.unknown_cells;
    } else if (value == 0) {
      ++metrics.free_cells;
    } else {
      ++metrics.occupied_cells;
    }
  }

  metrics.known_cells =
    metrics.free_cells +
    metrics.occupied_cells;

  metrics.known_ratio =
    safe_ratio(
    metrics.known_cells,
    metrics.total_cells);

  metrics.unknown_ratio =
    safe_ratio(
    metrics.unknown_cells,
    metrics.total_cells);

  metrics.free_ratio =
    safe_ratio(
    metrics.free_cells,
    metrics.total_cells);

  metrics.occupied_ratio_of_known =
    safe_ratio(
    metrics.occupied_cells,
    metrics.known_cells);

  metrics.largest_free_component_cells =
    calculate_largest_free_component(
    metrics.width,
    metrics.height,
    grid.data);

  metrics.largest_free_component_ratio =
    safe_ratio(
    metrics.largest_free_component_cells,
    metrics.free_cells);

  const double cell_area =
    metrics.resolution_m *
    metrics.resolution_m;

  metrics.mapped_area_m2 =
    static_cast<double>(
    metrics.known_cells) *
    cell_area;

  metrics.free_area_m2 =
    static_cast<double>(
    metrics.free_cells) *
    cell_area;

  append_failure(
    evaluation,
    metrics.width >=
    policy.min_width_cells,
    "width_below_minimum");

  append_failure(
    evaluation,
    metrics.height >=
    policy.min_height_cells,
    "height_below_minimum");

  append_failure(
    evaluation,
    std::isfinite(
      metrics.resolution_m) &&
    metrics.resolution_m >=
      policy.min_resolution_m &&
    metrics.resolution_m <=
      policy.max_resolution_m,
    "resolution_out_of_range");

  append_failure(
    evaluation,
    metrics.known_cells >=
    policy.min_known_cells,
    "known_cells_below_minimum");

  append_failure(
    evaluation,
    metrics.free_cells >=
    policy.min_free_cells,
    "free_cells_below_minimum");

  append_failure(
    evaluation,
    metrics.occupied_cells >=
    policy.min_occupied_cells,
    "occupied_cells_below_minimum");

  append_failure(
    evaluation,
    metrics.known_ratio >=
    policy.min_known_ratio,
    "known_ratio_below_minimum");

  append_failure(
    evaluation,
    metrics.largest_free_component_ratio >=
    policy
    .min_largest_free_component_ratio,
    "free_space_connectivity_below_minimum");

  evaluation.passed =
    evaluation.failed_checks.empty();

  evaluation.reason =
    evaluation.passed ?
    "quality_passed" :
    "quality_failed";

  return evaluation;
}

QualityEvaluation evaluate_saved_map_session(
  const session::SavedMapVerification & verification,
  const QualityPolicy & policy)
{
  QualityEvaluation evaluation;

  evaluation.evaluated = true;
  evaluation.map_id =
    verification.map_id;

  evaluation.frame_id =
    verification.frame_id;

  evaluation.policy = policy;

  evaluation.report_path =
    verification.paths.session_directory /
    "quality_report.yaml";

  if (!verification.valid) {
    evaluation.reason =
      "saved_map_verification_failed";

    evaluation.failed_checks.push_back(
      verification.reason);

    return evaluation;
  }

  nav_msgs::msg::OccupancyGrid grid;

  const auto status =
    nav2_map_server::loadMapFromYaml(
    verification.paths.grid_yaml.string(),
    grid);

  if (status !=
      nav2_map_server::LOAD_MAP_SUCCESS)
  {
    evaluation.reason =
      "saved_map_load_failed";

    evaluation.failed_checks.push_back(
      map_load_failure_reason(status));

    return evaluation;
  }

  evaluation =
    evaluate_occupancy_grid(
    verification.map_id,
    verification.frame_id,
    grid,
    policy);

  evaluation.report_path =
    verification.paths.session_directory /
    "quality_report.yaml";

  return evaluation;
}

void persist_quality_evaluation(
  const session::SavedMapVerification & verification,
  const QualityEvaluation & evaluation)
{
  if (!verification.valid) {
    throw std::runtime_error(
            "cannot_persist_unverified_session");
  }

  write_yaml_atomic(
    evaluation.report_path,
    make_quality_report(evaluation));

  YAML::Node manifest =
    YAML::LoadFile(
    verification.paths.manifest.string());

  auto map_quality =
    manifest["map_quality"];

  map_quality["structurally_valid"] = true;
  map_quality["evaluated"] = true;
  map_quality["passed"] =
    evaluation.passed;

  map_quality["report"] =
    evaluation.report_path.string();

  map_quality["evaluated_unix_ns"] =
    unix_now_ns();

  map_quality["known_ratio"] =
    evaluation.metrics.known_ratio;

  map_quality[
    "largest_free_component_ratio"] =
    evaluation.metrics
    .largest_free_component_ratio;

  map_quality["failed_checks"] =
    YAML::Node(YAML::NodeType::Sequence);

  for (const auto & check :
    evaluation.failed_checks)
  {
    map_quality["failed_checks"]
      .push_back(check);
  }

  manifest["navigation_handoff_ready"] =
    false;

  auto handoff =
    manifest["navigation_handoff"];

  handoff["contract_version"] = 1;
  handoff["map_id"] =
    verification.map_id;

  handoff["frame_id"] =
    verification.frame_id;

  handoff["approved"] = false;

  handoff["reason"] =
    evaluation.passed ?
    "quality_passed_approval_required" :
    "quality_failed";

  handoff["map_yaml"] =
    verification.paths.grid_yaml.string();

  handoff["quality_report"] =
    evaluation.report_path.string();

  write_yaml_atomic(
    verification.paths.manifest,
    manifest);
}

NavigationHandoff read_navigation_handoff(
  const session::SavedMapVerification & verification)
{
  NavigationHandoff handoff;

  handoff.map_id =
    verification.map_id;

  handoff.frame_id =
    verification.frame_id;

  handoff.map_yaml =
    verification.paths.grid_yaml;

  if (!verification.valid) {
    handoff.reason =
      verification.reason;

    return handoff;
  }

  const YAML::Node manifest =
    YAML::LoadFile(
    verification.paths.manifest.string());

  handoff.ready =
    manifest["navigation_handoff_ready"] &&
    manifest[
      "navigation_handoff_ready"].as<bool>();

  const YAML::Node contract =
    manifest["navigation_handoff"];

  if (!contract) {
    handoff.reason =
      "handoff_contract_missing";

    return handoff;
  }

  if (contract["contract_version"]) {
    handoff.contract_version =
      contract[
        "contract_version"].as<int>();
  }

  if (contract["approved"]) {
    handoff.approved =
      contract["approved"].as<bool>();
  }

  if (contract["reason"]) {
    handoff.reason =
      contract["reason"].as<std::string>();
  }

  if (contract["map_yaml"]) {
    handoff.map_yaml =
      resolve_manifest_path(
      verification.paths.manifest,
      contract["map_yaml"]
      .as<std::string>());
  }

  if (contract["quality_report"]) {
    handoff.quality_report =
      resolve_manifest_path(
      verification.paths.manifest,
      contract["quality_report"]
      .as<std::string>());
  }

  return handoff;
}

NavigationHandoff set_navigation_handoff(
  const session::SavedMapVerification & verification,
  const bool approved,
  const std::string & reason)
{
  if (!verification.valid) {
    throw std::runtime_error(
            "cannot_modify_unverified_session");
  }

  YAML::Node manifest =
    YAML::LoadFile(
    verification.paths.manifest.string());

  const YAML::Node map_quality =
    manifest["map_quality"];

  if (approved) {
    if (!map_quality ||
        !map_quality["evaluated"] ||
        !map_quality["evaluated"].as<bool>() ||
        !map_quality["passed"] ||
        !map_quality["passed"].as<bool>())
    {
      throw std::runtime_error(
              "quality_evaluation_not_passed");
    }

    if (!map_quality["report"]) {
      throw std::runtime_error(
              "quality_report_missing");
    }

    const fs::path report_path =
      resolve_manifest_path(
      verification.paths.manifest,
      map_quality["report"]
      .as<std::string>());

    if (!fs::is_regular_file(report_path)) {
      throw std::runtime_error(
              "quality_report_not_found");
    }

    const YAML::Node report =
      YAML::LoadFile(
      report_path.string());

    if (!report["passed"] ||
        !report["passed"].as<bool>() ||
        !report["map_id"] ||
        report["map_id"].as<std::string>() !=
          verification.map_id)
    {
      throw std::runtime_error(
              "quality_report_not_approved");
    }
  }

  manifest["navigation_handoff_ready"] =
    approved;

  auto contract =
    manifest["navigation_handoff"];

  contract["contract_version"] = 1;
  contract["map_id"] =
    verification.map_id;

  contract["frame_id"] =
    verification.frame_id;

  contract["approved"] =
    approved;

  contract["reason"] =
    reason;

  contract["map_yaml"] =
    verification.paths.grid_yaml.string();

  if (map_quality &&
      map_quality["report"])
  {
    contract["quality_report"] =
      map_quality["report"].as<std::string>();
  }

  write_yaml_atomic(
    verification.paths.manifest,
    manifest);

  return read_navigation_handoff(
    verification);
}

std::string quality_evaluation_to_json(
  const QualityEvaluation & evaluation)
{
  std::ostringstream output;

  output
    << std::boolalpha
    << std::fixed
    << std::setprecision(6)
    << "{"
    << "\"evaluated\":"
    << evaluation.evaluated
    << ",\"passed\":"
    << evaluation.passed
    << ",\"map_id\":\""
    << escape_json(evaluation.map_id)
    << "\",\"frame_id\":\""
    << escape_json(evaluation.frame_id)
    << "\",\"reason\":\""
    << escape_json(evaluation.reason)
    << "\",\"report\":\""
    << escape_json(
         evaluation.report_path.string())
    << "\",\"metrics\":{"
    << "\"width\":"
    << evaluation.metrics.width
    << ",\"height\":"
    << evaluation.metrics.height
    << ",\"resolution_m\":"
    << evaluation.metrics.resolution_m
    << ",\"known_cells\":"
    << evaluation.metrics.known_cells
    << ",\"unknown_cells\":"
    << evaluation.metrics.unknown_cells
    << ",\"free_cells\":"
    << evaluation.metrics.free_cells
    << ",\"occupied_cells\":"
    << evaluation.metrics.occupied_cells
    << ",\"known_ratio\":"
    << evaluation.metrics.known_ratio
    << ",\"largest_free_component_ratio\":"
    << evaluation.metrics
       .largest_free_component_ratio
    << ",\"mapped_area_m2\":"
    << evaluation.metrics.mapped_area_m2
    << "},\"failed_checks\":[";

  for (std::size_t index = 0U;
    index < evaluation.failed_checks.size();
    ++index)
  {
    if (index > 0U) {
      output << ',';
    }

    output
      << '"'
      << escape_json(
           evaluation.failed_checks[index])
      << '"';
  }

  output << "]}";

  return output.str();
}

std::string navigation_handoff_to_json(
  const NavigationHandoff & handoff)
{
  std::ostringstream output;

  output
    << std::boolalpha
    << "{"
    << "\"contract_version\":"
    << handoff.contract_version
    << ",\"ready\":"
    << handoff.ready
    << ",\"approved\":"
    << handoff.approved
    << ",\"map_id\":\""
    << escape_json(handoff.map_id)
    << "\",\"frame_id\":\""
    << escape_json(handoff.frame_id)
    << "\",\"reason\":\""
    << escape_json(handoff.reason)
    << "\",\"map_yaml\":\""
    << escape_json(
         handoff.map_yaml.string())
    << "\",\"quality_report\":\""
    << escape_json(
         handoff.quality_report.string())
    << "\"}";

  return output.str();
}

}  // namespace savo_mapping::quality
