// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_supervisor/system_state_store.hpp"

#include <exception>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>

namespace savo_supervisor
{

SystemStateStore::SystemStateStore(std::string path)
: path_(std::move(path))
{
}

bool SystemStateStore::enabled() const noexcept
{
  return !path_.empty();
}

const std::string & SystemStateStore::path() const noexcept
{
  return path_;
}

PersistentSystemState SystemStateStore::Load() const
{
  PersistentSystemState result;
  if (!enabled()) {
    result.error = "persistent_state_disabled";
    return result;
  }
  try {
    std::error_code existence_error;
    const bool exists = std::filesystem::exists(path_, existence_error);
    if (existence_error) {
      result.error = "persistent_state_stat_failed:" + existence_error.message();
      return result;
    }
    if (!exists) {
      result.error = "persistent_state_not_found";
      return result;
    }
    std::ifstream stream(path_);
    if (!stream.is_open()) {
      result.error = "persistent_state_open_failed";
      return result;
    }
    nlohmann::json json;
    stream >> json;
    if (!json.is_object() || json.value("schema_version", 0) != 1) {
      result.error = "persistent_state_schema_invalid";
      return result;
    }
    result.fault_latched = json.at("fault_latched").get<bool>();
    result.generation = json.at("generation").get<std::uint64_t>();
    result.reason = json.value("reason", std::string{});
    result.valid = true;
  } catch (const std::exception & exception) {
    result.error = exception.what();
  }
  return result;
}

bool SystemStateStore::Save(
  const bool fault_latched,
  const std::uint64_t generation,
  const std::string & reason,
  std::string & error) const
{
  error.clear();
  if (!enabled()) {
    return true;
  }
  try {
    const std::filesystem::path destination(path_);
    if (destination.has_parent_path()) {
      std::filesystem::create_directories(destination.parent_path());
    }
    const auto temporary = destination.string() + ".tmp";
    {
      std::ofstream stream(temporary, std::ios::trunc);
      if (!stream.is_open()) {
        error = "persistent_state_temporary_open_failed";
        return false;
      }
      const nlohmann::json json{
        {"schema_version", 1},
        {"fault_latched", fault_latched},
        {"generation", generation},
        {"reason", reason}
      };
      stream << json.dump(2) << '\n';
      stream.flush();
      if (!stream.good()) {
        error = "persistent_state_write_failed";
        return false;
      }
    }
    std::error_code ec;
    std::filesystem::rename(temporary, destination, ec);
    if (ec) {
      std::filesystem::remove(destination, ec);
      ec.clear();
      std::filesystem::rename(temporary, destination, ec);
    }
    if (ec) {
      error = "persistent_state_rename_failed:" + ec.message();
      return false;
    }
    return true;
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace savo_supervisor
