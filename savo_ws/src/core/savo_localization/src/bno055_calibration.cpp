#include "savo_localization/bno055_calibration.hpp"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <unistd.h>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace savo_localization
{

namespace
{

namespace fs = std::filesystem;

std::string system_error_text(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

bool metadata_matches(
  const BNO055CalibrationMetadata & actual,
  const BNO055CalibrationMetadata & expected,
  std::string & error)
{
  if (actual.i2c_bus != expected.i2c_bus) {
    error = "profile i2c_bus does not match configured BNO055 bus";
    return false;
  }
  if (actual.i2c_address != expected.i2c_address) {
    error = "profile i2c_address does not match configured BNO055 address";
    return false;
  }
  if (actual.operational_mode != expected.operational_mode) {
    error = "profile operational_mode does not match configured BNO055 mode";
    return false;
  }
  return true;
}

int64_t required_integer(const YAML::Node & node, const char * key)
{
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error(std::string("missing integer field: ") + key);
  }
  return value.as<int64_t>();
}

std::string required_string(const YAML::Node & node, const char * key)
{
  const YAML::Node value = node[key];
  if (!value || !value.IsScalar()) {
    throw std::runtime_error(std::string("missing string field: ") + key);
  }
  const std::string result = value.as<std::string>();
  if (result.empty()) {
    throw std::runtime_error(std::string("empty string field: ") + key);
  }
  return result;
}

int16_t checked_i16(
  const YAML::Node & node,
  const char * key,
  const int64_t minimum,
  const int64_t maximum)
{
  const int64_t value = required_integer(node, key);
  if (value < minimum || value > maximum) {
    throw std::runtime_error(std::string("out-of-range calibration field: ") + key);
  }
  return static_cast<int16_t>(value);
}

uint16_t checked_u16(
  const YAML::Node & node,
  const char * key,
  const int64_t minimum,
  const int64_t maximum)
{
  const int64_t value = required_integer(node, key);
  if (value < minimum || value > maximum) {
    throw std::runtime_error(std::string("out-of-range calibration field: ") + key);
  }
  return static_cast<uint16_t>(value);
}

BNO055Mode parse_mode(const std::string & value)
{
  if (value == "ndof") {
    return BNO055Mode::NDOF;
  }
  if (value == "imu") {
    return BNO055Mode::IMU;
  }
  throw std::runtime_error("unsupported calibration operational_mode: " + value);
}

BNO055CalibrationDocument parse_document(const YAML::Node & root)
{
  if (!root || !root.IsMap()) {
    throw std::runtime_error("calibration profile root must be a YAML mapping");
  }

  BNO055CalibrationDocument document;
  const int64_t schema_version = required_integer(root, "schema_version");
  if (schema_version != BNO055_CALIBRATION_SCHEMA_VERSION) {
    throw std::runtime_error("unsupported calibration schema_version");
  }
  document.schema_version = static_cast<int>(schema_version);
  document.sensor = required_string(root, "sensor");
  if (document.sensor != "bno055") {
    throw std::runtime_error("calibration sensor must be bno055");
  }

  const YAML::Node metadata = root["metadata"];
  const YAML::Node calibration = root["calibration"];
  if (!metadata || !metadata.IsMap()) {
    throw std::runtime_error("calibration profile metadata must be a mapping");
  }
  if (!calibration || !calibration.IsMap()) {
    throw std::runtime_error("calibration profile calibration must be a mapping");
  }

  const int64_t i2c_bus = required_integer(metadata, "i2c_bus");
  const int64_t i2c_address = required_integer(metadata, "i2c_address");
  if (i2c_bus < 0 || i2c_bus > 255) {
    throw std::runtime_error("metadata i2c_bus is out of range");
  }
  if (i2c_address < 0 || i2c_address > 0x7F) {
    throw std::runtime_error("metadata i2c_address is not a valid 7-bit address");
  }
  document.metadata.i2c_bus = static_cast<int>(i2c_bus);
  document.metadata.i2c_address = static_cast<uint8_t>(i2c_address);
  document.metadata.operational_mode = parse_mode(
    required_string(metadata, "operational_mode"));
  document.metadata.captured_at = required_string(metadata, "captured_at");

  const YAML::Node accel = calibration["accel_offset"];
  const YAML::Node mag = calibration["mag_offset"];
  const YAML::Node gyro = calibration["gyro_offset"];
  if (!accel || !accel.IsMap() || !mag || !mag.IsMap() ||
    !gyro || !gyro.IsMap())
  {
    throw std::runtime_error("calibration offset groups must be mappings");
  }

  document.calibration.accel_offset_x = checked_i16(accel, "x", -500, 500);
  document.calibration.accel_offset_y = checked_i16(accel, "y", -500, 500);
  document.calibration.accel_offset_z = checked_i16(accel, "z", -500, 500);
  document.calibration.mag_offset_x = checked_i16(mag, "x", -6400, 6400);
  document.calibration.mag_offset_y = checked_i16(mag, "y", -6400, 6400);
  document.calibration.mag_offset_z = checked_i16(mag, "z", -6400, 6400);
  document.calibration.gyro_offset_x = checked_i16(gyro, "x", -2000, 2000);
  document.calibration.gyro_offset_y = checked_i16(gyro, "y", -2000, 2000);
  document.calibration.gyro_offset_z = checked_i16(gyro, "z", -2000, 2000);
  document.calibration.accel_radius = checked_u16(
    calibration, "accel_radius", 0, 2048);
  document.calibration.mag_radius = checked_u16(
    calibration, "mag_radius", 144, 1280);
  return document;
}

std::string emit_document(const BNO055CalibrationDocument & document)
{
  YAML::Emitter output;
  output << YAML::BeginMap;
  output << YAML::Key << "schema_version" << YAML::Value << document.schema_version;
  output << YAML::Key << "sensor" << YAML::Value << document.sensor;
  output << YAML::Key << "metadata" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "i2c_bus" << YAML::Value << document.metadata.i2c_bus;
  output << YAML::Key << "i2c_address" << YAML::Value <<
    static_cast<int>(document.metadata.i2c_address);
  output << YAML::Key << "operational_mode" << YAML::Value <<
    BNO055Driver::mode_name(document.metadata.operational_mode);
  output << YAML::Key << "captured_at" << YAML::Value << document.metadata.captured_at;
  output << YAML::EndMap;
  output << YAML::Key << "calibration" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "accel_offset" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "x" << YAML::Value << document.calibration.accel_offset_x;
  output << YAML::Key << "y" << YAML::Value << document.calibration.accel_offset_y;
  output << YAML::Key << "z" << YAML::Value << document.calibration.accel_offset_z;
  output << YAML::EndMap;
  output << YAML::Key << "mag_offset" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "x" << YAML::Value << document.calibration.mag_offset_x;
  output << YAML::Key << "y" << YAML::Value << document.calibration.mag_offset_y;
  output << YAML::Key << "z" << YAML::Value << document.calibration.mag_offset_z;
  output << YAML::EndMap;
  output << YAML::Key << "gyro_offset" << YAML::Value << YAML::BeginMap;
  output << YAML::Key << "x" << YAML::Value << document.calibration.gyro_offset_x;
  output << YAML::Key << "y" << YAML::Value << document.calibration.gyro_offset_y;
  output << YAML::Key << "z" << YAML::Value << document.calibration.gyro_offset_z;
  output << YAML::EndMap;
  output << YAML::Key << "accel_radius" << YAML::Value << document.calibration.accel_radius;
  output << YAML::Key << "mag_radius" << YAML::Value << document.calibration.mag_radius;
  output << YAML::EndMap;
  output << YAML::EndMap;
  if (!output.good()) {
    throw std::runtime_error("calibration YAML serialization failed");
  }
  return std::string(output.c_str()) + "\n";
}

void write_all(const int fd, const std::string & contents)
{
  const char * cursor = contents.data();
  std::size_t remaining = contents.size();
  while (remaining > 0U) {
    const ssize_t written = ::write(fd, cursor, remaining);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(system_error_text("calibration profile write failed"));
    }
    if (written == 0) {
      throw std::runtime_error("calibration profile write returned zero bytes");
    }
    cursor += written;
    remaining -= static_cast<std::size_t>(written);
  }
}

void fsync_directory(const fs::path & directory)
{
  const int directory_fd = ::open(
    directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (directory_fd < 0) {
    throw std::runtime_error(system_error_text("calibration profile directory open failed"));
  }
  const int sync_result = ::fsync(directory_fd);
  const int sync_errno = errno;
  const int close_result = ::close(directory_fd);
  if (sync_result != 0) {
    errno = sync_errno;
    throw std::runtime_error(system_error_text("calibration profile directory fsync failed"));
  }
  if (close_result != 0) {
    throw std::runtime_error(system_error_text("calibration profile directory close failed"));
  }
}

}  // namespace

BNO055CalibrationStore::BNO055CalibrationStore(std::string path)
: path_(std::move(path))
{
}

const std::string & BNO055CalibrationStore::path() const
{
  return path_;
}

CalibrationProfileLoadResult BNO055CalibrationStore::load() const
{
  CalibrationProfileLoadResult result;
  if (path_.empty()) {
    result.status = CalibrationProfileLoadStatus::INVALID;
    result.error = "calibration profile path is empty";
    return result;
  }

  std::error_code existence_error;
  if (!fs::exists(path_, existence_error)) {
    if (existence_error) {
      result.status = CalibrationProfileLoadStatus::INVALID;
      result.error = "calibration profile stat failed: " + existence_error.message();
    } else {
      result.status = CalibrationProfileLoadStatus::MISSING;
    }
    return result;
  }

  try {
    result.document = parse_document(YAML::LoadFile(path_));
    std::string validation_error;
    if (!validate_bno055_calibration_document(result.document, &validation_error)) {
      throw std::runtime_error(validation_error);
    }
    result.status = CalibrationProfileLoadStatus::LOADED;
  } catch (const std::exception & exception) {
    result.status = CalibrationProfileLoadStatus::INVALID;
    result.error = exception.what();
  }
  return result;
}

bool BNO055CalibrationStore::save_atomic(
  const BNO055CalibrationDocument & document,
  std::string & error) const
{
  error.clear();
  std::string validation_error;
  if (!validate_bno055_calibration_document(document, &validation_error)) {
    error = "calibration profile validation failed: " + validation_error;
    return false;
  }
  if (path_.empty()) {
    error = "calibration profile path is empty";
    return false;
  }

  fs::path temporary;
  fs::path backup;
  fs::path destination;
  fs::path parent;
  int fd = -1;
  bool destination_replaced = false;
  try {
    destination = path_;
    parent = destination.has_parent_path() ?
      destination.parent_path() : fs::path(".");
    fs::create_directories(parent);
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    temporary = destination.string() + ".tmp." + std::to_string(::getpid()) + "." +
      std::to_string(nonce);

    fd = ::open(
      temporary.c_str(),
      O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
      0640);
    if (fd < 0) {
      throw std::runtime_error(system_error_text("calibration temporary file open failed"));
    }

    write_all(fd, emit_document(document));
    if (::fsync(fd) != 0) {
      throw std::runtime_error(system_error_text("calibration temporary file fsync failed"));
    }
    if (::close(fd) != 0) {
      fd = -1;
      throw std::runtime_error(system_error_text("calibration temporary file close failed"));
    }
    fd = -1;

    std::error_code existence_error;
    const bool destination_exists = fs::exists(destination, existence_error);
    if (existence_error) {
      throw std::runtime_error(
              "calibration destination stat failed: " + existence_error.message());
    }
    if (destination_exists) {
      backup = destination.string() + ".backup." + std::to_string(::getpid()) + "." +
        std::to_string(nonce);
      fs::create_hard_link(destination, backup);
    }

    if (::rename(temporary.c_str(), destination.c_str()) != 0) {
      throw std::runtime_error(system_error_text("calibration profile atomic rename failed"));
    }
    destination_replaced = true;
    temporary.clear();

    fsync_directory(parent);
    if (!backup.empty()) {
      std::error_code remove_error;
      fs::remove(backup, remove_error);
      if (remove_error) {
        throw std::runtime_error(
                "calibration backup cleanup failed: " + remove_error.message());
      }
      backup.clear();
    }
    return true;
  } catch (const std::exception & exception) {
    if (fd >= 0) {
      ::close(fd);
    }
    if (!temporary.empty()) {
      std::error_code cleanup_error;
      fs::remove(temporary, cleanup_error);
    }
    if (destination_replaced) {
      std::error_code rollback_error;
      if (!backup.empty()) {
        fs::rename(backup, destination, rollback_error);
      } else {
        fs::remove(destination, rollback_error);
      }
      if (!parent.empty()) {
        try {
          fsync_directory(parent);
        } catch (const std::exception &) {
          // Preserve the original save failure in the returned diagnostic.
        }
      }
      if (rollback_error) {
        error = std::string(exception.what()) +
          "; preserving the previous profile also failed: " + rollback_error.message();
        return false;
      }
    } else if (!backup.empty()) {
      std::error_code cleanup_error;
      fs::remove(backup, cleanup_error);
    }
    error = exception.what();
    return false;
  }
}

BNO055CalibrationRuntime::BNO055CalibrationRuntime(std::string profile_path)
: store_(std::move(profile_path))
{
}

const BNO055CalibrationRuntimeState & BNO055CalibrationRuntime::state() const
{
  return state_;
}

void BNO055CalibrationRuntime::restore(
  BNO055Driver & driver,
  const bool enabled,
  const bool verification_required,
  const BNO055CalibrationMetadata & expected_metadata)
{
  state_ = BNO055CalibrationRuntimeState{};
  state_.restore_enabled = enabled;
  state_.verification_required = verification_required;
  if (!enabled) {
    state_.status = "disabled";
    return;
  }

  const CalibrationProfileLoadResult loaded = store_.load();
  if (loaded.status == CalibrationProfileLoadStatus::MISSING) {
    state_.status = "profile_missing";
    return;
  }
  state_.profile_present = true;
  if (loaded.status == CalibrationProfileLoadStatus::INVALID) {
    mark_restore_failed("invalid calibration profile: " + loaded.error);
    return;
  }
  state_.profile_loaded = true;

  std::string compatibility_error;
  if (!metadata_matches(
      loaded.document.metadata, expected_metadata, compatibility_error))
  {
    mark_restore_failed(compatibility_error);
    return;
  }

  state_.restore_attempted = true;
  try {
    driver.write_calibration_profile(loaded.document.calibration);
    state_.profile_verified = true;
    state_.status = "profile_verified";
  } catch (const std::exception & exception) {
    mark_restore_failed(exception.what());
  }
}

void BNO055CalibrationRuntime::verify_operational_status(BNO055Driver & driver)
{
  if (!state_.profile_verified || state_.restore_failed) {
    return;
  }
  try {
    const BNO055Status status = driver.read_status();
    if (status.chip_id != BNO055_CHIP_ID) {
      mark_restore_failed("unexpected BNO055 chip id after calibration restore");
      return;
    }
    if (status.system_error != 0U) {
      mark_restore_failed("BNO055 system error after calibration restore: " +
        std::to_string(status.system_error));
      return;
    }
    state_.operational_status_verified = true;
    state_.status = "restored";
  } catch (const std::exception & exception) {
    mark_restore_failed(
      "BNO055 operational status verification failed: " + std::string(exception.what()));
  }
}

BNO055CalibrationCaptureResult BNO055CalibrationRuntime::capture(
  BNO055Driver & driver,
  const BNO055CalibrationMetadata & metadata)
{
  BNO055CalibrationCaptureResult result;
  try {
    const ImuCalibration live_calibration = driver.read_calibration();
    if (live_calibration.system != 3 || live_calibration.gyro != 3 ||
      live_calibration.accel != 3 || live_calibration.mag != 3)
    {
      result.message = "calibration save rejected: live status must be exactly 3/3/3/3";
      return result;
    }

    result.profile = driver.read_calibration_profile();
    std::string validation_error;
    if (!validate_bno055_calibration_profile(result.profile, &validation_error)) {
      result.message = "calibration save rejected: " + validation_error;
      return result;
    }

    BNO055CalibrationDocument document;
    document.metadata = metadata;
    document.calibration = result.profile;
    std::string save_error;
    if (!store_.save_atomic(document, save_error)) {
      result.message = "calibration save failed: " + save_error;
      return result;
    }
    state_.profile_present = true;
    if (!state_.restore_failed && !state_.profile_verified) {
      state_.status = "profile_captured";
    }
    result.success = true;
    result.message = "BNO055 calibration profile saved atomically";
  } catch (const std::exception & exception) {
    result.message = "calibration save failed: " + std::string(exception.what());
  }
  return result;
}

void BNO055CalibrationRuntime::mark_restore_failed(std::string error)
{
  state_.restore_failed = true;
  state_.operational_status_verified = false;
  state_.status = "restore_failed";
  state_.error = std::move(error);
}

bool validate_bno055_calibration_document(
  const BNO055CalibrationDocument & document,
  std::string * error)
{
  auto fail = [error](const std::string & message) {
      if (error != nullptr) {
        *error = message;
      }
      return false;
    };

  if (document.schema_version != BNO055_CALIBRATION_SCHEMA_VERSION) {
    return fail("unsupported calibration schema_version");
  }
  if (document.sensor != "bno055") {
    return fail("calibration sensor must be bno055");
  }
  if (document.metadata.i2c_bus < 0 || document.metadata.i2c_bus > 255) {
    return fail("calibration metadata i2c_bus is out of range");
  }
  if (document.metadata.i2c_address > 0x7FU) {
    return fail("calibration metadata i2c_address is invalid");
  }
  if (document.metadata.operational_mode != BNO055Mode::NDOF &&
    document.metadata.operational_mode != BNO055Mode::IMU)
  {
    return fail("calibration metadata operational_mode must be ndof or imu");
  }
  if (document.metadata.captured_at.empty()) {
    return fail("calibration metadata captured_at is empty");
  }
  return validate_bno055_calibration_profile(document.calibration, error);
}

std::string bno055_calibration_timestamp_utc()
{
  const std::time_t now = std::chrono::system_clock::to_time_t(
    std::chrono::system_clock::now());
  std::tm utc{};
  if (::gmtime_r(&now, &utc) == nullptr) {
    throw std::runtime_error("failed to generate UTC calibration timestamp");
  }
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

}  // namespace savo_localization
