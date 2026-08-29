#pragma once

#include <cstdint>
#include <string>

#include "savo_localization/bno055_driver.hpp"

namespace savo_localization
{

constexpr int BNO055_CALIBRATION_SCHEMA_VERSION = 1;

struct BNO055CalibrationMetadata
{
  int i2c_bus{1};
  uint8_t i2c_address{BNO055_DEFAULT_ADDRESS};
  BNO055Mode operational_mode{BNO055Mode::NDOF};
  std::string captured_at;
};

struct BNO055CalibrationDocument
{
  int schema_version{BNO055_CALIBRATION_SCHEMA_VERSION};
  std::string sensor{"bno055"};
  BNO055CalibrationMetadata metadata{};
  BNO055CalibrationProfile calibration{};
};

enum class CalibrationProfileLoadStatus
{
  MISSING,
  LOADED,
  INVALID,
};

struct CalibrationProfileLoadResult
{
  CalibrationProfileLoadStatus status{CalibrationProfileLoadStatus::MISSING};
  BNO055CalibrationDocument document{};
  std::string error;
};

class BNO055CalibrationStore
{
public:
  explicit BNO055CalibrationStore(std::string path);

  const std::string & path() const;
  CalibrationProfileLoadResult load() const;
  bool save_atomic(
    const BNO055CalibrationDocument & document,
    std::string & error) const;

private:
  std::string path_;
};

struct BNO055CalibrationRuntimeState
{
  bool restore_enabled{true};
  bool verification_required{true};
  bool profile_present{false};
  bool profile_loaded{false};
  bool restore_attempted{false};
  bool profile_verified{false};
  bool operational_status_verified{false};
  bool restore_failed{false};
  std::string status{"not_attempted"};
  std::string error;
};

struct BNO055CalibrationCaptureResult
{
  bool success{false};
  std::string message;
  BNO055CalibrationProfile profile{};
};

class BNO055CalibrationRuntime
{
public:
  explicit BNO055CalibrationRuntime(std::string profile_path);

  const BNO055CalibrationRuntimeState & state() const;

  void restore(
    BNO055Driver & driver,
    bool enabled,
    bool verification_required,
    const BNO055CalibrationMetadata & expected_metadata);

  void verify_operational_status(BNO055Driver & driver);

  BNO055CalibrationCaptureResult capture(
    BNO055Driver & driver,
    const BNO055CalibrationMetadata & metadata);

private:
  BNO055CalibrationStore store_;
  BNO055CalibrationRuntimeState state_{};

  void mark_restore_failed(std::string error);
};

bool validate_bno055_calibration_document(
  const BNO055CalibrationDocument & document,
  std::string * error = nullptr);

std::string bno055_calibration_timestamp_utc();

}  // namespace savo_localization
