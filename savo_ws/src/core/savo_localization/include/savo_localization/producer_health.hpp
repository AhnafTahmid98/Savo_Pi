// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <string_view>

namespace savo_localization
{

enum class RateQuality : std::uint8_t
{
  kBelowMinimum = 0,
  kMinimum,
  kGood,
  kExcellent
};

struct ProducerRateObservation
{
  bool available{false};
  double rate_hz{0.0};
  double last_success_age_s{-1.0};
  RateQuality quality{RateQuality::kBelowMinimum};
};

class ProducerRateTracker
{
public:
  bool RecordSuccess(
    std::int64_t monotonic_time_ns,
    std::int64_t source_stamp_ns,
    std::size_t window_size);

  [[nodiscard]] ProducerRateObservation Observe(
    std::int64_t monotonic_time_ns,
    double expected_rate_hz) const;

  [[nodiscard]] static RateQuality ClassifyQuality(
    double rate_hz,
    double expected_rate_hz) noexcept;

  [[nodiscard]] static std::string_view QualityString(
    RateQuality quality) noexcept;

private:
  std::deque<std::int64_t> success_times_ns_{};
  std::int64_t last_success_time_ns_{-1};
  std::int64_t last_source_stamp_ns_{-1};
};

class RateValidityDebouncer
{
public:
  [[nodiscard]] bool Observe(
    std::int64_t monotonic_time_ns,
    bool evidence_available,
    bool instant_valid,
    std::int64_t transition_debounce_ns);

private:
  std::int64_t low_rate_since_ns_{-1};
  bool established_valid_rate_{false};
  bool debounced_rate_valid_{false};
};

struct ProducerHealthSnapshot
{
  int schema_version{1};
  std::string node{};
  std::string health_state{"INITIALIZING"};
  std::string reason{"waiting_for_first_success"};
  std::string frame_id{};
  std::string parent_frame_id{};
  std::string child_frame_id{};
  bool data_valid{false};
  bool frame_valid{false};
  bool timestamp_valid{true};
  bool hardware_ok{false};
  bool motion_ready{false};
  int chip_id{0};
  int system_status{0};
  int system_error{0};
  int calibration_system{0};
  int calibration_gyro{0};
  int calibration_accel{0};
  int calibration_mag{0};
  bool producer_rate_available{false};
  double producer_rate_hz{0.0};
  double last_success_age_s{-1.0};
  std::string rate_quality{"BELOW_MINIMUM"};
  std::uint64_t sample_count{0U};
  std::uint64_t publish_count{0U};
  std::uint64_t error_count{0U};
  std::uint64_t illegal_transition_count{0U};
};

[[nodiscard]] std::string SerializeProducerHealth(
  const ProducerHealthSnapshot & snapshot);

[[nodiscard]] bool ParseProducerHealth(
  const std::string & payload,
  ProducerHealthSnapshot & snapshot,
  std::string & error);

struct ConsumedProducerHealth
{
  bool received{false};
  bool payload_valid{false};
  bool fresh{false};
  ProducerHealthSnapshot snapshot{};
  double receive_age_s{-1.0};
  double producer_age_s{-1.0};
  double receive_rate_hz{0.0};
  std::string detail{};
};

class ProducerHealthConsumer
{
public:
  void Record(
    std::int64_t receive_time_ns,
    const std::string & payload,
    std::size_t rate_window_size);

  [[nodiscard]] ConsumedProducerHealth Observe(
    std::int64_t current_receive_time_ns,
    double max_age_s) const;

  [[nodiscard]] bool ObserveRateValid(
    std::int64_t current_receive_time_ns,
    double expected_rate_hz,
    double minimum_rate_ratio,
    std::int64_t transition_debounce_ns);

private:
  bool received_{false};
  bool payload_valid_{false};
  ProducerHealthSnapshot snapshot_{};
  std::string detail_{};
  std::deque<std::int64_t> receive_timestamps_ns_{};
  std::int64_t last_receive_time_ns_{-1};
  RateValidityDebouncer rate_debouncer_{};
};

}  // namespace savo_localization
