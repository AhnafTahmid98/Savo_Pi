// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "savo_localization/producer_health.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace savo_localization
{
namespace
{

using Json = nlohmann::json;

double window_rate_hz(const std::deque<std::int64_t> & timestamps_ns) noexcept
{
  if (timestamps_ns.size() < 3U) {
    return 0.0;
  }

  const auto duration_ns = timestamps_ns.back() - timestamps_ns.front();
  if (duration_ns <= 0) {
    return 0.0;
  }

  return static_cast<double>(timestamps_ns.size() - 1U) * 1.0e9 /
         static_cast<double>(duration_ns);
}

bool valid_health_state(const std::string & state)
{
  return state == "INITIALIZING" || state == "OK" ||
         state == "DEGRADED" || state == "ERROR";
}

bool finite_nonnegative(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

bool ProducerRateTracker::RecordSuccess(
  const std::int64_t monotonic_time_ns,
  const std::int64_t source_stamp_ns,
  const std::size_t window_size)
{
  if (window_size < 3U) {
    throw std::invalid_argument("producer rate window must contain at least three samples");
  }
  if (monotonic_time_ns <= last_success_time_ns_) {
    throw std::invalid_argument("producer success times must be strictly monotonic");
  }

  success_times_ns_.push_back(monotonic_time_ns);
  while (success_times_ns_.size() > window_size) {
    success_times_ns_.pop_front();
  }
  last_success_time_ns_ = monotonic_time_ns;

  const bool timestamp_regressed =
    source_stamp_ns > 0 && last_source_stamp_ns_ > 0 &&
    source_stamp_ns < last_source_stamp_ns_;
  if (source_stamp_ns > 0) {
    last_source_stamp_ns_ = source_stamp_ns;
  }
  return timestamp_regressed;
}

ProducerRateObservation ProducerRateTracker::Observe(
  const std::int64_t monotonic_time_ns,
  const double expected_rate_hz) const
{
  if (!std::isfinite(expected_rate_hz) || expected_rate_hz <= 0.0) {
    throw std::invalid_argument("expected producer rate must be finite and positive");
  }

  ProducerRateObservation observation;
  observation.available = success_times_ns_.size() >= 3U;
  observation.rate_hz = window_rate_hz(success_times_ns_);
  observation.quality = ClassifyQuality(observation.rate_hz, expected_rate_hz);
  if (last_success_time_ns_ >= 0) {
    observation.last_success_age_s = std::max(
      0.0,
      static_cast<double>(monotonic_time_ns - last_success_time_ns_) / 1.0e9);
  }
  return observation;
}

RateQuality ProducerRateTracker::ClassifyQuality(
  const double rate_hz,
  const double expected_rate_hz) noexcept
{
  if (!std::isfinite(rate_hz) || !std::isfinite(expected_rate_hz) ||
    rate_hz < 0.0 || expected_rate_hz <= 0.0)
  {
    return RateQuality::kBelowMinimum;
  }

  const double ratio = rate_hz / expected_rate_hz;
  if (ratio >= 0.90) {
    return RateQuality::kExcellent;
  }
  if (ratio >= 0.75) {
    return RateQuality::kGood;
  }
  if (ratio >= 0.50) {
    return RateQuality::kMinimum;
  }
  return RateQuality::kBelowMinimum;
}

std::string_view ProducerRateTracker::QualityString(
  const RateQuality quality) noexcept
{
  switch (quality) {
    case RateQuality::kBelowMinimum:
      return "BELOW_MINIMUM";
    case RateQuality::kMinimum:
      return "MINIMUM";
    case RateQuality::kGood:
      return "GOOD";
    case RateQuality::kExcellent:
      return "EXCELLENT";
  }
  return "BELOW_MINIMUM";
}

bool RateValidityDebouncer::Observe(
  const std::int64_t monotonic_time_ns,
  const bool evidence_available,
  const bool instant_valid,
  const std::int64_t transition_debounce_ns)
{
  if (transition_debounce_ns < 0) {
    throw std::invalid_argument("rate transition debounce must be nonnegative");
  }
  if (!evidence_available) {
    return false;
  }

  if (instant_valid) {
    low_rate_since_ns_ = -1;
    debounced_rate_valid_ = true;
    established_valid_rate_ = true;
  } else {
    if (!established_valid_rate_) {
      debounced_rate_valid_ = false;
    } else if (debounced_rate_valid_) {
      if (low_rate_since_ns_ < 0) {
        low_rate_since_ns_ = monotonic_time_ns;
      }
      if (monotonic_time_ns - low_rate_since_ns_ >= transition_debounce_ns) {
        debounced_rate_valid_ = false;
      }
    }
  }

  return debounced_rate_valid_;
}

std::string SerializeProducerHealth(const ProducerHealthSnapshot & snapshot)
{
  Json payload{
    {"schema_version", snapshot.schema_version},
    {"node", snapshot.node},
    {"health_state", snapshot.health_state},
    {"reason", snapshot.reason},
    {"frame_id", snapshot.frame_id},
    {"parent_frame_id", snapshot.parent_frame_id},
    {"child_frame_id", snapshot.child_frame_id},
    {"data_valid", snapshot.data_valid},
    {"frame_valid", snapshot.frame_valid},
    {"timestamp_valid", snapshot.timestamp_valid},
    {"hardware_ok", snapshot.hardware_ok},
    {"motion_ready", snapshot.motion_ready},
    {"chip_id", snapshot.chip_id},
    {"system_status", snapshot.system_status},
    {"system_error", snapshot.system_error},
    {"calibration_system", snapshot.calibration_system},
    {"calibration_gyro", snapshot.calibration_gyro},
    {"calibration_accel", snapshot.calibration_accel},
    {"calibration_mag", snapshot.calibration_mag},
    {"producer_rate_available", snapshot.producer_rate_available},
    {"producer_rate_hz", snapshot.producer_rate_hz},
    {"last_success_age_s", snapshot.last_success_age_s},
    {"rate_quality", snapshot.rate_quality},
    {"sample_count", snapshot.sample_count},
    {"publish_count", snapshot.publish_count},
    {"error_count", snapshot.error_count},
    {"illegal_transition_count", snapshot.illegal_transition_count},
  };
  return payload.dump();
}

bool ParseProducerHealth(
  const std::string & payload,
  ProducerHealthSnapshot & snapshot,
  std::string & error)
{
  try {
    const auto object = Json::parse(payload);
    if (!object.is_object()) {
      error = "producer health payload must be a JSON object";
      return false;
    }

    ProducerHealthSnapshot parsed;
    parsed.schema_version = object.at("schema_version").get<int>();
    parsed.node = object.at("node").get<std::string>();
    parsed.health_state = object.at("health_state").get<std::string>();
    parsed.reason = object.at("reason").get<std::string>();
    parsed.frame_id = object.at("frame_id").get<std::string>();
    parsed.parent_frame_id = object.at("parent_frame_id").get<std::string>();
    parsed.child_frame_id = object.at("child_frame_id").get<std::string>();
    parsed.data_valid = object.at("data_valid").get<bool>();
    parsed.frame_valid = object.at("frame_valid").get<bool>();
    parsed.timestamp_valid = object.at("timestamp_valid").get<bool>();
    parsed.hardware_ok = object.at("hardware_ok").get<bool>();
    parsed.motion_ready = object.at("motion_ready").get<bool>();
    parsed.chip_id = object.at("chip_id").get<int>();
    parsed.system_status = object.at("system_status").get<int>();
    parsed.system_error = object.at("system_error").get<int>();
    parsed.calibration_system = object.at("calibration_system").get<int>();
    parsed.calibration_gyro = object.at("calibration_gyro").get<int>();
    parsed.calibration_accel = object.at("calibration_accel").get<int>();
    parsed.calibration_mag = object.at("calibration_mag").get<int>();
    parsed.producer_rate_available = object.at("producer_rate_available").get<bool>();
    parsed.producer_rate_hz = object.at("producer_rate_hz").get<double>();
    parsed.last_success_age_s = object.at("last_success_age_s").get<double>();
    parsed.rate_quality = object.at("rate_quality").get<std::string>();
    parsed.sample_count = object.at("sample_count").get<std::uint64_t>();
    parsed.publish_count = object.at("publish_count").get<std::uint64_t>();
    parsed.error_count = object.at("error_count").get<std::uint64_t>();
    parsed.illegal_transition_count =
      object.at("illegal_transition_count").get<std::uint64_t>();

    if (parsed.schema_version != 1) {
      error = "unsupported producer health schema version";
      return false;
    }
    if (parsed.node.empty() || !valid_health_state(parsed.health_state)) {
      error = "invalid producer health identity or state";
      return false;
    }
    if (!finite_nonnegative(parsed.producer_rate_hz) ||
      !std::isfinite(parsed.last_success_age_s) || parsed.last_success_age_s < -1.0)
    {
      error = "invalid producer rate or freshness value";
      return false;
    }

    snapshot = std::move(parsed);
    error.clear();
    return true;
  } catch (const Json::exception & exception) {
    error = std::string("invalid producer health payload: ") + exception.what();
    return false;
  }
}

void ProducerHealthConsumer::Record(
  const std::int64_t receive_time_ns,
  const std::string & payload,
  const std::size_t rate_window_size)
{
  if (rate_window_size < 3U) {
    throw std::invalid_argument(
        "producer health receive window must contain at least three samples");
  }
  received_ = true;
  if (receive_time_ns > last_receive_time_ns_) {
    receive_timestamps_ns_.push_back(receive_time_ns);
    while (receive_timestamps_ns_.size() > rate_window_size) {
      receive_timestamps_ns_.pop_front();
    }
    last_receive_time_ns_ = receive_time_ns;
  }

  ProducerHealthSnapshot parsed;
  std::string parse_error;
  payload_valid_ = ParseProducerHealth(payload, parsed, parse_error);
  if (payload_valid_) {
    snapshot_ = std::move(parsed);
    detail_ = snapshot_.reason;
  } else {
    snapshot_ = ProducerHealthSnapshot{};
    detail_ = std::move(parse_error);
  }
}

ConsumedProducerHealth ProducerHealthConsumer::Observe(
  const std::int64_t current_receive_time_ns,
  const double max_age_s) const
{
  if (!std::isfinite(max_age_s) || max_age_s <= 0.0) {
    throw std::invalid_argument("producer health max age must be finite and positive");
  }

  ConsumedProducerHealth observation;
  observation.received = received_;
  observation.payload_valid = payload_valid_;
  observation.snapshot = snapshot_;
  observation.detail = detail_;
  if (!received_ || last_receive_time_ns_ < 0) {
    return observation;
  }

  observation.receive_age_s = std::max(
    0.0,
    static_cast<double>(current_receive_time_ns - last_receive_time_ns_) / 1.0e9);
  if (payload_valid_ && snapshot_.last_success_age_s >= 0.0) {
    observation.producer_age_s =
      snapshot_.last_success_age_s + observation.receive_age_s;
  }
  observation.receive_rate_hz = window_rate_hz(receive_timestamps_ns_);
  observation.fresh = payload_valid_ && observation.receive_age_s <= max_age_s &&
    observation.producer_age_s >= 0.0 && observation.producer_age_s <= max_age_s;
  return observation;
}

bool ProducerHealthConsumer::ObserveRateValid(
  const std::int64_t current_receive_time_ns,
  const double expected_rate_hz,
  const double minimum_rate_ratio,
  const std::int64_t transition_debounce_ns)
{
  if (!std::isfinite(expected_rate_hz) || expected_rate_hz <= 0.0 ||
    !std::isfinite(minimum_rate_ratio) || minimum_rate_ratio <= 0.0 ||
    minimum_rate_ratio > 1.0)
  {
    throw std::invalid_argument("invalid producer health rate threshold");
  }
  const bool evidence_available = payload_valid_ && snapshot_.producer_rate_available;
  const bool instantaneous_valid = snapshot_.producer_rate_hz >=
    expected_rate_hz * minimum_rate_ratio;
  return rate_debouncer_.Observe(
    current_receive_time_ns, evidence_available, instantaneous_valid,
    transition_debounce_ns);
}

}  // namespace savo_localization
