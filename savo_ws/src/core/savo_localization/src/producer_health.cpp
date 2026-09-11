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

std::int64_t max_window_gap_ns(
  const std::deque<std::int64_t> & timestamps_ns) noexcept
{
  std::int64_t maximum_gap_ns = 0;
  for (std::size_t index = 1U; index < timestamps_ns.size(); ++index) {
    maximum_gap_ns = std::max(
      maximum_gap_ns, timestamps_ns[index] - timestamps_ns[index - 1U]);
  }
  return maximum_gap_ns;
}

double max_window_gap_s(const std::deque<std::int64_t> & timestamps_ns) noexcept
{
  return static_cast<double>(max_window_gap_ns(timestamps_ns)) / 1.0e9;
}

struct WindowRateEstimate
{
  double raw_rate_hz{0.0};
  double sustained_rate_hz{0.0};
  double max_gap_s{0.0};
  bool isolated_gap_excluded{false};
};

WindowRateEstimate estimate_window_rate(
  const std::deque<std::int64_t> & timestamps_ns,
  const double minimum_rate_hz) noexcept
{
  WindowRateEstimate estimate;
  estimate.raw_rate_hz = window_rate_hz(timestamps_ns);
  estimate.sustained_rate_hz = estimate.raw_rate_hz;
  if (timestamps_ns.size() < 2U) {
    return estimate;
  }

  const std::int64_t maximum_gap_ns = max_window_gap_ns(timestamps_ns);
  estimate.max_gap_s = static_cast<double>(maximum_gap_ns) / 1.0e9;

  if (timestamps_ns.size() < 4U || estimate.raw_rate_hz >= minimum_rate_hz) {
    return estimate;
  }

  // Freshness independently fails closed while a producer is stopped. Once it
  // is live again, do not let that one already-detected gap masquerade as a
  // sustained low publication rate for the rest of this count-based window.
  // Excluding one interval is allowed only when all remaining intervals,
  // evaluated together, satisfy the unchanged minimum rate.
  const std::int64_t duration_ns = timestamps_ns.back() - timestamps_ns.front();
  const std::int64_t sustained_duration_ns = duration_ns - maximum_gap_ns;
  if (sustained_duration_ns <= 0) {
    return estimate;
  }

  const std::size_t sustained_interval_count = timestamps_ns.size() - 2U;
  const double candidate_rate_hz =
    static_cast<double>(sustained_interval_count) * 1.0e9 /
    static_cast<double>(sustained_duration_ns);
  if (candidate_rate_hz >= minimum_rate_hz) {
    estimate.sustained_rate_hz = candidate_rate_hz;
    estimate.isolated_gap_excluded = true;
  }
  return estimate;
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
  const RateThresholds & thresholds) const
{
  ValidateThresholds(thresholds);

  ProducerRateObservation observation;
  observation.available = success_times_ns_.size() >= 3U;
  const auto estimate = estimate_window_rate(
    success_times_ns_, thresholds.minimum_hz);
  observation.rate_hz = estimate.sustained_rate_hz;
  observation.raw_window_rate_hz = estimate.raw_rate_hz;
  observation.max_inter_publication_gap_s = estimate.max_gap_s;
  observation.last_success_monotonic_ns = last_success_time_ns_;
  observation.window_sample_count = success_times_ns_.size();
  observation.isolated_gap_excluded = estimate.isolated_gap_excluded;
  observation.quality = ClassifyQuality(observation.rate_hz, thresholds);
  if (last_success_time_ns_ >= 0) {
    observation.last_success_age_s = std::max(
      0.0,
      static_cast<double>(monotonic_time_ns - last_success_time_ns_) / 1.0e9);
  }
  return observation;
}

void ProducerRateTracker::ValidateThresholds(const RateThresholds & thresholds)
{
  if (!std::isfinite(thresholds.minimum_hz) ||
    !std::isfinite(thresholds.good_hz) ||
    !std::isfinite(thresholds.excellent_hz) ||
    thresholds.minimum_hz <= 0.0 ||
    thresholds.minimum_hz > thresholds.good_hz ||
    thresholds.good_hz > thresholds.excellent_hz)
  {
    throw std::invalid_argument(
            "rate thresholds must satisfy 0 < minimum_hz <= good_hz <= excellent_hz");
  }
}

RateQuality ProducerRateTracker::ClassifyQuality(
  const double rate_hz,
  const RateThresholds & thresholds)
{
  ValidateThresholds(thresholds);
  if (!std::isfinite(rate_hz) || rate_hz < 0.0) {
    return RateQuality::kBelowMinimum;
  }

  if (rate_hz < thresholds.minimum_hz) {
    return RateQuality::kBelowMinimum;
  }
  if (rate_hz < thresholds.good_hz) {
    return RateQuality::kMinimum;
  }
  if (rate_hz < thresholds.excellent_hz) {
    return RateQuality::kGood;
  }
  return RateQuality::kExcellent;
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
    if (!established_valid_rate_) {
      valid_rate_since_ns_ = -1;
      debounced_rate_valid_ = true;
      established_valid_rate_ = true;
    } else if (!debounced_rate_valid_) {
      if (valid_rate_since_ns_ < 0) {
        valid_rate_since_ns_ = monotonic_time_ns;
      }
      if (monotonic_time_ns - valid_rate_since_ns_ >= transition_debounce_ns) {
        valid_rate_since_ns_ = -1;
        debounced_rate_valid_ = true;
      }
    } else {
      valid_rate_since_ns_ = -1;
    }
  } else {
    valid_rate_since_ns_ = -1;
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
    {"raw_window_rate_hz", snapshot.raw_window_rate_hz},
    {"last_success_age_s", snapshot.last_success_age_s},
    {"max_inter_publication_gap_s", snapshot.max_inter_publication_gap_s},
    {"last_success_monotonic_ns", snapshot.last_success_monotonic_ns},
    {"rate_window_sample_count", snapshot.rate_window_sample_count},
    {"isolated_gap_excluded", snapshot.isolated_gap_excluded},
    {"health_publish_monotonic_ns", snapshot.health_publish_monotonic_ns},
    {"health_publish_gap_s", snapshot.health_publish_gap_s},
    {"max_health_publish_gap_s", snapshot.max_health_publish_gap_s},
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
    parsed.raw_window_rate_hz = object.value(
      "raw_window_rate_hz", parsed.producer_rate_hz);
    parsed.last_success_age_s = object.at("last_success_age_s").get<double>();
    parsed.max_inter_publication_gap_s = object.value(
      "max_inter_publication_gap_s", 0.0);
    parsed.last_success_monotonic_ns = object.value(
      "last_success_monotonic_ns", static_cast<std::int64_t>(-1));
    parsed.rate_window_sample_count = object.value(
      "rate_window_sample_count", static_cast<std::uint64_t>(0U));
    parsed.isolated_gap_excluded = object.value("isolated_gap_excluded", false);
    parsed.health_publish_monotonic_ns = object.value(
      "health_publish_monotonic_ns", static_cast<std::int64_t>(-1));
    parsed.health_publish_gap_s = object.value("health_publish_gap_s", -1.0);
    parsed.max_health_publish_gap_s = object.value("max_health_publish_gap_s", 0.0);
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
      !finite_nonnegative(parsed.raw_window_rate_hz) ||
      !finite_nonnegative(parsed.max_inter_publication_gap_s) ||
      !std::isfinite(parsed.last_success_age_s) || parsed.last_success_age_s < -1.0 ||
      parsed.last_success_monotonic_ns < -1 || parsed.health_publish_monotonic_ns < -1 ||
      !std::isfinite(parsed.health_publish_gap_s) || parsed.health_publish_gap_s < -1.0 ||
      !finite_nonnegative(parsed.max_health_publish_gap_s))
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
  observation.max_receive_gap_s = max_window_gap_s(receive_timestamps_ns_);
  observation.fresh = payload_valid_ && observation.receive_age_s <= max_age_s &&
    observation.producer_age_s >= 0.0 && observation.producer_age_s <= max_age_s;
  return observation;
}

bool ProducerHealthConsumer::ObserveRateValid(
  const std::int64_t current_receive_time_ns,
  const RateThresholds & thresholds,
  const std::int64_t transition_debounce_ns)
{
  ProducerRateTracker::ValidateThresholds(thresholds);
  const bool evidence_available = payload_valid_ && snapshot_.producer_rate_available;
  const bool instantaneous_valid = ProducerRateTracker::ClassifyQuality(
    snapshot_.producer_rate_hz, thresholds) != RateQuality::kBelowMinimum;
  return rate_debouncer_.Observe(
    current_receive_time_ns, evidence_available, instantaneous_valid,
    transition_debounce_ns);
}

}  // namespace savo_localization
