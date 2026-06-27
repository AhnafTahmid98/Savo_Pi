#ifndef SAVO_PERCEPTION__PERCEPTION_TYPES_HPP_
#define SAVO_PERCEPTION__PERCEPTION_TYPES_HPP_

#include "savo_perception/constants.hpp"
#include "savo_perception/range_sample.hpp"
#include "savo_perception/safety_state.hpp"
#include "savo_perception/topic_names.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

enum class SensorStatus : std::uint8_t
{
  kOk = 0,
  kStale = 1,
  kError = 2,
};

struct SAVO_PERCEPTION_PUBLIC SensorHealth
{
  std::string sensor_name;
  SensorStatus status{SensorStatus::kError};

  bool ok{false};
  bool stale{true};
  bool valid{false};

  std::optional<double> last_distance_m;
  double age_s{0.0};

  std::string error;
  std::string source;
};

inline const char * to_string(const SensorStatus status)
{
  switch (status) {
    case SensorStatus::kOk:
      return "OK";
    case SensorStatus::kStale:
      return "STALE";
    case SensorStatus::kError:
      return "ERROR";
  }

  return "ERROR";
}

inline SensorHealth make_sensor_health(
  const RangeSample & sample,
  const double stale_timeout_s,
  const SteadyTimePoint & now = std::chrono::steady_clock::now())
{
  SensorHealth health;
  health.sensor_name = sample.sensor_name;
  health.valid = sample.valid;
  health.last_distance_m = sample.distance_m;
  health.age_s = sample.age_s(now);
  health.stale = sample.stale(stale_timeout_s, now);
  health.error = sample.error;
  health.source = sample.source;

  if (health.stale) {
    health.status = SensorStatus::kStale;
    health.ok = false;
  } else if (!sample.valid || !sample.distance_m.has_value()) {
    health.status = SensorStatus::kError;
    health.ok = false;
  } else {
    health.status = SensorStatus::kOk;
    health.ok = true;
  }

  return health;
}

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__PERCEPTION_TYPES_HPP_