#include "savo_localization/encoder_reader.hpp"

#include <cmath>
#include <stdexcept>

namespace savo_localization
{

EncoderReader::EncoderReader(
  EncoderHardwareConfig config,
  std::unique_ptr<EncoderBackend> backend)
: config_(std::move(config)),
  backend_(std::move(backend))
{
  if (!backend_) {
    throw std::invalid_argument("EncoderReader requires a valid backend");
  }

  if (!config_.valid()) {
    throw std::invalid_argument("EncoderReader received invalid encoder config");
  }
}

EncoderReader::~EncoderReader()
{
  close();
}

bool EncoderReader::open()
{
  if (!backend_) {
    return false;
  }

  if (backend_->is_open()) {
    return true;
  }

  previous_counts_ = {0, 0, 0, 0};
  return backend_->open(config_);
}

void EncoderReader::close()
{
  if (backend_) {
    backend_->close();
  }
}

bool EncoderReader::is_open() const
{
  return backend_ && backend_->is_open();
}

void EncoderReader::poll()
{
  if (!is_open()) {
    throw std::runtime_error("EncoderReader backend is not open");
  }

  backend_->poll();
}

EncoderSample EncoderReader::sample(
  double stamp_s,
  double dt_s,
  double wheel_diameter_m,
  int counts_per_wheel_rev,
  double wheelbase_m,
  double track_m)
{
  if (!is_open()) {
    throw std::runtime_error("EncoderReader backend is not open");
  }

  if (dt_s <= 0.0) {
    throw std::invalid_argument("dt_s must be > 0.0");
  }

  if (wheel_diameter_m <= 0.0) {
    throw std::invalid_argument("wheel_diameter_m must be > 0.0");
  }

  if (counts_per_wheel_rev <= 0) {
    throw std::invalid_argument("counts_per_wheel_rev must be > 0");
  }

  if (wheelbase_m <= 0.0 || track_m <= 0.0) {
    throw std::invalid_argument("wheelbase_m and track_m must be > 0.0");
  }

  const auto current_counts = backend_->counts();
  const auto illegal = backend_->illegal_transitions();
  const auto wheel_configs = config_.wheels();

  EncoderSample result{};
  result.stamp_s = stamp_s;
  result.dt_s = dt_s;

  result.fl = make_wheel_state(
    wheel_configs[0],
    previous_counts_[0],
    current_counts[0],
    illegal[0],
    dt_s,
    wheel_diameter_m,
    counts_per_wheel_rev,
    stamp_s);

  result.fr = make_wheel_state(
    wheel_configs[1],
    previous_counts_[1],
    current_counts[1],
    illegal[1],
    dt_s,
    wheel_diameter_m,
    counts_per_wheel_rev,
    stamp_s);

  result.rl = make_wheel_state(
    wheel_configs[2],
    previous_counts_[2],
    current_counts[2],
    illegal[2],
    dt_s,
    wheel_diameter_m,
    counts_per_wheel_rev,
    stamp_s);

  result.rr = make_wheel_state(
    wheel_configs[3],
    previous_counts_[3],
    current_counts[3],
    illegal[3],
    dt_s,
    wheel_diameter_m,
    counts_per_wheel_rev,
    stamp_s);

  previous_counts_ = current_counts;

  const double fl = result.fl.speed_mps;
  const double fr = result.fr.speed_mps;
  const double rl = result.rl.speed_mps;
  const double rr = result.rr.speed_mps;

  const double radius_sum_m = wheelbase_m + track_m;

  result.vx_mps = (fl + fr + rl + rr) / 4.0;
  result.vy_mps = (-fl + fr + rl - rr) / 4.0;
  result.omega_rad_s = (-fl + fr - rl + rr) / (4.0 * radius_sum_m);

  return result;
}

void EncoderReader::reset_counts()
{
  if (!backend_) {
    return;
  }

  backend_->reset_counts();
  previous_counts_ = {0, 0, 0, 0};
}

EncoderHardwareConfig EncoderReader::config() const
{
  return config_;
}

std::array<std::int64_t, 4> EncoderReader::counts() const
{
  if (!backend_) {
    return {0, 0, 0, 0};
  }

  return backend_->counts();
}

std::array<std::uint64_t, 4> EncoderReader::illegal_transitions() const
{
  if (!backend_) {
    return {0, 0, 0, 0};
  }

  return backend_->illegal_transitions();
}

WheelEncoderState EncoderReader::make_wheel_state(
  const WheelEncoderConfig & wheel_config,
  std::int64_t previous_count,
  std::int64_t current_count,
  std::uint64_t illegal_transitions,
  double dt_s,
  double wheel_diameter_m,
  int counts_per_wheel_rev,
  double stamp_s) const
{
  const std::int64_t previous = apply_inversion(
    previous_count,
    wheel_config.inverted);

  const std::int64_t current = apply_inversion(
    current_count,
    wheel_config.inverted);

  const std::int64_t delta = current - previous;

  WheelEncoderState state{};
  state.wheel_id = wheel_config.wheel_id;
  state.name = wheel_config.name;
  state.count = previous;

  const double speed_mps = count_delta_to_speed_mps(
    delta,
    dt_s,
    wheel_diameter_m,
    counts_per_wheel_rev);

  state.update(
    current,
    dt_s,
    speed_mps,
    stamp_s,
    illegal_transitions);

  return state;
}

std::int64_t EncoderReader::apply_inversion(
  std::int64_t count,
  bool inverted)
{
  return inverted ? -count : count;
}

double EncoderReader::count_delta_to_speed_mps(
  std::int64_t delta_count,
  double dt_s,
  double wheel_diameter_m,
  int counts_per_wheel_rev)
{
  const double safe_dt_s = dt_s > 1e-9 ? dt_s : 1e-9;
  const double wheel_circumference_m = M_PI * wheel_diameter_m;
  const double revolutions =
    static_cast<double>(delta_count) / static_cast<double>(counts_per_wheel_rev);

  return (revolutions * wheel_circumference_m) / safe_dt_s;
}

}  // namespace savo_localization