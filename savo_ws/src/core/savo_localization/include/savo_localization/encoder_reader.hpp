#pragma once

#include <array>
#include <cstdint>
#include <memory>

#include "savo_localization/encoder_state.hpp"

namespace savo_localization
{

class EncoderBackend
{
public:
  virtual ~EncoderBackend() = default;

  virtual bool open(const EncoderHardwareConfig & config) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;

  virtual void poll() = 0;
  virtual std::array<std::int64_t, 4> counts() const = 0;
  virtual std::array<std::uint64_t, 4> illegal_transitions() const = 0;

  virtual void reset_counts() = 0;
};

class EncoderReader
{
public:
  EncoderReader(
    EncoderHardwareConfig config,
    std::unique_ptr<EncoderBackend> backend);

  ~EncoderReader();

  EncoderReader(const EncoderReader &) = delete;
  EncoderReader & operator=(const EncoderReader &) = delete;

  EncoderReader(EncoderReader &&) noexcept = default;
  EncoderReader & operator=(EncoderReader &&) noexcept = default;

  bool open();
  void close();
  bool is_open() const;

  void poll();

  EncoderSample sample(
    double stamp_s,
    double dt_s,
    double wheel_diameter_m,
    int counts_per_wheel_rev,
    double wheelbase_m,
    double track_m);

  void reset_counts();

  EncoderHardwareConfig config() const;
  std::array<std::int64_t, 4> counts() const;
  std::array<std::uint64_t, 4> illegal_transitions() const;

private:
  EncoderHardwareConfig config_{};
  std::unique_ptr<EncoderBackend> backend_;

  std::array<std::int64_t, 4> previous_counts_{0, 0, 0, 0};

  WheelEncoderState make_wheel_state(
    const WheelEncoderConfig & wheel_config,
    std::int64_t previous_count,
    std::int64_t current_count,
    std::uint64_t illegal_transitions,
    double dt_s,
    double wheel_diameter_m,
    int counts_per_wheel_rev,
    double stamp_s) const;

  static std::int64_t apply_inversion(
    std::int64_t count,
    bool inverted);

  static double count_delta_to_speed_mps(
    std::int64_t delta_count,
    double dt_s,
    double wheel_diameter_m,
    int counts_per_wheel_rev);
};

}  // namespace savo_localization