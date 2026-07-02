#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "savo_head/core/head_types.hpp"
#include "savo_head/core/servo_calibration.hpp"
#include "savo_head/drivers/pca9685_driver.hpp"

namespace savo_head
{

inline constexpr const char * kHeadBackendPca9685 = "pca9685";
inline constexpr const char * kHeadBackendDryrun = "dryrun";

struct PanTiltDriverConfig
{
  std::string backend{kHeadBackendPca9685};
  int i2c_bus{kI2cBusDefault};
  int pca9685_address{kPca9685AddressDefault};
  double pwm_frequency_hz{kPca9685PwmFrequencyHzDefault};
  HeadServoCalibration calibration{};
  bool center_on_open{false};
  bool center_on_close{false};

  [[nodiscard]] PanTiltDriverConfig normalized() const
  {
    PanTiltDriverConfig out = *this;

    if (out.backend != kHeadBackendPca9685 && out.backend != kHeadBackendDryrun) {
      out.backend = kHeadBackendDryrun;
    }

    if (out.i2c_bus < 0) {
      out.i2c_bus = kI2cBusDefault;
    }

    if (out.pca9685_address <= 0 || out.pca9685_address > 0x7f) {
      out.pca9685_address = kPca9685AddressDefault;
    }

    if (out.pwm_frequency_hz <= 0.0) {
      out.pwm_frequency_hz = kPca9685PwmFrequencyHzDefault;
    }

    out.calibration = out.calibration.normalized();
    return out;
  }

  [[nodiscard]] bool dryrun() const
  {
    return normalized().backend == kHeadBackendDryrun;
  }

  [[nodiscard]] Pca9685Config pca9685_config() const
  {
    const auto item = normalized();

    return Pca9685Config{
      item.i2c_bus,
      item.pca9685_address,
      item.pwm_frequency_hz,
      item.dryrun()
    };
  }
};

class PanTiltDriver
{
public:
  explicit PanTiltDriver(PanTiltDriverConfig config = PanTiltDriverConfig{});
  PanTiltDriver(PanTiltDriverConfig config, std::unique_ptr<Pca9685Driver> pca_driver);
  ~PanTiltDriver();

  PanTiltDriver(const PanTiltDriver &) = delete;
  PanTiltDriver & operator=(const PanTiltDriver &) = delete;

  PanTiltDriver(PanTiltDriver && other) noexcept;
  PanTiltDriver & operator=(PanTiltDriver && other) noexcept;

  void open();
  void close();
  void ensure_open();

  [[nodiscard]] bool opened() const;
  [[nodiscard]] bool dryrun() const;
  [[nodiscard]] const PanTiltDriverConfig & config() const;
  [[nodiscard]] const HeadServoCalibration & calibration() const;
  [[nodiscard]] PanTiltLimits limits() const;
  [[nodiscard]] PanTiltState state() const;
  [[nodiscard]] const std::vector<ServoOutput> & last_outputs() const;
  [[nodiscard]] std::optional<std::string> last_error() const;
  [[nodiscard]] Pca9685Driver & pca9685();
  [[nodiscard]] const Pca9685Driver & pca9685() const;

  void clear_error();

  [[nodiscard]] ServoOutput set_axis(
    const std::string & axis,
    int angle_deg,
    HeadMode mode = HeadMode::kManual,
    CommandSource source = CommandSource::kManual,
    double stamp_s = 0.0);

  [[nodiscard]] PanTiltState set_pan_tilt(
    int pan_deg,
    int tilt_deg,
    HeadMode mode = HeadMode::kManual,
    CommandSource source = CommandSource::kManual,
    double stamp_s = 0.0);

  [[nodiscard]] PanTiltState center(
    CommandSource source = CommandSource::kSystem,
    double stamp_s = 0.0);

  [[nodiscard]] PanTiltState hold(
    CommandSource source = CommandSource::kSystem,
    double stamp_s = 0.0);

  [[nodiscard]] PanTiltState stop(
    CommandSource source = CommandSource::kSystem,
    double stamp_s = 0.0);

  [[nodiscard]] PanTiltState apply_command(const PanTiltCommand & command);

  [[nodiscard]] std::string status_text() const;

private:
  [[nodiscard]] ServoOutput write_axis(
    const std::string & axis,
    int angle_deg);

  void set_error(std::string message);
  void update_state(
    int pan_deg,
    int tilt_deg,
    HeadMode mode,
    HeadStatus status,
    CommandSource source,
    double stamp_s);

  PanTiltDriverConfig config_{};
  HeadServoCalibration calibration_{};
  std::unique_ptr<Pca9685Driver> pca_driver_{};
  bool opened_{false};
  PanTiltState state_{};
  std::vector<ServoOutput> last_outputs_{};
  std::optional<std::string> last_error_{};
};

[[nodiscard]] inline PanTiltDriverConfig dryrun_pantilt_config(bool center_on_open = false)
{
  PanTiltDriverConfig config;
  config.backend = kHeadBackendDryrun;
  config.center_on_open = center_on_open;
  config.center_on_close = false;
  return config.normalized();
}

[[nodiscard]] inline PanTiltDriverConfig pca9685_pantilt_config(
  bool center_on_open = true,
  bool center_on_close = true)
{
  PanTiltDriverConfig config;
  config.backend = kHeadBackendPca9685;
  config.center_on_open = center_on_open;
  config.center_on_close = center_on_close;
  return config.normalized();
}

[[nodiscard]] inline bool valid_head_backend(const std::string & backend)
{
  return backend == kHeadBackendPca9685 || backend == kHeadBackendDryrun;
}

[[nodiscard]] inline HeadStatus driver_status_for_backend(bool dryrun, bool opened)
{
  if (!opened) {
    return HeadStatus::kStale;
  }

  return dryrun ? HeadStatus::kDryrun : HeadStatus::kOk;
}

}  // namespace savo_head
