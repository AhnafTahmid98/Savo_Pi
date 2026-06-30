#ifndef SAVO_PERCEPTION__VL53L1X_DRIVER_HPP_
#define SAVO_PERCEPTION__VL53L1X_DRIVER_HPP_

#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "savo_perception/constants.hpp"
#include "savo_perception/range_sample.hpp"
#include "savo_perception/tca9548a.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC Vl53l1xConfig
{
  int bus{constants::kI2cBusDefault};
  std::uint8_t sensor_address{constants::kVl53l1xAddrDefault};

  int mux_channel{constants::kVl53RightChannelDefault};

  double settle_s{constants::kVl53SettleSDefault};
  double init_settle_s{constants::kVl53InitSettleSDefault};

  double valid_min_m{constants::kVl53ValidMinMDefault};
  double valid_max_m{constants::kVl53ValidMaxMDefault};

  int median_window{constants::kVl53MedianWindowDefault};

  std::string sensor_name{"tof"};
  std::string source{"vl53l1x"};
};

struct SAVO_PERCEPTION_PUBLIC Vl53l1xReading
{
  std::string sensor_name;
  std::optional<double> raw_m;
  std::optional<double> filtered_m;

  int mux_channel{-1};

  bool valid{false};
  std::string error;

  [[nodiscard]] RangeSample to_sample() const
  {
    if (!valid || !filtered_m.has_value()) {
      return make_invalid_range_sample(
        sensor_name,
        error.empty() ? "invalid_distance" : error,
        "vl53l1x");
    }

    return make_valid_range_sample(sensor_name, *filtered_m, "vl53l1x");
  }
};

class SAVO_PERCEPTION_PUBLIC Vl53l1xDriver
{
public:
  Vl53l1xDriver(
    Vl53l1xConfig config,
    std::shared_ptr<Tca9548a> mux);

  ~Vl53l1xDriver();

  Vl53l1xDriver(const Vl53l1xDriver &) = delete;
  Vl53l1xDriver & operator=(const Vl53l1xDriver &) = delete;

  Vl53l1xDriver(Vl53l1xDriver && other) noexcept;
  Vl53l1xDriver & operator=(Vl53l1xDriver && other) noexcept;

  [[nodiscard]] const Vl53l1xConfig & config() const;
  [[nodiscard]] bool started() const;
  [[nodiscard]] std::string last_error() const;

  bool start();
  void stop();

  [[nodiscard]] Vl53l1xReading read_once();
  [[nodiscard]] RangeSample read_sample();

private:
  bool select_mux_channel();
  bool initialize_sensor();
  bool start_ranging();
  bool stop_ranging();

  [[nodiscard]] std::optional<int> read_distance_mm();
  [[nodiscard]] std::optional<double> read_distance_m();
  [[nodiscard]] std::optional<double> median_distance_m() const;

  bool append_valid_reading(double distance_m);
  void set_error(std::string error);
  void move_from(Vl53l1xDriver && other) noexcept;

  Vl53l1xConfig config_{};
  std::shared_ptr<Tca9548a> mux_;

  bool started_{false};
  std::deque<double> history_;
  std::string last_error_;
};

struct SAVO_PERCEPTION_PUBLIC Vl53MuxPairConfig
{
  int bus{constants::kI2cBusDefault};

  std::uint8_t tca_address{constants::kTca9548aAddrDefault};
  std::uint8_t sensor_address{constants::kVl53l1xAddrDefault};

  int right_channel{constants::kVl53RightChannelDefault};
  int left_channel{constants::kVl53LeftChannelDefault};

  double settle_s{constants::kVl53SettleSDefault};
  double init_settle_s{constants::kVl53InitSettleSDefault};

  double valid_min_m{constants::kVl53ValidMinMDefault};
  double valid_max_m{constants::kVl53ValidMaxMDefault};

  int median_window{constants::kVl53MedianWindowDefault};
};

struct SAVO_PERCEPTION_PUBLIC Vl53MuxPairReading
{
  Vl53l1xReading left;
  Vl53l1xReading right;
};

struct SAVO_PERCEPTION_PUBLIC Vl53MuxPairSamples
{
  RangeSample left;
  RangeSample right;
};

class SAVO_PERCEPTION_PUBLIC Vl53MuxPairDriver
{
public:
  explicit Vl53MuxPairDriver(Vl53MuxPairConfig config = Vl53MuxPairConfig{});
  ~Vl53MuxPairDriver();

  Vl53MuxPairDriver(const Vl53MuxPairDriver &) = delete;
  Vl53MuxPairDriver & operator=(const Vl53MuxPairDriver &) = delete;

  Vl53MuxPairDriver(Vl53MuxPairDriver && other) noexcept;
  Vl53MuxPairDriver & operator=(Vl53MuxPairDriver && other) noexcept;

  [[nodiscard]] const Vl53MuxPairConfig & config() const;
  [[nodiscard]] bool started() const;
  [[nodiscard]] std::string last_error() const;

  bool start();
  void stop();

  [[nodiscard]] Vl53MuxPairReading read_once();
  [[nodiscard]] Vl53MuxPairSamples read_samples();

private:
  void set_error(std::string error);
  void move_from(Vl53MuxPairDriver && other) noexcept;

  Vl53MuxPairConfig config_{};

  std::shared_ptr<Tca9548a> mux_;
  std::unique_ptr<Vl53l1xDriver> left_;
  std::unique_ptr<Vl53l1xDriver> right_;

  bool started_{false};
  std::string last_error_;
};

SAVO_PERCEPTION_PUBLIC bool valid_vl53_median_window(int median_window);
SAVO_PERCEPTION_PUBLIC bool valid_vl53_distance_mm(int distance_mm);

SAVO_PERCEPTION_PUBLIC std::optional<double> vl53_mm_to_m(
  int distance_mm,
  double valid_min_m,
  double valid_max_m);

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__VL53L1X_DRIVER_HPP_