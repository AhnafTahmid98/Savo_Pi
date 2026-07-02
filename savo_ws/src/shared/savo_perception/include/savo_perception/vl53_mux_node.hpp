#ifndef SAVO_PERCEPTION__VL53_MUX_NODE_HPP_
#define SAVO_PERCEPTION__VL53_MUX_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "savo_perception/constants.hpp"
#include "savo_perception/topic_names.hpp"
#include "savo_perception/vl53l1x_driver.hpp"
#include "savo_perception/visibility_control.hpp"

namespace savo_perception
{

struct SAVO_PERCEPTION_PUBLIC Vl53MuxNodeConfig
{
  int bus{constants::kI2cBusDefault};

  std::uint8_t tca_addr{constants::kTca9548aAddrDefault};
  std::uint8_t vl53_addr{constants::kVl53l1xAddrDefault};

  int right_channel{constants::kVl53RightChannelDefault};
  int left_channel{constants::kVl53LeftChannelDefault};

  double rate_hz{constants::kVl53RateHzDefault};
  int median_window{constants::kVl53MedianWindowDefault};

  double settle_s{constants::kVl53SettleSDefault};
  double init_settle_s{constants::kVl53InitSettleSDefault};

  double valid_min_m{constants::kVl53ValidMinMDefault};
  double valid_max_m{constants::kVl53ValidMaxMDefault};

  std::string left_topic{topics::kTofLeftM};
  std::string right_topic{topics::kTofRightM};

  bool publish_nan_on_error{constants::kPublishNanOnErrorDefault};
  bool startup_fail_is_fatal{constants::kStartupFailIsFatalDefault};

  double stale_timeout_s{0.50};
};

struct SAVO_PERCEPTION_PUBLIC Vl53LatestState
{
  Vl53l1xReading left;
  Vl53l1xReading right;

  std::chrono::steady_clock::time_point stamp{std::chrono::steady_clock::now()};
  std::string driver_error;
  bool has_update{false};
};

class SAVO_PERCEPTION_PUBLIC Vl53MuxNode : public rclcpp::Node
{
public:
  explicit Vl53MuxNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Vl53MuxNode() override;

private:
  void declare_parameters();
  void load_parameters();
  void setup_interfaces();

  void start_driver();
  void stop_driver();

  void start_worker();
  void request_worker_stop();
  void worker_loop();

  void on_timer();

  void publish_latest();
  void publish_reading(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher,
    const Vl53l1xReading & reading);

  void publish_distance(
    const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & publisher,
    const std::optional<double> & distance_m);

  [[nodiscard]] bool latest_is_stale(const Vl53LatestState & latest) const;
  [[nodiscard]] Vl53MuxPairConfig make_driver_config() const;

  Vl53MuxNodeConfig config_{};

  std::shared_ptr<Vl53MuxPairDriver> driver_;
  std::string driver_error_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic_bool worker_stop_requested_{false};
  std::atomic_bool worker_running_{false};

  mutable std::mutex latest_mutex_;
  Vl53LatestState latest_;
};

}  // namespace savo_perception

#endif  // SAVO_PERCEPTION__VL53_MUX_NODE_HPP_