// wheel_odom_node.cpp
// Robot SAVO — Wheel Odometry Node (C++, ROS 2 Jazzy)
// ------------------------------------------------------------
// Style: explicit includes like your other nodes (rclcpp + msgs)
//
// Reads TWO rear-wheel quadrature encoders (A/B per wheel; 4 GPIO lines total)
// via lgpio on Raspberry Pi 5, publishes nav_msgs/Odometry on /wheel/odom.
//
// IMPORTANT (mecanum + 2 encoders):
// - Estimates vx and wz using a differential-style approximation
// - Sets vy = 0.0 (not observable with rear-only encoders)
// Fuse with IMU + SLAM/AMCL in robot_localization EKF for navigation.
//
// Notes:
// - This node does NOT publish TF. Let EKF be the only TF publisher (odom->base_link).
// - Tight polling in a background thread for reliable edge capture.
// - ROS timer publishes at publish_hz.
//
// System deps:
//   sudo apt install -y liblgpio-dev

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

extern "C" {
#include <lgpio.h>
}

namespace
{

inline geometry_msgs::msg::Quaternion yaw_to_quat(double yaw_rad)
{
  // quaternion from yaw only (roll=pitch=0)
  // q = [x,y,z,w] = [0,0,sin(yaw/2),cos(yaw/2)]
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw_rad);
  q.w = std::cos(0.5 * yaw_rad);
  return q;
}

// Valid Gray-code transitions (A is bit1, B is bit0)
// +1: 00->01->11->10->00
// -1: 00->10->11->01->00
inline int8_t quad_delta(uint8_t prev, uint8_t curr)
{
  if (prev == curr) return 0;

  if ((prev == 0b00 && curr == 0b01) ||
      (prev == 0b01 && curr == 0b11) ||
      (prev == 0b11 && curr == 0b10) ||
      (prev == 0b10 && curr == 0b00))
  {
    return +1;
  }

  if ((prev == 0b00 && curr == 0b10) ||
      (prev == 0b10 && curr == 0b11) ||
      (prev == 0b11 && curr == 0b01) ||
      (prev == 0b01 && curr == 0b00))
  {
    return -1;
  }

  // illegal transition (e.g., 00->11)
  return 127;  // marker
}

struct Pins
{
  int a{-1};
  int b{-1};
};

struct EncState
{
  Pins pins;
  int invert{+1};  // +1 or -1
  std::atomic<int64_t> count{0};
  std::atomic<int64_t> illegal{0};
  uint8_t prev_state{0};
};

int autodetect_gpiochip(const std::vector<int>& pins, int start = 0, int end = 7)
{
  for (int chip = start; chip <= end; ++chip) {
    const int h = lgGpiochipOpen(chip);
    if (h < 0) continue;

    bool ok = true;
    std::vector<int> claimed;
    for (int p : pins) {
      const int rc = lgGpioClaimInput(h, 0, p);
      if (rc < 0) { ok = false; break; }
      claimed.push_back(p);
    }
    for (int p : claimed) (void)lgGpioFree(h, p);
    lgGpiochipClose(h);

    if (ok) return chip;
  }
  return -1;
}

}  // namespace

class WheelOdomNode : public rclcpp::Node
{
public:
  WheelOdomNode()
  : rclcpp::Node("wheel_odom_node")
  {
    // ---------------- Parameters ----------------
    left_.pins.a  = declare_parameter<int>("left_a", 21);
    left_.pins.b  = declare_parameter<int>("left_b", 20);
    right_.pins.a = declare_parameter<int>("right_a", 12);
    right_.pins.b = declare_parameter<int>("right_b", 26);

    invert_left_  = declare_parameter<bool>("invert_left", true);
    invert_right_ = declare_parameter<bool>("invert_right", true);

    wheel_dia_m_  = declare_parameter<double>("wheel_dia_m", 0.065);
    cpr_          = declare_parameter<int>("cpr", 20);
    decoding_     = declare_parameter<int>("decoding", 4);
    gear_         = declare_parameter<double>("gear", 1.0);
    track_m_      = declare_parameter<double>("track_m", 0.165);

    publish_hz_   = declare_parameter<double>("publish_hz", 50.0);
    poll_hz_      = declare_parameter<double>("poll_hz", 2000.0);

    debounce_us_  = declare_parameter<int>("debounce_us", 300); // 0 disables
    chip_index_   = declare_parameter<int>("gpiochip", -1);     // -1 autodetect

    frame_odom_   = declare_parameter<std::string>("frame_odom", "odom");
    frame_base_   = declare_parameter<std::string>("frame_base", "base_link");

    cov_x_   = declare_parameter<double>("cov_x", 0.02);
    cov_y_   = declare_parameter<double>("cov_y", 0.50);  // large: vy unknown
    cov_yaw_ = declare_parameter<double>("cov_yaw", 0.05);

    cov_vx_  = declare_parameter<double>("cov_vx", 0.05);
    cov_vy_  = declare_parameter<double>("cov_vy", 1.00); // large: vy unknown
    cov_wz_  = declare_parameter<double>("cov_wz", 0.05);

    // Sanity checks
    if (decoding_ != 1 && decoding_ != 2 && decoding_ != 4) {
      throw std::runtime_error("decoding must be 1,2,or 4");
    }
    if (wheel_dia_m_ <= 0.0 || track_m_ <= 0.0 || cpr_ <= 0 || gear_ <= 0.0) {
      throw std::runtime_error("Invalid wheel parameters (wheel_dia, track, cpr, gear)");
    }

    // Apply invert so forward becomes positive
    left_.invert  = invert_left_  ? -1 : +1;
    right_.invert = invert_right_ ? -1 : +1;

    // Precompute scaling
    const double edges_per_rev = static_cast<double>(cpr_) * static_cast<double>(decoding_);
    counts_per_wrev_ = std::max(1.0, edges_per_rev * gear_);
    wheel_circ_m_ = M_PI * wheel_dia_m_;

    // ---------------- GPIO init ----------------
    const std::vector<int> pins = {left_.pins.a, left_.pins.b, right_.pins.a, right_.pins.b};

    int chip_used = chip_index_;
    if (chip_used < 0) {
      chip_used = autodetect_gpiochip(pins);
      if (chip_used < 0) {
        throw std::runtime_error("Could not autodetect gpiochip that can claim encoder pins.");
      }
    }

    gpio_h_ = lgGpiochipOpen(chip_used);
    if (gpio_h_ < 0) {
      throw std::runtime_error("Failed to open gpiochip handle.");
    }

    claim_input(left_.pins.a);
    claim_input(left_.pins.b);
    claim_input(right_.pins.a);
    claim_input(right_.pins.b);

    // Hardware debounce (best effort; may be ignored depending on driver)
    if (debounce_us_ > 0) {
      (void)lgGpioSetDebounce(gpio_h_, left_.pins.a, debounce_us_);
      (void)lgGpioSetDebounce(gpio_h_, left_.pins.b, debounce_us_);
      (void)lgGpioSetDebounce(gpio_h_, right_.pins.a, debounce_us_);
      (void)lgGpioSetDebounce(gpio_h_, right_.pins.b, debounce_us_);
    }

    // Initialize prev state
    left_.prev_state  = read_ab(left_.pins);
    right_.prev_state = read_ab(right_.pins);

    // ---------------- ROS pub/timer ----------------
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", rclcpp::QoS(20));

    const double phz = std::max(1.0, publish_hz_);
    const auto pub_period = std::chrono::duration<double>(1.0 / phz);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(pub_period),
      std::bind(&WheelOdomNode::publish_odom, this)
    );

    // Background sampler thread
    running_.store(true);
    sampler_ = std::thread(&WheelOdomNode::sample_loop, this);

    last_stamp_ = now();
    last_L_ = left_.count.load();
    last_R_ = right_.count.load();

    RCLCPP_INFO(get_logger(),
      "[wheel_odom_node] gpiochip=%d  L(A=%d,B=%d) R(A=%d,B=%d) poll_hz=%.0f publish_hz=%.1f",
      chip_used, left_.pins.a, left_.pins.b, right_.pins.a, right_.pins.b,
      poll_hz_, publish_hz_);

    RCLCPP_WARN(get_logger(),
      "[wheel_odom_node] mecanum + rear-only encoders: vy=0.0 (unknown). Fuse with IMU + SLAM/AMCL in EKF.");
  }

  ~WheelOdomNode() override
  {
    running_.store(false);
    if (sampler_.joinable()) sampler_.join();

    if (gpio_h_ >= 0) {
      (void)lgGpioFree(gpio_h_, left_.pins.a);
      (void)lgGpioFree(gpio_h_, left_.pins.b);
      (void)lgGpioFree(gpio_h_, right_.pins.a);
      (void)lgGpioFree(gpio_h_, right_.pins.b);
      lgGpiochipClose(gpio_h_);
      gpio_h_ = -1;
    }
  }

private:
  void claim_input(int bcm_pin)
  {
    const int rc = lgGpioClaimInput(gpio_h_, 0, bcm_pin);
    if (rc < 0) {
      throw std::runtime_error("Failed to claim GPIO input pin " + std::to_string(bcm_pin));
    }
  }

  inline uint8_t read_gpio(int bcm_pin) const
  {
    const int v = lgGpioRead(gpio_h_, bcm_pin);
    return static_cast<uint8_t>((v > 0) ? 1 : 0);
  }

  inline uint8_t read_ab(const Pins& p) const
  {
    const uint8_t a = read_gpio(p.a) & 1U;
    const uint8_t b = read_gpio(p.b) & 1U;
    return static_cast<uint8_t>((a << 1) | b);
  }

  inline void sample_one(EncState& s)
  {
    const uint8_t curr = read_ab(s.pins);
    if (curr == s.prev_state) return;

    const int8_t d = quad_delta(s.prev_state, curr);
    if (d == 127) {
      s.illegal.fetch_add(1, std::memory_order_relaxed);
    } else if (d != 0) {
      s.count.fetch_add(static_cast<int64_t>(d) * static_cast<int64_t>(s.invert),
                        std::memory_order_relaxed);
    }
    s.prev_state = curr;
  }

  void sample_loop()
  {
    const double hz = std::max(200.0, poll_hz_);
    const int64_t ns = static_cast<int64_t>(1e9 / hz);
    const auto sleep_ns = std::chrono::nanoseconds(std::max<int64_t>(1, ns));

    while (running_.load()) {
      sample_one(left_);
      sample_one(right_);
      std::this_thread::sleep_for(sleep_ns);
    }
  }

  void publish_odom()
  {
    const rclcpp::Time t = now();
    const double dt = std::max(1e-6, (t - last_stamp_).seconds());

    const int64_t L = left_.count.load(std::memory_order_relaxed);
    const int64_t R = right_.count.load(std::memory_order_relaxed);

    const int64_t dL = L - last_L_;
    const int64_t dR = R - last_R_;

    last_L_ = L;
    last_R_ = R;
    last_stamp_ = t;

    // Wheel linear speeds (m/s)
    const double L_v = (static_cast<double>(dL) / dt) / counts_per_wrev_ * wheel_circ_m_;
    const double R_v = (static_cast<double>(dR) / dt) / counts_per_wrev_ * wheel_circ_m_;

    // Differential approximation (rear wheels only)
    const double vx = 0.5 * (L_v + R_v);
    const double wz = (R_v - L_v) / std::max(1e-9, track_m_);
    const double vy = 0.0;

    // Integrate planar pose (wheel-only dead-reckoning)
    yaw_ += wz * dt;
    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);
    x_ += vx * cy * dt;
    y_ += vx * sy * dt;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_odom_;
    msg.child_frame_id = frame_base_;

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = yaw_to_quat(yaw_);

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.angular.z = wz;

    // Covariances (conservative)
    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    std::fill(msg.twist.covariance.begin(), msg.twist.covariance.end(), 0.0);

    msg.pose.covariance[0]  = cov_x_;
    msg.pose.covariance[7]  = cov_y_;
    msg.pose.covariance[35] = cov_yaw_;

    msg.twist.covariance[0]  = cov_vx_;
    msg.twist.covariance[7]  = cov_vy_;
    msg.twist.covariance[35] = cov_wz_;

    pub_->publish(msg);
  }

private:
  // ROS
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // GPIO
  int gpio_h_{-1};
  int chip_index_{-1};

  // Encoders
  EncState left_;
  EncState right_;

  // Threading
  std::atomic<bool> running_{false};
  std::thread sampler_;

  // Parameters / constants
  bool invert_left_{true};
  bool invert_right_{true};
  int debounce_us_{300};
  double publish_hz_{50.0};
  double poll_hz_{2000.0};

  double wheel_dia_m_{0.065};
  int cpr_{20};
  int decoding_{4};
  double gear_{1.0};
  double track_m_{0.165};

  std::string frame_odom_{"odom"};
  std::string frame_base_{"base_link"};

  double cov_x_{0.02};
  double cov_y_{0.50};
  double cov_yaw_{0.05};
  double cov_vx_{0.05};
  double cov_vy_{1.00};
  double cov_wz_{0.05};

  double counts_per_wrev_{80.0};
  double wheel_circ_m_{M_PI * 0.065};

  // Integration state
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  int64_t last_L_{0};
  int64_t last_R_{0};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<WheelOdomNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[wheel_odom_node] FATAL: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}