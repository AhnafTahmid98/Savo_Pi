#include "savo_localization/wheel_odom_node.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "savo_localization/gpio_encoder_backend.hpp"

namespace savo_localization
{

namespace
{

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

std::string escape_json(const std::string & text)
{
  std::ostringstream oss;

  for (const char c : text) {
    switch (c) {
      case '"':
        oss << "\\\"";
        break;
      case '\\':
        oss << "\\\\";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        oss << c;
        break;
    }
  }

  return oss.str();
}

}  // namespace

WheelOdomNode::WheelOdomNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("wheel_odom_node", options)
{
  declare_parameters();
  load_parameters();
  configure_encoder_reader();
  configure_odometry();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    wheel_odom_topic_,
    rclcpp::QoS(10).reliable());

  state_pub_ = create_publisher<std_msgs::msg::String>(
    wheel_odom_state_topic_,
    rclcpp::QoS(10).reliable());

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  const auto poll_period = std::chrono::duration<double>(poll_s_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(poll_period),
    std::bind(&WheelOdomNode::timer_callback, this));

  RCLCPP_INFO(
    get_logger(),
    "wheel_odom_node started | odom_topic=%s | state_topic=%s | rate=%.2f Hz | poll=%.4fs",
    wheel_odom_topic_.c_str(),
    wheel_odom_state_topic_.c_str(),
    publish_rate_hz_,
    poll_s_);
}

WheelOdomNode::~WheelOdomNode()
{
  if (encoder_reader_) {
    encoder_reader_->close();
  }
}

void WheelOdomNode::declare_parameters()
{
  declare_parameter<std::string>("odom_frame_id", odom_frame_id_);
  declare_parameter<std::string>("base_frame_id", base_frame_id_);

  declare_parameter<std::string>("wheel_odom_topic", wheel_odom_topic_);
  declare_parameter<std::string>("wheel_odom_state_topic", wheel_odom_state_topic_);

  declare_parameter<double>("publish_rate_hz", publish_rate_hz_);
  declare_parameter<double>("timeout_s", timeout_s_);
  declare_parameter<bool>("publish_tf", publish_tf_);

  declare_parameter<double>("wheel_diameter_m", wheel_diameter_m_);
  declare_parameter<double>("wheelbase_m", wheelbase_m_);
  declare_parameter<double>("track_m", track_m_);

  declare_parameter<int>("cpr", cpr_);
  declare_parameter<int>("decoding", decoding_);
  declare_parameter<double>("gear_ratio", gear_ratio_);

  declare_parameter<int>("gpiochip", gpiochip_);
  declare_parameter<double>("poll_s", poll_s_);
  declare_parameter<double>("debounce_s", debounce_s_);
  declare_parameter<bool>("use_internal_pullup", use_internal_pullup_);
  declare_parameter<bool>("use_hw_debounce", use_hw_debounce_);

  declare_parameter<int>("fl_a_gpio", fl_a_gpio_);
  declare_parameter<int>("fl_b_gpio", fl_b_gpio_);
  declare_parameter<int>("fr_a_gpio", fr_a_gpio_);
  declare_parameter<int>("fr_b_gpio", fr_b_gpio_);
  declare_parameter<int>("rl_a_gpio", rl_a_gpio_);
  declare_parameter<int>("rl_b_gpio", rl_b_gpio_);
  declare_parameter<int>("rr_a_gpio", rr_a_gpio_);
  declare_parameter<int>("rr_b_gpio", rr_b_gpio_);

  declare_parameter<bool>("invert_fl", invert_fl_);
  declare_parameter<bool>("invert_fr", invert_fr_);
  declare_parameter<bool>("invert_rl", invert_rl_);
  declare_parameter<bool>("invert_rr", invert_rr_);

  declare_parameter<bool>("reset_pose_on_start", reset_pose_on_start_);
  declare_parameter<double>("start_x_m", start_x_m_);
  declare_parameter<double>("start_y_m", start_y_m_);
  declare_parameter<double>("start_yaw_rad", start_yaw_rad_);

  declare_parameter<double>("odom_covariance_scale", odom_covariance_scale_);

  declare_parameter<double>("pose_x_covariance", pose_x_covariance_);
  declare_parameter<double>("pose_y_covariance", pose_y_covariance_);
  declare_parameter<double>("pose_z_covariance", pose_z_covariance_);
  declare_parameter<double>("pose_roll_covariance", pose_roll_covariance_);
  declare_parameter<double>("pose_pitch_covariance", pose_pitch_covariance_);
  declare_parameter<double>("pose_yaw_covariance", pose_yaw_covariance_);

  declare_parameter<double>("twist_vx_covariance", twist_vx_covariance_);
  declare_parameter<double>("twist_vy_covariance", twist_vy_covariance_);
  declare_parameter<double>("twist_vz_covariance", twist_vz_covariance_);
  declare_parameter<double>("twist_wx_covariance", twist_wx_covariance_);
  declare_parameter<double>("twist_wy_covariance", twist_wy_covariance_);
  declare_parameter<double>("twist_wz_covariance", twist_wz_covariance_);
}

void WheelOdomNode::load_parameters()
{
  odom_frame_id_ = get_parameter("odom_frame_id").as_string();
  base_frame_id_ = get_parameter("base_frame_id").as_string();

  wheel_odom_topic_ = get_parameter("wheel_odom_topic").as_string();
  wheel_odom_state_topic_ = get_parameter("wheel_odom_state_topic").as_string();

  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  timeout_s_ = get_parameter("timeout_s").as_double();
  publish_tf_ = get_parameter("publish_tf").as_bool();

  wheel_diameter_m_ = get_parameter("wheel_diameter_m").as_double();
  wheelbase_m_ = get_parameter("wheelbase_m").as_double();
  track_m_ = get_parameter("track_m").as_double();

  cpr_ = static_cast<int>(get_parameter("cpr").as_int());
  decoding_ = static_cast<int>(get_parameter("decoding").as_int());
  gear_ratio_ = get_parameter("gear_ratio").as_double();

  gpiochip_ = static_cast<int>(get_parameter("gpiochip").as_int());
  poll_s_ = get_parameter("poll_s").as_double();
  debounce_s_ = get_parameter("debounce_s").as_double();
  use_internal_pullup_ = get_parameter("use_internal_pullup").as_bool();
  use_hw_debounce_ = get_parameter("use_hw_debounce").as_bool();

  fl_a_gpio_ = static_cast<int>(get_parameter("fl_a_gpio").as_int());
  fl_b_gpio_ = static_cast<int>(get_parameter("fl_b_gpio").as_int());
  fr_a_gpio_ = static_cast<int>(get_parameter("fr_a_gpio").as_int());
  fr_b_gpio_ = static_cast<int>(get_parameter("fr_b_gpio").as_int());
  rl_a_gpio_ = static_cast<int>(get_parameter("rl_a_gpio").as_int());
  rl_b_gpio_ = static_cast<int>(get_parameter("rl_b_gpio").as_int());
  rr_a_gpio_ = static_cast<int>(get_parameter("rr_a_gpio").as_int());
  rr_b_gpio_ = static_cast<int>(get_parameter("rr_b_gpio").as_int());

  invert_fl_ = get_parameter("invert_fl").as_bool();
  invert_fr_ = get_parameter("invert_fr").as_bool();
  invert_rl_ = get_parameter("invert_rl").as_bool();
  invert_rr_ = get_parameter("invert_rr").as_bool();

  reset_pose_on_start_ = get_parameter("reset_pose_on_start").as_bool();
  start_x_m_ = get_parameter("start_x_m").as_double();
  start_y_m_ = get_parameter("start_y_m").as_double();
  start_yaw_rad_ = get_parameter("start_yaw_rad").as_double();

  odom_covariance_scale_ = get_parameter("odom_covariance_scale").as_double();

  pose_x_covariance_ = get_parameter("pose_x_covariance").as_double();
  pose_y_covariance_ = get_parameter("pose_y_covariance").as_double();
  pose_z_covariance_ = get_parameter("pose_z_covariance").as_double();
  pose_roll_covariance_ = get_parameter("pose_roll_covariance").as_double();
  pose_pitch_covariance_ = get_parameter("pose_pitch_covariance").as_double();
  pose_yaw_covariance_ = get_parameter("pose_yaw_covariance").as_double();

  twist_vx_covariance_ = get_parameter("twist_vx_covariance").as_double();
  twist_vy_covariance_ = get_parameter("twist_vy_covariance").as_double();
  twist_vz_covariance_ = get_parameter("twist_vz_covariance").as_double();
  twist_wx_covariance_ = get_parameter("twist_wx_covariance").as_double();
  twist_wy_covariance_ = get_parameter("twist_wy_covariance").as_double();
  twist_wz_covariance_ = get_parameter("twist_wz_covariance").as_double();

  if (odom_frame_id_.empty() || base_frame_id_.empty()) {
    throw std::runtime_error("odom_frame_id and base_frame_id cannot be empty");
  }

  if (odom_frame_id_ == base_frame_id_) {
    throw std::runtime_error("odom_frame_id and base_frame_id cannot be the same");
  }

  if (wheel_odom_topic_.empty() || wheel_odom_state_topic_.empty()) {
    throw std::runtime_error("wheel odom topics cannot be empty");
  }

  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be > 0.0");
  }

  if (timeout_s_ <= 0.0) {
    throw std::runtime_error("timeout_s must be > 0.0");
  }

  if (poll_s_ <= 0.0) {
    throw std::runtime_error("poll_s must be > 0.0");
  }

  if (debounce_s_ < 0.0) {
    throw std::runtime_error("debounce_s must be >= 0.0");
  }

  if (wheel_diameter_m_ <= 0.0 || wheelbase_m_ <= 0.0 || track_m_ <= 0.0) {
    throw std::runtime_error("wheel geometry values must be > 0.0");
  }

  if (cpr_ <= 0 || decoding_ <= 0 || gear_ratio_ <= 0.0) {
    throw std::runtime_error("encoder CPR, decoding, and gear_ratio must be positive");
  }
}

void WheelOdomNode::configure_encoder_reader()
{
  auto backend = std::make_unique<GpioEncoderBackend>();
  encoder_reader_ = std::make_unique<EncoderReader>(
    make_encoder_config_from_params(),
    std::move(backend));

  if (!encoder_reader_->open()) {
    throw std::runtime_error("failed to open GPIO encoder backend");
  }
}

void WheelOdomNode::configure_odometry()
{
  odom_ = std::make_unique<MecanumOdom>(
    make_mecanum_geometry_from_params());

  if (reset_pose_on_start_) {
    odom_->reset(start_x_m_, start_y_m_, start_yaw_rad_);
  }
}

void WheelOdomNode::timer_callback()
{
  try {
    encoder_reader_->poll();

    const rclcpp::Time now_time = now();

    if (!have_last_update_) {
      last_update_time_ = now_time;
      have_last_update_ = true;
      return;
    }

    const double dt_s = (now_time - last_update_time_).seconds();
    const double publish_period_s = 1.0 / publish_rate_hz_;

    if (dt_s < publish_period_s) {
      return;
    }

    const double stamp_s = now_time.seconds();

    const EncoderSample encoder_sample = encoder_reader_->sample(
      stamp_s,
      dt_s,
      wheel_diameter_m_,
      counts_per_wheel_rev(cpr_, decoding_, gear_ratio_),
      wheelbase_m_,
      track_m_);

    const WheelOdomSample odom_sample = odom_->update_from_encoder_sample(
      encoder_sample);

    publish_odometry(odom_sample, encoder_sample);
    publish_state(odom_sample, encoder_sample);

    if (publish_tf_) {
      publish_transform(odom_sample);
    }

    last_update_time_ = now_time;
    ++loop_count_;
    ++publish_count_;
  } catch (const std::exception & exc) {
    ++error_count_;

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "wheel odometry update failed: %s",
      exc.what());
  }
}

void WheelOdomNode::publish_odometry(
  const WheelOdomSample & odom_sample,
  const EncoderSample & encoder_sample)
{
  (void)encoder_sample;
  odom_pub_->publish(make_odometry_msg(odom_sample));
}

void WheelOdomNode::publish_state(
  const WheelOdomSample & odom_sample,
  const EncoderSample & encoder_sample)
{
  std_msgs::msg::String msg;
  msg.data = make_state_json(odom_sample, encoder_sample);
  state_pub_->publish(msg);
}

void WheelOdomNode::publish_transform(const WheelOdomSample & odom_sample)
{
  if (tf_broadcaster_) {
    tf_broadcaster_->sendTransform(make_transform_msg(odom_sample));
  }
}

nav_msgs::msg::Odometry WheelOdomNode::make_odometry_msg(
  const WheelOdomSample & odom_sample) const
{
  nav_msgs::msg::Odometry msg;

  msg.header.stamp = now();
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_frame_id_;

  msg.pose.pose.position.x = odom_sample.pose.x_m;
  msg.pose.pose.position.y = odom_sample.pose.y_m;
  msg.pose.pose.position.z = 0.0;

  msg.pose.pose.orientation.x = 0.0;
  msg.pose.pose.orientation.y = 0.0;
  msg.pose.pose.orientation.z = yaw_to_quaternion_z(odom_sample.pose.yaw_rad);
  msg.pose.pose.orientation.w = yaw_to_quaternion_w(odom_sample.pose.yaw_rad);

  msg.pose.covariance = pose_covariance();

  msg.twist.twist.linear.x = odom_sample.twist.vx_mps;
  msg.twist.twist.linear.y = odom_sample.twist.vy_mps;
  msg.twist.twist.linear.z = 0.0;

  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = odom_sample.twist.omega_rad_s;

  msg.twist.covariance = twist_covariance();

  return msg;
}

geometry_msgs::msg::TransformStamped WheelOdomNode::make_transform_msg(
  const WheelOdomSample & odom_sample) const
{
  geometry_msgs::msg::TransformStamped msg;

  msg.header.stamp = now();
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_frame_id_;

  msg.transform.translation.x = odom_sample.pose.x_m;
  msg.transform.translation.y = odom_sample.pose.y_m;
  msg.transform.translation.z = 0.0;

  msg.transform.rotation.x = 0.0;
  msg.transform.rotation.y = 0.0;
  msg.transform.rotation.z = yaw_to_quaternion_z(odom_sample.pose.yaw_rad);
  msg.transform.rotation.w = yaw_to_quaternion_w(odom_sample.pose.yaw_rad);

  return msg;
}

std::string WheelOdomNode::make_state_json(
  const WheelOdomSample & odom_sample,
  const EncoderSample & encoder_sample) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6);

  oss << "{";
  oss << "\"node\":\"wheel_odom_node\",";
  oss << "\"status\":\"OK\",";
  oss << "\"odom_topic\":\"" << escape_json(wheel_odom_topic_) << "\",";
  oss << "\"state_topic\":\"" << escape_json(wheel_odom_state_topic_) << "\",";
  oss << "\"odom_frame_id\":\"" << escape_json(odom_frame_id_) << "\",";
  oss << "\"base_frame_id\":\"" << escape_json(base_frame_id_) << "\",";
  oss << "\"publish_tf\":" << bool_text(publish_tf_) << ",";
  oss << "\"stamp_s\":" << odom_sample.stamp_s << ",";
  oss << "\"dt_s\":" << odom_sample.dt_s << ",";

  oss << "\"pose\":{";
  oss << "\"x_m\":" << odom_sample.pose.x_m << ",";
  oss << "\"y_m\":" << odom_sample.pose.y_m << ",";
  oss << "\"yaw_rad\":" << odom_sample.pose.yaw_rad;
  oss << "},";

  oss << "\"twist\":{";
  oss << "\"vx_mps\":" << odom_sample.twist.vx_mps << ",";
  oss << "\"vy_mps\":" << odom_sample.twist.vy_mps << ",";
  oss << "\"omega_rad_s\":" << odom_sample.twist.omega_rad_s;
  oss << "},";

  oss << "\"wheel_speeds\":{";
  oss << "\"FL\":" << odom_sample.wheel_speeds.fl_mps << ",";
  oss << "\"FR\":" << odom_sample.wheel_speeds.fr_mps << ",";
  oss << "\"RL\":" << odom_sample.wheel_speeds.rl_mps << ",";
  oss << "\"RR\":" << odom_sample.wheel_speeds.rr_mps;
  oss << "},";

  oss << "\"encoders\":{";
  oss << "\"active_wheel_count\":" << encoder_sample.active_wheel_count() << ",";
  oss << "\"total_illegal_transitions\":" << encoder_sample.total_illegal_transitions() << ",";

  oss << "\"FL\":{";
  oss << "\"count\":" << encoder_sample.fl.count << ",";
  oss << "\"delta\":" << encoder_sample.fl.delta_count << ",";
  oss << "\"cps\":" << encoder_sample.fl.counts_per_second << ",";
  oss << "\"speed_mps\":" << encoder_sample.fl.speed_mps << ",";
  oss << "\"direction\":" << encoder_sample.fl.direction << ",";
  oss << "\"illegal\":" << encoder_sample.fl.illegal_transitions;
  oss << "},";

  oss << "\"FR\":{";
  oss << "\"count\":" << encoder_sample.fr.count << ",";
  oss << "\"delta\":" << encoder_sample.fr.delta_count << ",";
  oss << "\"cps\":" << encoder_sample.fr.counts_per_second << ",";
  oss << "\"speed_mps\":" << encoder_sample.fr.speed_mps << ",";
  oss << "\"direction\":" << encoder_sample.fr.direction << ",";
  oss << "\"illegal\":" << encoder_sample.fr.illegal_transitions;
  oss << "},";

  oss << "\"RL\":{";
  oss << "\"count\":" << encoder_sample.rl.count << ",";
  oss << "\"delta\":" << encoder_sample.rl.delta_count << ",";
  oss << "\"cps\":" << encoder_sample.rl.counts_per_second << ",";
  oss << "\"speed_mps\":" << encoder_sample.rl.speed_mps << ",";
  oss << "\"direction\":" << encoder_sample.rl.direction << ",";
  oss << "\"illegal\":" << encoder_sample.rl.illegal_transitions;
  oss << "},";

  oss << "\"RR\":{";
  oss << "\"count\":" << encoder_sample.rr.count << ",";
  oss << "\"delta\":" << encoder_sample.rr.delta_count << ",";
  oss << "\"cps\":" << encoder_sample.rr.counts_per_second << ",";
  oss << "\"speed_mps\":" << encoder_sample.rr.speed_mps << ",";
  oss << "\"direction\":" << encoder_sample.rr.direction << ",";
  oss << "\"illegal\":" << encoder_sample.rr.illegal_transitions;
  oss << "}";

  oss << "},";

  oss << "\"sample_count\":" << odom_->sample_count() << ",";
  oss << "\"total_distance_m\":" << odom_->total_distance_m() << ",";
  oss << "\"total_rotation_rad\":" << odom_->total_rotation_rad() << ",";
  oss << "\"loop_count\":" << loop_count_ << ",";
  oss << "\"publish_count\":" << publish_count_ << ",";
  oss << "\"error_count\":" << error_count_;
  oss << "}";

  return oss.str();
}

EncoderHardwareConfig WheelOdomNode::make_encoder_config_from_params() const
{
  EncoderHardwareConfig config{};

  config.fl = WheelEncoderConfig{
    WheelId::FL,
    WHEEL_FL,
    fl_a_gpio_,
    fl_b_gpio_,
    invert_fl_,
  };

  config.fr = WheelEncoderConfig{
    WheelId::FR,
    WHEEL_FR,
    fr_a_gpio_,
    fr_b_gpio_,
    invert_fr_,
  };

  config.rl = WheelEncoderConfig{
    WheelId::RL,
    WHEEL_RL,
    rl_a_gpio_,
    rl_b_gpio_,
    invert_rl_,
  };

  config.rr = WheelEncoderConfig{
    WheelId::RR,
    WHEEL_RR,
    rr_a_gpio_,
    rr_b_gpio_,
    invert_rr_,
  };

  config.gpiochip = gpiochip_;
  config.poll_s = poll_s_;
  config.debounce_s = debounce_s_;
  config.use_internal_pullup = use_internal_pullup_;
  config.use_hw_debounce = use_hw_debounce_;

  if (!config.valid()) {
    throw std::runtime_error("invalid encoder GPIO configuration");
  }

  return config;
}

MecanumGeometry WheelOdomNode::make_mecanum_geometry_from_params() const
{
  MecanumGeometry geometry{};
  geometry.wheelbase_m = wheelbase_m_;
  geometry.track_m = track_m_;
  geometry.wheel_diameter_m = wheel_diameter_m_;
  geometry.counts_per_wheel_rev = counts_per_wheel_rev(
    cpr_,
    decoding_,
    gear_ratio_);

  if (!geometry.valid()) {
    throw std::runtime_error("invalid mecanum odometry geometry");
  }

  return geometry;
}

std::array<double, 36> WheelOdomNode::pose_covariance() const
{
  return covariance_from_diagonal(
    pose_x_covariance_,
    pose_y_covariance_,
    pose_z_covariance_,
    pose_roll_covariance_,
    pose_pitch_covariance_,
    pose_yaw_covariance_,
    odom_covariance_scale_);
}

std::array<double, 36> WheelOdomNode::twist_covariance() const
{
  return covariance_from_diagonal(
    twist_vx_covariance_,
    twist_vy_covariance_,
    twist_vz_covariance_,
    twist_wx_covariance_,
    twist_wy_covariance_,
    twist_wz_covariance_,
    odom_covariance_scale_);
}

std::array<double, 36> WheelOdomNode::covariance_from_diagonal(
  double x,
  double y,
  double z,
  double roll,
  double pitch,
  double yaw,
  double scale)
{
  const double safe_scale = scale >= 0.0 ? scale : 1.0;

  std::array<double, 36> values{};
  values.fill(0.0);

  values[0] = x * safe_scale;
  values[7] = y * safe_scale;
  values[14] = z * safe_scale;
  values[21] = roll * safe_scale;
  values[28] = pitch * safe_scale;
  values[35] = yaw * safe_scale;

  return values;
}

double WheelOdomNode::yaw_to_quaternion_z(double yaw_rad)
{
  return std::sin(yaw_rad * 0.5);
}

double WheelOdomNode::yaw_to_quaternion_w(double yaw_rad)
{
  return std::cos(yaw_rad * 0.5);
}

int WheelOdomNode::counts_per_wheel_rev(
  int cpr,
  int decoding,
  double gear_ratio)
{
  const double value =
    static_cast<double>(cpr) * static_cast<double>(decoding) * gear_ratio;

  return std::max(1, static_cast<int>(std::round(value)));
}

}  // namespace savo_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<savo_localization::WheelOdomNode>());
  } catch (const std::exception & exc) {
    RCLCPP_FATAL(
      rclcpp::get_logger("wheel_odom_node"),
      "Fatal error: %s",
      exc.what());
  }

  rclcpp::shutdown();
  return 0;
}