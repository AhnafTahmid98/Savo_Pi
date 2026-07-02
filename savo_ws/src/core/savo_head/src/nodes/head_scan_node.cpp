#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "savo_head/core/diagnostics.hpp"
#include "savo_head/core/scan_pattern.hpp"

namespace savo_head
{

namespace
{

constexpr double kCmdAbsolute = 0.0;
constexpr double kCmdCenter = 2.0;
constexpr double kCmdHold = 3.0;
constexpr double kCmdStop = 4.0;

constexpr const char * kScanCmdStart = "start";
constexpr const char * kScanCmdStop = "stop";
constexpr const char * kScanCmdPause = "pause";
constexpr const char * kScanCmdResume = "resume";
constexpr const char * kScanCmdToggle = "toggle";
constexpr const char * kScanCmdCenter = "center";

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, int value)
{
  return kv(key, std::to_string(value));
}

diagnostic_msgs::msg::KeyValue kv(const std::string & key, bool value)
{
  return kv(key, value ? "true" : "false");
}

std::uint8_t ros_level(DiagnosticLevel level)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  switch (level) {
    case DiagnosticLevel::kOk:
      return DiagnosticStatus::OK;
    case DiagnosticLevel::kWarn:
      return DiagnosticStatus::WARN;
    case DiagnosticLevel::kError:
      return DiagnosticStatus::ERROR;
  }

  return DiagnosticStatus::ERROR;
}

std::vector<int> to_int_vector(const std::vector<int64_t> & values)
{
  std::vector<int> out;
  out.reserve(values.size());

  for (const auto value : values) {
    out.push_back(static_cast<int>(value));
  }

  return out;
}

std::chrono::nanoseconds seconds_to_period(double seconds)
{
  const auto safe_seconds = std::max(0.001, seconds);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(safe_seconds));
}

}  // namespace

class HeadScanNode : public rclcpp::Node
{
public:
  HeadScanNode()
  : Node("head_scan_node")
  {
    declare_parameters();

    runtime_ = make_scan_runtime(scan_profile_from_parameters());
    last_error_.reset();

    pan_tilt_pub_ = create_publisher<geometry_msgs::msg::Vector3>(
      get_parameter("pan_tilt_cmd_topic").as_string(),
      rclcpp::QoS(10).reliable());

    scan_state_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("scan_state_topic").as_string(),
      rclcpp::QoS(10).reliable());

    status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      get_parameter("status_topic").as_string(),
      rclcpp::QoS(10).reliable());

    dashboard_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("dashboard_text_topic").as_string(),
      rclcpp::QoS(10).reliable());

    scan_cmd_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("scan_cmd_topic").as_string(),
      rclcpp::QoS(10).reliable(),
      std::bind(&HeadScanNode::on_scan_cmd, this, std::placeholders::_1));

    start_scan_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("start_scan_service").as_string(),
      std::bind(
        &HeadScanNode::on_start_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    stop_scan_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("stop_scan_service").as_string(),
      std::bind(
        &HeadScanNode::on_stop_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    pause_scan_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("pause_scan_service").as_string(),
      std::bind(
        &HeadScanNode::on_pause_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    resume_scan_service_ = create_service<std_srvs::srv::Trigger>(
      get_parameter("resume_scan_service").as_string(),
      std::bind(
        &HeadScanNode::on_resume_service,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto step_delay_s = std::max(
      0.01,
      get_parameter("semantic_scan_step_delay_s").as_double());
    const auto status_hz = std::max(0.2, get_parameter("status_hz").as_double());

    scan_timer_ = create_wall_timer(
      seconds_to_period(step_delay_s),
      std::bind(&HeadScanNode::scan_tick, this));

    status_timer_ = create_wall_timer(
      seconds_to_period(1.0 / status_hz),
      std::bind(&HeadScanNode::publish_status, this));

    if (get_parameter("auto_start").as_bool()) {
      start_scan_runtime("auto_start");
    }

    RCLCPP_INFO(get_logger(), "head scan node started");
  }

  ~HeadScanNode() override
  {
    try {
      publish_stop_command();
    } catch (...) {
    }
  }

private:
  void declare_parameters()
  {
    declare_parameter<std::string>("pan_tilt_cmd_topic", kTopicPanTiltCmd);
    declare_parameter<std::string>("scan_cmd_topic", kTopicScanCmd);
    declare_parameter<std::string>("scan_state_topic", kTopicScanState);
    declare_parameter<std::string>("status_topic", kTopicStatus);
    declare_parameter<std::string>("dashboard_text_topic", kTopicDashboardText);

    declare_parameter<std::string>("start_scan_service", "/savo_head/start_scan");
    declare_parameter<std::string>("stop_scan_service", "/savo_head/stop_scan");
    declare_parameter<std::string>("pause_scan_service", "/savo_head/pause_scan");
    declare_parameter<std::string>("resume_scan_service", "/savo_head/resume_scan");

    declare_parameter<bool>("auto_start", false);
    declare_parameter<double>("status_hz", kStatusHz);

    declare_parameter<std::string>("default_scan_profile", "semantic_scan");
    declare_parameter<bool>("semantic_scan_enabled", true);
    declare_parameter<std::string>("semantic_scan_mode", kScanModeStaged);

    declare_parameter<int>("semantic_scan_pan_min_deg", kPanMinDeg);
    declare_parameter<int>("semantic_scan_pan_center_deg", kPanCenterDeg);
    declare_parameter<int>("semantic_scan_pan_max_deg", kPanMaxDeg);

    declare_parameter<int>("semantic_scan_tilt_min_deg", kTiltMinDeg);
    declare_parameter<int>("semantic_scan_tilt_max_deg", kTiltMaxDeg);

    declare_parameter<int>("semantic_scan_pan_step_deg", kScanPanStepDeg);
    declare_parameter<int>("semantic_scan_tilt_step_deg", kScanTiltStepDeg);
    declare_parameter<double>("semantic_scan_step_delay_s", kScanStepDelayS);

    declare_parameter<int>("semantic_scan_start_pan_deg", kPanMinDeg);
    declare_parameter<int>("semantic_scan_start_tilt_deg", kTiltMinDeg);

    declare_parameter<std::vector<int64_t>>(
      "semantic_scan_pan_targets_deg",
      std::vector<int64_t>{kPanCenterDeg, kPanMaxDeg, kPanCenterDeg, kPanMinDeg});

    declare_parameter<std::vector<int64_t>>(
      "semantic_scan_tilt_sweep_pan_targets_deg",
      std::vector<int64_t>{kPanCenterDeg});

    declare_parameter<double>("semantic_scan_hold_at_pan_target_s", 0.10);
    declare_parameter<double>("semantic_scan_hold_after_tilt_sweep_s", 0.15);
    declare_parameter<bool>("semantic_scan_pause_on_manual_command", true);
    declare_parameter<double>("semantic_scan_resume_after_manual_s", 0.0);
    declare_parameter<bool>("semantic_scan_center_on_stop", true);
  }

  ScanProfile scan_profile_from_parameters() const
  {
    ScanProfile profile;

    profile.name = get_parameter("default_scan_profile").as_string();
    profile.enabled = get_parameter("semantic_scan_enabled").as_bool();
    profile.mode = get_parameter("semantic_scan_mode").as_string();

    profile.pan_min_deg = get_parameter("semantic_scan_pan_min_deg").as_int();
    profile.pan_center_deg = get_parameter("semantic_scan_pan_center_deg").as_int();
    profile.pan_max_deg = get_parameter("semantic_scan_pan_max_deg").as_int();

    profile.tilt_min_deg = get_parameter("semantic_scan_tilt_min_deg").as_int();
    profile.tilt_max_deg = get_parameter("semantic_scan_tilt_max_deg").as_int();

    profile.pan_step_deg = get_parameter("semantic_scan_pan_step_deg").as_int();
    profile.tilt_step_deg = get_parameter("semantic_scan_tilt_step_deg").as_int();
    profile.step_delay_s = get_parameter("semantic_scan_step_delay_s").as_double();

    profile.start_pan_deg = get_parameter("semantic_scan_start_pan_deg").as_int();
    profile.start_tilt_deg = get_parameter("semantic_scan_start_tilt_deg").as_int();

    profile.pan_targets_deg = to_int_vector(
      get_parameter("semantic_scan_pan_targets_deg").as_integer_array());

    profile.tilt_sweep_pan_targets_deg = to_int_vector(
      get_parameter("semantic_scan_tilt_sweep_pan_targets_deg").as_integer_array());

    profile.hold_at_pan_target_s =
      get_parameter("semantic_scan_hold_at_pan_target_s").as_double();
    profile.hold_after_tilt_sweep_s =
      get_parameter("semantic_scan_hold_after_tilt_sweep_s").as_double();

    profile.pause_on_manual_command =
      get_parameter("semantic_scan_pause_on_manual_command").as_bool();
    profile.resume_after_manual_s =
      get_parameter("semantic_scan_resume_after_manual_s").as_double();
    profile.center_on_stop =
      get_parameter("semantic_scan_center_on_stop").as_bool();

    profile = profile.normalized();

    const auto errors = profile.validation_errors();
    if (!errors.empty()) {
      for (const auto & error : errors) {
        RCLCPP_ERROR(get_logger(), "scan profile error: %s", error.c_str());
      }
      throw std::runtime_error("invalid head scan profile");
    }

    return profile;
  }

  void on_scan_cmd(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto command = trim_lower(msg->data);

    if (command == kScanCmdStart) {
      start_scan_runtime("topic");
    } else if (command == kScanCmdStop) {
      stop_scan_runtime("topic");
    } else if (command == kScanCmdPause) {
      pause_scan_runtime("topic");
    } else if (command == kScanCmdResume) {
      resume_scan_runtime("topic");
    } else if (command == kScanCmdToggle) {
      toggle_scan_runtime("topic");
    } else if (command == kScanCmdCenter) {
      publish_center_command("topic");
    } else {
      last_error_ = "unknown scan command: " + command;
      RCLCPP_WARN(get_logger(), "%s", last_error_->c_str());
    }

    publish_scan_state();
  }

  void on_start_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    start_scan_runtime("service");

    response->success = runtime_.status.state == kScanStateRunning;
    response->message = state_text();
  }

  void on_stop_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    stop_scan_runtime("service");

    response->success =
      runtime_.status.state == kScanStateDone ||
      runtime_.status.state == kScanStateIdle;
    response->message = state_text();
  }

  void on_pause_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    pause_scan_runtime("service");

    response->success = runtime_.status.state == kScanStatePaused;
    response->message = state_text();
  }

  void on_resume_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    resume_scan_runtime("service");

    response->success = runtime_.status.state == kScanStateRunning;
    response->message = state_text();
  }

  void start_scan_runtime(const std::string & source)
  {
    (void)source;
    runtime_ = make_scan_runtime(scan_profile_from_parameters()).start(now_s());
    last_error_.reset();
    publish_scan_state();
  }

  void stop_scan_runtime(const std::string & source)
  {
    (void)source;

    if (get_parameter("semantic_scan_center_on_stop").as_bool()) {
      publish_center_command("stop");
    } else {
      publish_stop_command();
    }

    runtime_ = runtime_.stop(now_s());
    publish_scan_state();
  }

  void pause_scan_runtime(const std::string & source)
  {
    (void)source;
    runtime_ = runtime_.pause(now_s());
    publish_hold_command("pause");
    publish_scan_state();
  }

  void resume_scan_runtime(const std::string & source)
  {
    (void)source;
    runtime_ = runtime_.resume(now_s());
    publish_scan_state();
  }

  void toggle_scan_runtime(const std::string & source)
  {
    if (runtime_.status.state == kScanStateRunning) {
      pause_scan_runtime(source);
    } else if (runtime_.status.state == kScanStatePaused) {
      resume_scan_runtime(source);
    } else {
      start_scan_runtime(source);
    }
  }

  void scan_tick()
  {
    if (runtime_.status.state != kScanStateRunning) {
      return;
    }

    try {
      auto [next_runtime, result] = runtime_.step(now_s());
      runtime_ = next_runtime;

      publish_command(result.command);
      last_error_.reset();
      publish_scan_state();
    } catch (const std::exception & exc) {
      last_error_ = exc.what();
      runtime_ = runtime_.stop(now_s(), exc.what());
      RCLCPP_ERROR(get_logger(), "scan step failed: %s", exc.what());
      publish_scan_state();
    }
  }

  void publish_command(const PanTiltCommand & command)
  {
    geometry_msgs::msg::Vector3 msg;
    msg.x = static_cast<double>(command.pan_deg.value_or(runtime_.status.pan_deg));
    msg.y = static_cast<double>(command.tilt_deg.value_or(runtime_.status.tilt_deg));
    msg.z = kCmdAbsolute;

    if (command.type == CommandType::kCenter) {
      msg.x = 0.0;
      msg.y = 0.0;
      msg.z = kCmdCenter;
    } else if (command.type == CommandType::kHold) {
      msg.x = 0.0;
      msg.y = 0.0;
      msg.z = kCmdHold;
    } else if (command.type == CommandType::kStop) {
      msg.x = 0.0;
      msg.y = 0.0;
      msg.z = kCmdStop;
    }

    pan_tilt_pub_->publish(msg);
  }

  void publish_center_command(const std::string & source)
  {
    (void)source;

    geometry_msgs::msg::Vector3 msg;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = kCmdCenter;
    pan_tilt_pub_->publish(msg);
  }

  void publish_hold_command(const std::string & source)
  {
    (void)source;

    geometry_msgs::msg::Vector3 msg;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = kCmdHold;
    pan_tilt_pub_->publish(msg);
  }

  void publish_stop_command()
  {
    if (!pan_tilt_pub_) {
      return;
    }

    geometry_msgs::msg::Vector3 msg;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = kCmdStop;
    pan_tilt_pub_->publish(msg);
  }

  void publish_scan_state()
  {
    if (!scan_state_pub_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = state_text();
    scan_state_pub_->publish(msg);
  }

  void publish_status()
  {
    const auto health = current_health();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "savo_head.head_scan";
    status.hardware_id = "savo_head";
    status.level = ros_level(health.level);
    status.message = health.message;

    status.values = {
      kv("state", runtime_.status.state),
      kv("phase", runtime_.status.phase),
      kv("pan_deg", runtime_.status.pan_deg),
      kv("tilt_deg", runtime_.status.tilt_deg),
      kv("target_deg", runtime_.status.current_pan_target_deg),
      kv("cycle_count", runtime_.status.cycle_count),
      kv("step_count", runtime_.status.step_count),
      kv("enabled", runtime_.profile.enabled),
      kv("last_error", last_error_.value_or(""))
    };

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status = {status};

    status_pub_->publish(array);

    std_msgs::msg::String dashboard;
    dashboard.data = dashboard_text(health) + " " + state_text();
    dashboard_pub_->publish(dashboard);
  }

  HeadHealthSummary current_health() const
  {
    const auto scan = scan_health(
      runtime_.status.state,
      runtime_.status.phase,
      runtime_.status.pan_deg,
      runtime_.status.tilt_deg,
      last_error_);

    return summarize_health({scan}, true);
  }

  std::string state_text() const
  {
    std::ostringstream stream;
    stream << "state=" << runtime_.status.state
           << ";phase=" << runtime_.status.phase
           << ";pan=" << runtime_.status.pan_deg
           << ";tilt=" << runtime_.status.tilt_deg
           << ";target=" << runtime_.status.current_pan_target_deg
           << ";cycle=" << runtime_.status.cycle_count
           << ";step=" << runtime_.status.step_count
           << ";error=" << last_error_.value_or("");

    return stream.str();
  }

  double now_s() const
  {
    return now().seconds();
  }

  static std::string trim_lower(std::string value)
  {
    value.erase(
      value.begin(),
      std::find_if(
        value.begin(),
        value.end(),
        [](unsigned char c) {return !std::isspace(c);}));

    value.erase(
      std::find_if(
        value.rbegin(),
        value.rend(),
        [](unsigned char c) {return !std::isspace(c);}).base(),
      value.end());

    std::transform(
      value.begin(),
      value.end(),
      value.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});

    return value;
  }

  ScanRuntime runtime_{};
  std::optional<std::string> last_error_{};

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pan_tilt_pub_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scan_state_pub_{};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_{};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dashboard_pub_{};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scan_cmd_sub_{};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_scan_service_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_scan_service_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_scan_service_{};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_scan_service_{};

  rclcpp::TimerBase::SharedPtr scan_timer_{};
  rclcpp::TimerBase::SharedPtr status_timer_{};
};

}  // namespace savo_head

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<savo_head::HeadScanNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    std::cerr << "head_scan_node failed: " << exc.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
