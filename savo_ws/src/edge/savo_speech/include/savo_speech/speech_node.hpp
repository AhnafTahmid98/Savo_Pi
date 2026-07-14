#ifndef SAVO_SPEECH__SPEECH_NODE_HPP_
#define SAVO_SPEECH__SPEECH_NODE_HPP_

#include <cstdint>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "savo_speech/session/speech_error.hpp"
#include "savo_speech/session/speech_phase.hpp"

namespace savo_speech
{

class SpeechNode final : public rclcpp::Node
{
public:
  explicit SpeechNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct RuntimeConfig
  {
    bool enabled{true};

    std::string profile{"edge_real_robot_v1"};
    std::string robot_id{"robot-savo"};
    std::string host_role{"edge"};
    std::string device_id{"savo-edge"};

    bool audio_required{true};
    std::string capture_device{"savo_respeaker"};
    std::string playback_device{"savo_respeaker"};
    bool allow_numeric_device_fallback{false};

    double status_publish_rate_hz{2.0};
    double heartbeat_rate_hz{1.0};
  };

  void declare_parameters();
  void load_parameters();
  void validate_parameters() const;

  void configure_initial_state();

  void publish_runtime_status();
  void publish_heartbeat();

  [[nodiscard]] std::string readiness_text() const;
  [[nodiscard]] std::string dashboard_text() const;

  [[nodiscard]]
  diagnostic_msgs::msg::DiagnosticArray create_diagnostics() const;

  RuntimeConfig config_{};

  session::SpeechPhase phase_{
    session::SpeechPhase::Starting};

  session::SpeechError error_{
    session::SpeechError::None};

  bool ready_{false};
  bool audio_initialized_{false};
  bool savomind_initialized_{false};

  std::string reason_{"starting"};

  std::uint64_t heartbeat_count_{0U};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    readiness_publisher_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    dashboard_publisher_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr
    heartbeat_publisher_;

  rclcpp::Publisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

}  // namespace savo_speech

#endif  // SAVO_SPEECH__SPEECH_NODE_HPP_
