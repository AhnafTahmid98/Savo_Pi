#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "savo_speech/speech_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    const auto node =
      std::make_shared<savo_speech::SpeechNode>();

    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("savo_speech_main"),
      "savo_speech terminated: %s",
      exception.what());

    exit_code = 1;
  }

  rclcpp::shutdown();

  return exit_code;
}
