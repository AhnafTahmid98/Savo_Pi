// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

class ObserverDashboardNode final : public rclcpp::Node
{
public:
  ObserverDashboardNode()
  : Node("observer_dashboard_node")
  {
    bind_address_ = declare_parameter<std::string>("bind_address", "127.0.0.1");
    port_ = declare_parameter<int>("port", 8765);
    polling_interval_ms_ = declare_parameter<int>("polling_interval_ms", 1000);
    history_capacity_ = declare_parameter<int>("history_capacity", 120);
    if (port_ < 1024 || port_ > 65535) {
      throw std::invalid_argument("dashboard port must be between 1024 and 65535");
    }
    if (polling_interval_ms_ < 250 || polling_interval_ms_ > 10000) {
      throw std::invalid_argument("polling_interval_ms must be between 250 and 10000");
    }
    if (history_capacity_ < 10 || history_capacity_ > 1000) {
      throw std::invalid_argument("history_capacity must be between 10 and 1000");
    }
    web_root_ = declare_parameter<std::string>(
      "web_root",
      ament_index_cpp::get_package_share_directory("savo_observer") + "/dashboard/web");
    telemetry_subscription_ = create_subscription<std_msgs::msg::String>(
      "/savo_observer/telemetry", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry_ = message->data;
      });
    StartServer();
  }

  ~ObserverDashboardNode() override
  {
    stopping_.store(true);
    if (server_fd_ >= 0) {
      shutdown(server_fd_, SHUT_RDWR);
      close(server_fd_);
      server_fd_ = -1;
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  void StartServer()
  {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      throw std::runtime_error("unable to create dashboard socket");
    }
    const int reuse = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port_));
    if (inet_pton(AF_INET, bind_address_.c_str(), &address.sin_addr) != 1) {
      close(server_fd_);
      server_fd_ = -1;
      throw std::invalid_argument("dashboard bind_address must be an IPv4 address");
    }
    if (bind(server_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0 ||
      listen(server_fd_, 8) < 0)
    {
      const auto reason = std::string(std::strerror(errno));
      close(server_fd_);
      server_fd_ = -1;
      throw std::runtime_error("unable to bind dashboard: " + reason);
    }
    server_thread_ = std::thread([this]() {Serve();});
    RCLCPP_INFO(
      get_logger(), "Read-only observer dashboard listening on http://%s:%d",
      bind_address_.c_str(), port_);
  }

  void Serve()
  {
    while (!stopping_.load()) {
      fd_set descriptors;
      FD_ZERO(&descriptors);
      FD_SET(server_fd_, &descriptors);
      timeval timeout{0, 250000};
      const int ready = select(server_fd_ + 1, &descriptors, nullptr, nullptr, &timeout);
      if (ready <= 0 || stopping_.load()) {
        continue;
      }
      const int client = accept(server_fd_, nullptr, nullptr);
      if (client >= 0) {
        HandleClient(client);
        close(client);
      }
    }
  }

  void HandleClient(const int client)
  {
    char buffer[4096]{};
    const auto received = recv(client, buffer, sizeof(buffer) - 1U, 0);
    if (received <= 0) {
      return;
    }
    std::istringstream request(std::string(buffer, static_cast<std::size_t>(received)));
    std::string method;
    std::string path;
    request >> method >> path;
    if (method != "GET") {
      Send(client, 405, "text/plain", "read-only GET requests only\n");
      return;
    }
    if (path == "/api/telemetry") {
      std::lock_guard<std::mutex> lock(telemetry_mutex_);
      Send(client, 200, "application/json", telemetry_);
      return;
    }
    if (path == "/api/config") {
      std::ostringstream config;
      config << "{\"polling_interval_ms\":" << polling_interval_ms_
             << ",\"history_capacity\":" << history_capacity_ << '}';
      Send(client, 200, "application/json", config.str());
      return;
    }
    if (path == "/") {
      path = "/index.html";
    }
    if (path.find("..") != std::string::npos) {
      Send(client, 400, "text/plain", "invalid path\n");
      return;
    }
    const auto file_path = std::filesystem::path(web_root_) / path.substr(1U);
    std::ifstream file(file_path, std::ios::binary);
    if (!file) {
      Send(client, 404, "text/plain", "not found\n");
      return;
    }
    std::ostringstream contents;
    contents << file.rdbuf();
    Send(client, 200, ContentType(file_path.extension().string()), contents.str());
  }

  static std::string ContentType(const std::string & extension)
  {
    if (extension == ".html") {return "text/html; charset=utf-8";}
    if (extension == ".css") {return "text/css; charset=utf-8";}
    if (extension == ".js") {return "application/javascript; charset=utf-8";}
    return "application/octet-stream";
  }

  static void Send(
    const int client, const int status, const std::string & content_type,
    const std::string & body)
  {
    const std::string label = status == 200 ? "OK" : "Error";
    std::ostringstream response;
    response << "HTTP/1.1 " << status << ' ' << label << "\r\n"
             << "Content-Type: " << content_type << "\r\n"
             << "Content-Length: " << body.size() << "\r\n"
             << "Cache-Control: no-store\r\n"
             << "X-Content-Type-Options: nosniff\r\n"
             << "Connection: close\r\n\r\n" << body;
    const auto payload = response.str();
    std::size_t sent = 0U;
    while (sent < payload.size()) {
      const auto result = send(client, payload.data() + sent, payload.size() - sent, MSG_NOSIGNAL);
      if (result <= 0) {
        break;
      }
      sent += static_cast<std::size_t>(result);
    }
  }

  std::string bind_address_;
  int port_{8765};
  int polling_interval_ms_{1000};
  int history_capacity_{120};
  std::string web_root_;
  int server_fd_{-1};
  std::atomic<bool> stopping_{false};
  std::thread server_thread_;
  std::mutex telemetry_mutex_;
  std::string telemetry_{
    "{\"schema\":\"savo_observer.telemetry.v1\",\"state\":\"starting\","
    "\"connected\":false,\"dependencies\":[],\"alerts\":[\"waiting_for_telemetry\"]}"};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetry_subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ObserverDashboardNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("observer_dashboard_node"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
