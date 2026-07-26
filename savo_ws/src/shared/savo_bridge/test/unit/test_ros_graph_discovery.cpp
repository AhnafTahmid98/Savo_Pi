// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "savo_bridge/graph_evidence.hpp"
#include "savo_bridge/ros_graph_discovery.hpp"

namespace
{

class RclcppGuard final
{
public:
  RclcppGuard()
  {
    int argument_count = 0;
    char ** arguments = nullptr;

    rclcpp::init(
      argument_count,
      arguments);
  }

  ~RclcppGuard()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  RclcppGuard(const RclcppGuard &) = delete;
  RclcppGuard & operator=(const RclcppGuard &) = delete;
};

TEST(SavoBridgeRosGraphDiscovery, DiscoversConfiguredDdsEvidence)
{
  const RclcppGuard guard;

  auto observer = std::make_shared<rclcpp::Node>(
    "graph_observer",
    "/savo_bridge_test");

  auto core_probe = std::make_shared<rclcpp::Node>(
    "core_probe",
    "/phase1c");

  auto edge_probe = std::make_shared<rclcpp::Node>(
    "edge_probe",
    "/phase1c");

  auto owned_publisher =
    observer->create_publisher<std_msgs::msg::String>(
    "/savo_bridge_test/owned",
    rclcpp::QoS(1));

  auto core_publisher =
    core_probe->create_publisher<std_msgs::msg::String>(
    "/phase1c/core_evidence",
    rclcpp::QoS(1));

  auto edge_publisher =
    edge_probe->create_publisher<std_msgs::msg::String>(
    "/phase1c/edge_evidence",
    rclcpp::QoS(1));

  (void)owned_publisher;
  (void)core_publisher;
  (void)edge_publisher;

  savo_bridge::GraphEvidenceConfig config;

  config.local_node_fully_qualified_name =
    "/savo_bridge_test/graph_observer";

  config.owned_topic_names = {
    "/savo_bridge_test/owned",
  };

  config.core_node_names = {
    "/phase1c/core_probe",
  };

  config.core_topic_names = {
    "/phase1c/core_evidence",
  };

  config.edge_node_names = {
    "/phase1c/edge_probe",
  };

  config.edge_topic_names = {
    "/phase1c/edge_evidence",
  };

  const savo_bridge::GraphEvidenceEvaluator evaluator;

  savo_bridge::GraphEvidence evidence;

  const auto deadline =
    std::chrono::steady_clock::now() +
    std::chrono::seconds(3);

  do {
    const auto snapshot =
      savo_bridge::collect_ros_graph_snapshot(*observer);

    evidence = evaluator.evaluate(config, snapshot);

    if (evidence.dds_active &&
      evidence.core_visible &&
      evidence.edge_visible)
    {
      break;
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(25));
  } while (
    std::chrono::steady_clock::now() < deadline);

  EXPECT_TRUE(evidence.dds_active);
  EXPECT_TRUE(evidence.core_visible);
  EXPECT_TRUE(evidence.edge_visible);

  EXPECT_FALSE(evidence.matched_core_nodes.empty());
  EXPECT_FALSE(evidence.matched_core_topics.empty());
  EXPECT_FALSE(evidence.matched_edge_nodes.empty());
  EXPECT_FALSE(evidence.matched_edge_topics.empty());
}

}  // namespace
