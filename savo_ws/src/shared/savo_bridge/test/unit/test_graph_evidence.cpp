// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "savo_bridge/graph_evidence.hpp"

namespace
{

[[nodiscard]] savo_bridge::GraphTopicSnapshot make_topic(
  std::string topic_name,
  std::vector<std::string> type_names)
{
  savo_bridge::GraphTopicSnapshot topic;
  topic.topic_name = std::move(topic_name);
  topic.type_names = std::move(type_names);
  return topic;
}

[[nodiscard]] savo_bridge::GraphEvidenceConfig make_config()
{
  savo_bridge::GraphEvidenceConfig config;

  config.local_node_fully_qualified_name =
    "/savo_bridge/savo_bridge_node";

  config.owned_topic_names = {
    "/savo_bridge/diagnostics",
    "/savo_bridge/heartbeat",
    "/savo_bridge/readiness",
    "/savo_bridge/state",
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

  return config;
}

[[nodiscard]] savo_bridge::GraphSnapshot make_snapshot()
{
  savo_bridge::GraphSnapshot snapshot;

  snapshot.node_fully_qualified_names = {
    "/phase1c/edge_probe",
    "/savo_bridge/savo_bridge_node",
    "/phase1c/core_probe",
  };

  snapshot.topics = {
    make_topic(
      "/savo_bridge/state",
      {"std_msgs/msg/String"}),
    make_topic(
      "/phase1c/edge_evidence",
      {"std_msgs/msg/String"}),
    make_topic(
      "/savo_bridge/readiness",
      {"std_msgs/msg/Bool"}),
    make_topic(
      "/phase1c/core_evidence",
      {"std_msgs/msg/String"}),
    make_topic(
      "/savo_bridge/heartbeat",
      {"std_msgs/msg/UInt64"}),
    make_topic(
      "/savo_bridge/diagnostics",
      {"diagnostic_msgs/msg/DiagnosticArray"}),
  };

  return snapshot;
}

TEST(SavoBridgeGraphEvidence, DeterministicEvidenceContract)
{
  const savo_bridge::GraphEvidenceEvaluator evaluator;
  const auto config = make_config();
  auto snapshot = make_snapshot();

  const auto evidence = evaluator.evaluate(config, snapshot);

  EXPECT_TRUE(evidence.dds_active);
  EXPECT_TRUE(evidence.core_evidence_configured);
  EXPECT_TRUE(evidence.edge_evidence_configured);
  EXPECT_TRUE(evidence.core_visible);
  EXPECT_TRUE(evidence.edge_visible);

  EXPECT_EQ(evidence.discovered_nodes, 3U);
  EXPECT_EQ(evidence.discovered_topics, 6U);
  EXPECT_EQ(evidence.external_nodes, 2U);
  EXPECT_EQ(evidence.external_topics, 2U);

  EXPECT_EQ(evidence.visible_owned_topics.size(), 4U);
  EXPECT_TRUE(evidence.missing_owned_topics.empty());

  ASSERT_EQ(evidence.matched_core_nodes.size(), 1U);
  EXPECT_EQ(
    evidence.matched_core_nodes.at(0),
    "/phase1c/core_probe");

  ASSERT_EQ(evidence.matched_core_topics.size(), 1U);
  EXPECT_EQ(
    evidence.matched_core_topics.at(0),
    "/phase1c/core_evidence");

  ASSERT_EQ(evidence.matched_edge_nodes.size(), 1U);
  EXPECT_EQ(
    evidence.matched_edge_nodes.at(0),
    "/phase1c/edge_probe");

  ASSERT_EQ(evidence.matched_edge_topics.size(), 1U);
  EXPECT_EQ(
    evidence.matched_edge_topics.at(0),
    "/phase1c/edge_evidence");

  std::reverse(
    snapshot.node_fully_qualified_names.begin(),
    snapshot.node_fully_qualified_names.end());

  std::reverse(
    snapshot.topics.begin(),
    snapshot.topics.end());

  const auto reordered =
    evaluator.evaluate(config, snapshot);

  EXPECT_EQ(reordered.dds_active, evidence.dds_active);
  EXPECT_EQ(
    reordered.core_visible,
    evidence.core_visible);
  EXPECT_EQ(
    reordered.edge_visible,
    evidence.edge_visible);

  EXPECT_EQ(
    reordered.matched_core_nodes,
    evidence.matched_core_nodes);

  EXPECT_EQ(
    reordered.matched_core_topics,
    evidence.matched_core_topics);

  EXPECT_EQ(
    reordered.matched_edge_nodes,
    evidence.matched_edge_nodes);

  EXPECT_EQ(
    reordered.matched_edge_topics,
    evidence.matched_edge_topics);

  auto missing_owned_snapshot = make_snapshot();

  missing_owned_snapshot.topics.erase(
    std::remove_if(
      missing_owned_snapshot.topics.begin(),
      missing_owned_snapshot.topics.end(),
      [](const auto & topic) {
        return topic.topic_name == "/savo_bridge/state";
      }),
    missing_owned_snapshot.topics.end());

  const auto missing_owned =
    evaluator.evaluate(config, missing_owned_snapshot);

  EXPECT_FALSE(missing_owned.dds_active);

  ASSERT_EQ(
    missing_owned.missing_owned_topics.size(),
    1U);

  EXPECT_EQ(
    missing_owned.missing_owned_topics.at(0),
    "/savo_bridge/state");

  auto missing_edge_snapshot = make_snapshot();

  missing_edge_snapshot.node_fully_qualified_names.erase(
    std::remove(
      missing_edge_snapshot.node_fully_qualified_names.begin(),
      missing_edge_snapshot.node_fully_qualified_names.end(),
      "/phase1c/edge_probe"),
    missing_edge_snapshot.node_fully_qualified_names.end());

  missing_edge_snapshot.topics.erase(
    std::remove_if(
      missing_edge_snapshot.topics.begin(),
      missing_edge_snapshot.topics.end(),
      [](const auto & topic) {
        return topic.topic_name ==
               "/phase1c/edge_evidence";
      }),
    missing_edge_snapshot.topics.end());

  const auto missing_edge =
    evaluator.evaluate(config, missing_edge_snapshot);

  EXPECT_TRUE(missing_edge.dds_active);
  EXPECT_TRUE(missing_edge.core_visible);
  EXPECT_FALSE(missing_edge.edge_visible);

  auto unconfigured = config;
  unconfigured.core_node_names.clear();
  unconfigured.core_topic_names.clear();
  unconfigured.edge_node_names.clear();
  unconfigured.edge_topic_names.clear();

  const auto no_selectors =
    evaluator.evaluate(unconfigured, make_snapshot());

  EXPECT_FALSE(no_selectors.core_evidence_configured);
  EXPECT_FALSE(no_selectors.edge_evidence_configured);
  EXPECT_FALSE(no_selectors.core_visible);
  EXPECT_FALSE(no_selectors.edge_visible);

  auto duplicate_nodes = make_snapshot();

  duplicate_nodes.node_fully_qualified_names.push_back(
    "/phase1c/core_probe");

  EXPECT_THROW(
    (void)evaluator.evaluate(config, duplicate_nodes),
    std::invalid_argument);

  auto invalid_config = config;

  invalid_config.core_node_names = {
    "/phase1c/shared_probe",
  };

  invalid_config.edge_node_names = {
    "/phase1c/shared_probe",
  };

  EXPECT_THROW(
    (void)evaluator.evaluate(
      invalid_config,
      make_snapshot()),
    std::invalid_argument);

  invalid_config = config;

  invalid_config.core_node_names = {
    config.local_node_fully_qualified_name,
  };

  EXPECT_THROW(
    (void)evaluator.evaluate(
      invalid_config,
      make_snapshot()),
    std::invalid_argument);

  invalid_config = config;

  invalid_config.core_topic_names = {
    "/savo_bridge/state",
  };

  EXPECT_THROW(
    (void)evaluator.evaluate(
      invalid_config,
      make_snapshot()),
    std::invalid_argument);

  invalid_config = config;

  invalid_config.local_node_fully_qualified_name =
    "relative/node";

  EXPECT_THROW(
    (void)evaluator.evaluate(
      invalid_config,
      make_snapshot()),
    std::invalid_argument);

  auto invalid_snapshot = make_snapshot();

  invalid_snapshot.topics.at(0).type_names.push_back(
    "std_msgs/msg/String");

  EXPECT_THROW(
    (void)evaluator.evaluate(config, invalid_snapshot),
    std::invalid_argument);
}

}  // namespace
