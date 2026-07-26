// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/ros_graph_discovery.hpp"

#include <algorithm>
#include <string>
#include <utility>

namespace savo_bridge
{
namespace
{

[[nodiscard]] std::string make_fully_qualified_node_name(
  const std::string & node_namespace,
  const std::string & node_name)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + node_name;
  }

  if (node_namespace.back() == '/') {
    return node_namespace + node_name;
  }

  return node_namespace + "/" + node_name;
}

}  // namespace

GraphSnapshot collect_ros_graph_snapshot(
  rclcpp::Node & node)
{
  GraphSnapshot snapshot;

  const auto graph_interface =
    node.get_node_graph_interface();

  for (const auto & node_identity :
    graph_interface->get_node_names_and_namespaces())
  {
    snapshot.node_fully_qualified_names.push_back(
      make_fully_qualified_node_name(
        node_identity.second,
        node_identity.first));
  }

  std::sort(
    snapshot.node_fully_qualified_names.begin(),
    snapshot.node_fully_qualified_names.end());

  snapshot.node_fully_qualified_names.erase(
    std::unique(
      snapshot.node_fully_qualified_names.begin(),
      snapshot.node_fully_qualified_names.end()),
    snapshot.node_fully_qualified_names.end());

  const auto topics_and_types =
    graph_interface->get_topic_names_and_types();

  snapshot.topics.reserve(topics_and_types.size());

  for (const auto & topic_and_types : topics_and_types) {
    GraphTopicSnapshot topic;
    topic.topic_name = topic_and_types.first;
    topic.type_names = topic_and_types.second;

    std::sort(
      topic.type_names.begin(),
      topic.type_names.end());

    topic.type_names.erase(
      std::unique(
        topic.type_names.begin(),
        topic.type_names.end()),
      topic.type_names.end());

    snapshot.topics.push_back(std::move(topic));
  }

  std::sort(
    snapshot.topics.begin(),
    snapshot.topics.end(),
    [](const auto & left, const auto & right) {
      return left.topic_name < right.topic_name;
    });

  return snapshot;
}

}  // namespace savo_bridge
