// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/graph_evidence.hpp"

#include <algorithm>
#include <cctype>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace savo_bridge
{
namespace
{

using NameSet = std::set<std::string>;

[[nodiscard]] bool is_valid_absolute_graph_name(
  const std::string_view name) noexcept
{
  if (name.size() < 2U) {
    return false;
  }

  if (name.front() != '/' || name.back() == '/') {
    return false;
  }

  if (name.find("//") != std::string_view::npos) {
    return false;
  }

  for (const char character : name) {
    const auto value = static_cast<unsigned char>(character);

    if (character != '/' &&
      character != '_' &&
      std::isalnum(value) == 0)
    {
      return false;
    }
  }

  return true;
}

[[nodiscard]] NameSet validate_unique_names(
  const std::vector<std::string> & names,
  const std::string & description,
  const bool require_non_empty)
{
  if (require_non_empty && names.empty()) {
    throw std::invalid_argument(
            description + " must not be empty");
  }

  NameSet result;

  for (const auto & name : names) {
    if (!is_valid_absolute_graph_name(name)) {
      throw std::invalid_argument(
              description +
              " contains an invalid absolute graph name");
    }

    if (!result.insert(name).second) {
      throw std::invalid_argument(
              description +
              " contains a duplicate graph name");
    }
  }

  return result;
}

void reject_overlap(
  const NameSet & left,
  const NameSet & right,
  const std::string & description)
{
  std::vector<std::string> overlap;

  std::set_intersection(
    left.begin(),
    left.end(),
    right.begin(),
    right.end(),
    std::back_inserter(overlap));

  if (!overlap.empty()) {
    throw std::invalid_argument(description);
  }
}

void copy_matches(
  const NameSet & selectors,
  const NameSet & discovered,
  std::vector<std::string> & output)
{
  std::set_intersection(
    selectors.begin(),
    selectors.end(),
    discovered.begin(),
    discovered.end(),
    std::back_inserter(output));
}

}  // namespace

GraphEvidence GraphEvidenceEvaluator::evaluate(
  const GraphEvidenceConfig & config,
  const GraphSnapshot & snapshot) const
{
  validate_config(config);
  validate_snapshot(snapshot);

  const NameSet discovered_nodes(
    snapshot.node_fully_qualified_names.begin(),
    snapshot.node_fully_qualified_names.end());

  NameSet discovered_topics;

  for (const auto & topic : snapshot.topics) {
    discovered_topics.insert(topic.topic_name);
  }

  const NameSet owned_topics(
    config.owned_topic_names.begin(),
    config.owned_topic_names.end());

  const NameSet core_nodes(
    config.core_node_names.begin(),
    config.core_node_names.end());

  const NameSet core_topics(
    config.core_topic_names.begin(),
    config.core_topic_names.end());

  const NameSet edge_nodes(
    config.edge_node_names.begin(),
    config.edge_node_names.end());

  const NameSet edge_topics(
    config.edge_topic_names.begin(),
    config.edge_topic_names.end());

  GraphEvidence result;
  result.discovered_nodes = discovered_nodes.size();
  result.discovered_topics = discovered_topics.size();

  result.core_evidence_configured =
    !core_nodes.empty() || !core_topics.empty();

  result.edge_evidence_configured =
    !edge_nodes.empty() || !edge_topics.empty();

  for (const auto & topic : owned_topics) {
    if (discovered_topics.count(topic) != 0U) {
      result.visible_owned_topics.push_back(topic);
    } else {
      result.missing_owned_topics.push_back(topic);
    }
  }

  const bool local_node_visible =
    discovered_nodes.count(
    config.local_node_fully_qualified_name) != 0U;

  result.dds_active =
    local_node_visible &&
    result.missing_owned_topics.empty();

  copy_matches(
    core_nodes,
    discovered_nodes,
    result.matched_core_nodes);

  copy_matches(
    core_topics,
    discovered_topics,
    result.matched_core_topics);

  copy_matches(
    edge_nodes,
    discovered_nodes,
    result.matched_edge_nodes);

  copy_matches(
    edge_topics,
    discovered_topics,
    result.matched_edge_topics);

  result.core_visible =
    result.core_evidence_configured &&
    (
    !result.matched_core_nodes.empty() ||
    !result.matched_core_topics.empty());

  result.edge_visible =
    result.edge_evidence_configured &&
    (
    !result.matched_edge_nodes.empty() ||
    !result.matched_edge_topics.empty());

  result.external_nodes =
    discovered_nodes.size() -
    (
    local_node_visible ? 1U : 0U);

  result.external_topics = discovered_topics.size();

  for (const auto & owned_topic : owned_topics) {
    if (discovered_topics.count(owned_topic) != 0U) {
      --result.external_topics;
    }
  }

  return result;
}

void GraphEvidenceEvaluator::validate_config(
  const GraphEvidenceConfig & config)
{
  if (!is_valid_absolute_graph_name(
      config.local_node_fully_qualified_name))
  {
    throw std::invalid_argument(
            "GraphEvidence local node name must be absolute");
  }

  const NameSet owned_topics = validate_unique_names(
    config.owned_topic_names,
    "GraphEvidence owned topics",
    true);

  const NameSet core_nodes = validate_unique_names(
    config.core_node_names,
    "GraphEvidence core nodes",
    false);

  const NameSet core_topics = validate_unique_names(
    config.core_topic_names,
    "GraphEvidence core topics",
    false);

  const NameSet edge_nodes = validate_unique_names(
    config.edge_node_names,
    "GraphEvidence edge nodes",
    false);

  const NameSet edge_topics = validate_unique_names(
    config.edge_topic_names,
    "GraphEvidence edge topics",
    false);

  if (core_nodes.count(
      config.local_node_fully_qualified_name) != 0U ||
    edge_nodes.count(
      config.local_node_fully_qualified_name) != 0U)
  {
    throw std::invalid_argument(
            "The local bridge node cannot prove "
            "external visibility");
  }

  reject_overlap(
    core_nodes,
    edge_nodes,
    "Core and edge node evidence selectors must not overlap");

  reject_overlap(
    core_topics,
    edge_topics,
    "Core and edge topic evidence selectors must not overlap");

  reject_overlap(
    owned_topics,
    core_topics,
    "Bridge-owned topics cannot be core evidence selectors");

  reject_overlap(
    owned_topics,
    edge_topics,
    "Bridge-owned topics cannot be edge evidence selectors");
}

void GraphEvidenceEvaluator::validate_snapshot(
  const GraphSnapshot & snapshot)
{
  (void)validate_unique_names(
    snapshot.node_fully_qualified_names,
    "Graph snapshot nodes",
    false);

  NameSet topic_names;

  for (const auto & topic : snapshot.topics) {
    if (!is_valid_absolute_graph_name(topic.topic_name)) {
      throw std::invalid_argument(
              "Graph snapshot contains an invalid topic name");
    }

    if (!topic_names.insert(topic.topic_name).second) {
      throw std::invalid_argument(
              "Graph snapshot contains a duplicate topic");
    }

    NameSet type_names;

    for (const auto & type_name : topic.type_names) {
      if (type_name.empty()) {
        throw std::invalid_argument(
                "Graph snapshot contains an empty topic type");
      }

      if (!type_names.insert(type_name).second) {
        throw std::invalid_argument(
                "Graph snapshot contains a duplicate topic type");
      }
    }
  }
}

}  // namespace savo_bridge
