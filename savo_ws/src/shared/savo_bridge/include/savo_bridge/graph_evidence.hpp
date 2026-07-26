// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#ifndef SAVO_BRIDGE__GRAPH_EVIDENCE_HPP_
#define SAVO_BRIDGE__GRAPH_EVIDENCE_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace savo_bridge
{

struct GraphTopicSnapshot
{
  std::string topic_name;
  std::vector<std::string> type_names;
};

struct GraphSnapshot
{
  std::vector<std::string> node_fully_qualified_names;
  std::vector<GraphTopicSnapshot> topics;
};

struct GraphEvidenceConfig
{
  std::string local_node_fully_qualified_name;
  std::vector<std::string> owned_topic_names;

  std::vector<std::string> core_node_names;
  std::vector<std::string> core_topic_names;

  std::vector<std::string> edge_node_names;
  std::vector<std::string> edge_topic_names;
};

struct GraphEvidence
{
  bool dds_active{false};

  bool core_evidence_configured{false};
  bool edge_evidence_configured{false};

  bool core_visible{false};
  bool edge_visible{false};

  std::size_t discovered_nodes{0U};
  std::size_t discovered_topics{0U};

  std::size_t external_nodes{0U};
  std::size_t external_topics{0U};

  std::vector<std::string> visible_owned_topics;
  std::vector<std::string> missing_owned_topics;

  std::vector<std::string> matched_core_nodes;
  std::vector<std::string> matched_core_topics;

  std::vector<std::string> matched_edge_nodes;
  std::vector<std::string> matched_edge_topics;
};

class GraphEvidenceEvaluator final
{
public:
  [[nodiscard]] GraphEvidence evaluate(
    const GraphEvidenceConfig & config,
    const GraphSnapshot & snapshot) const;

private:
  static void validate_config(
    const GraphEvidenceConfig & config);

  static void validate_snapshot(
    const GraphSnapshot & snapshot);
};

}  // namespace savo_bridge

#endif  // SAVO_BRIDGE__GRAPH_EVIDENCE_HPP_
