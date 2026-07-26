// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include "savo_bridge/health_evaluator.hpp"

#include <set>
#include <stdexcept>
#include <string>

namespace savo_bridge
{
namespace
{

using TopicNameSet = std::set<std::string>;

void copy_topic_names(
  const TopicNameSet & source,
  std::vector<std::string> & destination)
{
  destination.assign(source.begin(), source.end());
}

}  // namespace

HealthEvaluation HealthEvaluator::evaluate(
  const std::vector<TopicSnapshot> & topics) const
{
  HealthEvaluation result;
  result.total_topics = topics.size();

  if (topics.empty()) {
    return result;
  }

  TopicNameSet observed_topic_names;
  TopicNameSet required_unavailable;
  TopicNameSet optional_unavailable;
  TopicNameSet clock_regressions;

  for (const auto & topic : topics) {
    validate_snapshot(topic);

    const auto insertion =
      observed_topic_names.insert(topic.topic_name);

    if (!insertion.second) {
      throw std::invalid_argument(
              "HealthEvaluator topic names must be unique");
    }

    if (topic.requirement == TopicRequirement::kRequired) {
      ++result.required_topics;
    } else {
      ++result.optional_topics;
    }

    switch (topic.classification) {
      case TopicClassification::kNeverObserved:
        ++result.never_observed_topics;

        if (topic.requirement == TopicRequirement::kRequired) {
          required_unavailable.insert(topic.topic_name);
        } else {
          optional_unavailable.insert(topic.topic_name);
        }
        break;

      case TopicClassification::kFresh:
        ++result.fresh_topics;
        break;

      case TopicClassification::kStale:
        ++result.stale_topics;

        if (topic.requirement == TopicRequirement::kRequired) {
          required_unavailable.insert(topic.topic_name);
        } else {
          optional_unavailable.insert(topic.topic_name);
        }
        break;

      case TopicClassification::kClockRegression:
        ++result.clock_regression_topics;
        clock_regressions.insert(topic.topic_name);
        break;
    }
  }

  copy_topic_names(
    required_unavailable,
    result.required_unavailable_topics);

  copy_topic_names(
    optional_unavailable,
    result.optional_unavailable_topics);

  copy_topic_names(
    clock_regressions,
    result.clock_regression_topic_names);

  if (!clock_regressions.empty()) {
    result.status = HealthStatus::kUnhealthy;
    result.reason = HealthReason::kClockRegression;
    return result;
  }

  if (!required_unavailable.empty()) {
    result.status = HealthStatus::kUnhealthy;
    result.reason = HealthReason::kRequiredTopicUnavailable;
    return result;
  }

  result.required_topics_ready = true;

  if (!optional_unavailable.empty()) {
    result.status = HealthStatus::kDegraded;
    result.reason = HealthReason::kOptionalTopicUnavailable;
    return result;
  }

  result.status = HealthStatus::kHealthy;
  result.reason = HealthReason::kAllTopicsFresh;
  result.all_topics_fresh = true;

  return result;
}

void HealthEvaluator::validate_snapshot(
  const TopicSnapshot & snapshot)
{
  if (snapshot.topic_name.size() < 2U ||
    snapshot.topic_name.front() != '/')
  {
    throw std::invalid_argument(
            "HealthEvaluator requires absolute topic names");
  }

  if (snapshot.stale_after <= TopicObservation::Duration::zero()) {
    throw std::invalid_argument(
            "HealthEvaluator stale_after must be positive");
  }

  if (snapshot.age < TopicObservation::Duration::zero()) {
    throw std::invalid_argument(
            "HealthEvaluator topic age cannot be negative");
  }

  switch (snapshot.classification) {
    case TopicClassification::kNeverObserved:
      if (snapshot.observed ||
        snapshot.accepted_observations != 0U ||
        snapshot.age != TopicObservation::Duration::zero())
      {
        throw std::invalid_argument(
                "Invalid never-observed topic snapshot");
      }
      break;

    case TopicClassification::kFresh:
      if (!snapshot.observed ||
        snapshot.accepted_observations == 0U ||
        snapshot.age >= snapshot.stale_after)
      {
        throw std::invalid_argument(
                "Invalid fresh topic snapshot");
      }
      break;

    case TopicClassification::kStale:
      if (!snapshot.observed ||
        snapshot.accepted_observations == 0U ||
        snapshot.age < snapshot.stale_after)
      {
        throw std::invalid_argument(
                "Invalid stale topic snapshot");
      }
      break;

    case TopicClassification::kClockRegression:
      if (!snapshot.observed ||
        snapshot.accepted_observations == 0U ||
        snapshot.age != TopicObservation::Duration::zero())
      {
        throw std::invalid_argument(
                "Invalid clock-regression topic snapshot");
      }
      break;
  }
}

}  // namespace savo_bridge
