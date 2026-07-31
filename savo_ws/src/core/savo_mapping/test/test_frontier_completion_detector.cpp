#include "savo_mapping/frontier_completion_detector.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{

using savo_mapping::autonomous::FrontierCompletionConfig;
using savo_mapping::autonomous::FrontierCompletionDetector;
using savo_mapping::autonomous::FrontierExhaustionKind;
using savo_mapping::autonomous::FrontierPlanObservation;

FrontierCompletionConfig fast_config()
{
  FrontierCompletionConfig config;
  config.minimum_observations = 3U;
  config.minimum_stable_duration_s = 2.0;
  config.status_timeout_s = 1.0;
  return config;
}

FrontierPlanObservation no_frontiers(
  const std::uint64_t sequence,
  const double received_at_s)
{
  FrontierPlanObservation observation;
  observation.received = true;
  observation.runtime_enabled = true;
  observation.plan_sequence = sequence;
  observation.map_generation = 10U;
  observation.planned_map_generation = 10U;
  observation.planning_status = "no_frontiers";
  observation.received_at_s = received_at_s;
  return observation;
}

}  // namespace

TEST(FrontierCompletionDetectorTest, RejectsInvalidConfiguration)
{
  auto config = fast_config();
  config.minimum_observations = 0U;

  EXPECT_THROW(
    FrontierCompletionDetector detector(config),
    std::invalid_argument);
}

TEST(FrontierCompletionDetectorTest, RequiresDistinctStableObservations)
{
  FrontierCompletionDetector detector(fast_config());

  auto snapshot = detector.observe(no_frontiers(1U, 10.0), 10.0);
  EXPECT_TRUE(snapshot.candidate);
  EXPECT_FALSE(snapshot.confirmed);
  EXPECT_EQ(snapshot.observations, 1U);

  snapshot = detector.observe(no_frontiers(1U, 10.5), 10.5);
  EXPECT_EQ(snapshot.observations, 1U);

  snapshot = detector.observe(no_frontiers(2U, 11.0), 11.0);
  EXPECT_EQ(snapshot.observations, 2U);
  EXPECT_FALSE(snapshot.confirmed);

  snapshot = detector.observe(no_frontiers(3U, 12.1), 12.1);
  EXPECT_TRUE(snapshot.confirmed);
  EXPECT_EQ(
    snapshot.exhaustion_kind,
    FrontierExhaustionKind::NoFrontiers);
  EXPECT_GE(snapshot.stable_duration_s, 2.0);
}

TEST(FrontierCompletionDetectorTest, MapChangeRestartsStableEvidence)
{
  FrontierCompletionDetector detector(fast_config());

  detector.observe(no_frontiers(1U, 1.0), 1.0);
  auto snapshot = detector.observe(no_frontiers(2U, 2.0), 2.0);
  ASSERT_EQ(snapshot.observations, 2U);

  auto changed_map = no_frontiers(3U, 3.2);
  changed_map.map_generation = 11U;
  changed_map.planned_map_generation = 11U;
  snapshot = detector.observe(changed_map, 3.2);

  EXPECT_TRUE(snapshot.candidate);
  EXPECT_FALSE(snapshot.confirmed);
  EXPECT_EQ(snapshot.observations, 1U);
  EXPECT_EQ(snapshot.stable_duration_s, 0.0);
}

TEST(FrontierCompletionDetectorTest, FrontierReappearanceResetsCandidate)
{
  FrontierCompletionDetector detector(fast_config());
  detector.observe(no_frontiers(1U, 1.0), 1.0);

  auto available = no_frontiers(2U, 1.5);
  available.planning_status = "goal_selected";
  available.detected_frontiers = 2U;
  available.reachable_frontiers = 2U;

  const auto snapshot = detector.observe(available, 1.5);
  EXPECT_FALSE(snapshot.candidate);
  EXPECT_FALSE(snapshot.confirmed);
  EXPECT_EQ(snapshot.observations, 0U);
}

TEST(FrontierCompletionDetectorTest, AcceptsStableUnreachableExhaustion)
{
  FrontierCompletionDetector detector(fast_config());

  FrontierPlanObservation observation;
  observation.received = true;
  observation.runtime_enabled = true;
  observation.planning_status = "no_reachable_frontiers";
  observation.detected_frontiers = 4U;
  observation.reachable_frontiers = 0U;

  observation.plan_sequence = 1U;
  observation.received_at_s = 4.0;
  detector.observe(observation, 4.0);

  observation.plan_sequence = 2U;
  observation.received_at_s = 5.0;
  detector.observe(observation, 5.0);

  observation.plan_sequence = 3U;
  observation.received_at_s = 6.1;
  const auto snapshot = detector.observe(observation, 6.1);

  EXPECT_TRUE(snapshot.confirmed);
  EXPECT_EQ(
    snapshot.exhaustion_kind,
    FrontierExhaustionKind::NoReachableFrontiers);
}

TEST(FrontierCompletionDetectorTest, ActiveGoalBlocksExhaustion)
{
  FrontierCompletionDetector detector(fast_config());
  auto observation = no_frontiers(1U, 2.0);
  observation.goal_pending = true;

  const auto snapshot = detector.observe(observation, 2.0);
  EXPECT_FALSE(snapshot.candidate);
  EXPECT_FALSE(snapshot.confirmed);
}

TEST(FrontierCompletionDetectorTest, StaleCandidateResetsBeforeConfirmation)
{
  FrontierCompletionDetector detector(fast_config());
  detector.observe(no_frontiers(1U, 1.0), 1.0);

  const auto snapshot = detector.tick(2.2);
  EXPECT_FALSE(snapshot.status_fresh);
  EXPECT_FALSE(snapshot.candidate);
  EXPECT_FALSE(snapshot.confirmed);
}

TEST(FrontierCompletionDetectorTest, ConfirmedEvidenceRemainsLatched)
{
  auto config = fast_config();
  config.minimum_observations = 1U;
  config.minimum_stable_duration_s = 0.0;
  FrontierCompletionDetector detector(config);

  const auto confirmed = detector.observe(no_frontiers(1U, 1.0), 1.0);
  ASSERT_TRUE(confirmed.confirmed);

  const auto stale = detector.tick(3.0);
  EXPECT_FALSE(stale.status_fresh);
  EXPECT_TRUE(stale.confirmed);
}

TEST(FrontierCompletionDetectorTest, NoSelectableIsDisabledByDefault)
{
  FrontierCompletionDetector detector(fast_config());
  auto observation = no_frontiers(1U, 2.0);
  observation.planning_status = "no_selectable_frontier";
  observation.detected_frontiers = 3U;
  observation.reachable_frontiers = 2U;

  const auto snapshot = detector.observe(observation, 2.0);
  EXPECT_FALSE(snapshot.candidate);
  EXPECT_FALSE(snapshot.confirmed);
}
