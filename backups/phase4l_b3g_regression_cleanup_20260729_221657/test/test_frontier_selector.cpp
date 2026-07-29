#include "savo_mapping/frontier_selector.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

using savo_mapping::frontier::FrontierCluster;
using savo_mapping::frontier::FrontierSelector;
using savo_mapping::frontier::FrontierSelectorConfig;
using savo_mapping::frontier::GridCell;

FrontierCluster make_frontier(
  double x_m,
  double y_m,
  double information_gain_m2,
  GridCell representative)
{
  FrontierCluster frontier;
  frontier.cells.push_back(representative);
  frontier.representative = representative;
  frontier.centroid_x_m = x_m;
  frontier.centroid_y_m = y_m;
  frontier.information_gain_m2 = information_gain_m2;
  return frontier;
}

}  // namespace

TEST(FrontierSelector, RejectsInvalidConfiguration)
{
  FrontierSelectorConfig config;
  config.distance_weight = -1.0;

  EXPECT_THROW(
    static_cast<void>(FrontierSelector{config}),
    std::invalid_argument);
}

TEST(FrontierSelector, RejectsNonfiniteRobotPose)
{
  const FrontierSelector selector;
  const std::vector<FrontierCluster> frontiers;

  EXPECT_THROW(
    selector.select(
      frontiers,
      std::numeric_limits<double>::quiet_NaN(),
      0.0),
    std::invalid_argument);
}

TEST(FrontierSelector, EmptyInputHasNoSelection)
{
  const FrontierSelector selector;
  const std::vector<FrontierCluster> frontiers;

  EXPECT_FALSE(selector.select(frontiers, 0.0, 0.0).has_value());
}

TEST(FrontierSelector, InformationGainCanOutweighDistance)
{
  const std::vector<FrontierCluster> frontiers{
    make_frontier(1.0, 0.0, 1.0, GridCell{1, 0}),
    make_frontier(2.0, 0.0, 5.0, GridCell{2, 0}),
  };

  const FrontierSelector selector;
  const auto selection = selector.select(frontiers, 0.0, 0.0);

  ASSERT_TRUE(selection.has_value());
  EXPECT_EQ(selection->frontier_index, 1u);
  EXPECT_DOUBLE_EQ(selection->distance_m, 2.0);
  EXPECT_DOUBLE_EQ(selection->score, 3.0);
}

TEST(FrontierSelector, DistancePenaltyCanPreferNearerFrontier)
{
  FrontierSelectorConfig config;
  config.information_gain_weight = 0.0;
  config.distance_weight = 1.0;

  const std::vector<FrontierCluster> frontiers{
    make_frontier(3.0, 0.0, 100.0, GridCell{3, 0}),
    make_frontier(1.0, 0.0, 1.0, GridCell{1, 0}),
  };

  const FrontierSelector selector(config);
  const auto selection = selector.select(frontiers, 0.0, 0.0);

  ASSERT_TRUE(selection.has_value());
  EXPECT_EQ(selection->frontier_index, 1u);
}

TEST(FrontierSelector, MinimumInformationGainFiltersCandidates)
{
  FrontierSelectorConfig config;
  config.minimum_information_gain_m2 = 2.0;

  const std::vector<FrontierCluster> frontiers{
    make_frontier(1.0, 0.0, 1.0, GridCell{1, 0}),
    make_frontier(2.0, 0.0, 3.0, GridCell{2, 0}),
  };

  const FrontierSelector selector(config);
  const auto selection = selector.select(frontiers, 0.0, 0.0);

  ASSERT_TRUE(selection.has_value());
  EXPECT_EQ(selection->frontier_index, 1u);
}

TEST(FrontierSelector, MaximumDistanceFiltersCandidates)
{
  FrontierSelectorConfig config;
  config.maximum_distance_m = 1.5;

  const std::vector<FrontierCluster> frontiers{
    make_frontier(2.0, 0.0, 10.0, GridCell{2, 0}),
    make_frontier(1.0, 0.0, 1.0, GridCell{1, 0}),
  };

  const FrontierSelector selector(config);
  const auto selection = selector.select(frontiers, 0.0, 0.0);

  ASSERT_TRUE(selection.has_value());
  EXPECT_EQ(selection->frontier_index, 1u);
}

TEST(FrontierSelector, InvalidFrontierIsIgnored)
{
  FrontierCluster invalid =
    make_frontier(1.0, 0.0, 1.0, GridCell{1, 0});
  invalid.cells.clear();

  const std::vector<FrontierCluster> frontiers{
    invalid,
    make_frontier(2.0, 0.0, 2.0, GridCell{2, 0}),
  };

  const FrontierSelector selector;
  const auto selection = selector.select(frontiers, 0.0, 0.0);

  ASSERT_TRUE(selection.has_value());
  EXPECT_EQ(selection->frontier_index, 1u);
}

TEST(FrontierSelector, TieBreaksByRepresentativeRowMajorOrder)
{
  FrontierSelectorConfig config;
  config.information_gain_weight = 0.0;
  config.distance_weight = 0.0;

  const std::vector<FrontierCluster> frontiers{
    make_frontier(0.0, 0.0, 1.0, GridCell{5, 2}),
    make_frontier(0.0, 0.0, 1.0, GridCell{4, 1}),
    make_frontier(0.0, 0.0, 1.0, GridCell{2, 1}),
  };

  const FrontierSelector selector(config);
  const auto selection = selector.select(frontiers, 0.0, 0.0);

  ASSERT_TRUE(selection.has_value());
  EXPECT_EQ(selection->frontier_index, 2u);
}

TEST(FrontierSelector, ExactDuplicateTieUsesInputIndex)
{
  FrontierSelectorConfig config;
  config.information_gain_weight = 0.0;
  config.distance_weight = 0.0;

  const std::vector<FrontierCluster> frontiers{
    make_frontier(0.0, 0.0, 1.0, GridCell{1, 1}),
    make_frontier(0.0, 0.0, 1.0, GridCell{1, 1}),
  };

  const FrontierSelector selector(config);
  const auto selection = selector.select(frontiers, 0.0, 0.0);

  ASSERT_TRUE(selection.has_value());
  EXPECT_EQ(selection->frontier_index, 0u);
}
