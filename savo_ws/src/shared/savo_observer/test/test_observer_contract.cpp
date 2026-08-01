// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <string>

#include "gtest/gtest.h"
#include "savo_observer/observer_contract.hpp"

TEST(ObserverContract, StableStateSerialization)
{
  using savo_observer::DependencyState;
  using savo_observer::ObserverState;
  EXPECT_EQ(savo_observer::ToString(ObserverState::kConnected), "connected");
  EXPECT_EQ(savo_observer::ToString(ObserverState::kShuttingDown), "shutting_down");
  EXPECT_EQ(savo_observer::ToString(DependencyState::kUnavailable), "unavailable");
  EXPECT_EQ(savo_observer::ParseObserverState("degraded"), ObserverState::kDegraded);
  EXPECT_EQ(savo_observer::ParseDependencyState("stale"), DependencyState::kStale);
  EXPECT_FALSE(savo_observer::ParseObserverState("ready"));
}

TEST(ObserverContract, RejectsControlInterfaces)
{
  EXPECT_TRUE(savo_observer::IsProhibitedInterface("/cmd_vel"));
  EXPECT_TRUE(savo_observer::IsProhibitedInterface("/cmd_vel/manual"));
  EXPECT_TRUE(savo_observer::IsProhibitedInterface("/goal_pose"));
  EXPECT_TRUE(savo_observer::IsProhibitedInterface("/savo_mapping/autonomous/run"));
  EXPECT_FALSE(savo_observer::IsProhibitedInterface("/savo_nav/readiness"));
  EXPECT_FALSE(savo_observer::IsProhibitedInterface("/scan"));
}

TEST(ObserverContract, PublicationsStayInObserverNamespace)
{
  EXPECT_TRUE(savo_observer::ValidateOutputNamespace("/savo_observer").empty());
  EXPECT_FALSE(savo_observer::ValidateOutputNamespace("savo_observer").empty());
  EXPECT_FALSE(savo_observer::ValidateOutputNamespace("/savo_observer/").empty());
  EXPECT_FALSE(savo_observer::ValidateOutputNamespace("/savo_control").empty());
}
