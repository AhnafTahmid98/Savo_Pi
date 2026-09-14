// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "savo_supervisor/system_state_store.hpp"
#include "savo_supervisor/system_authority.hpp"

TEST(SystemStateStore, SavesAndLoadsFaultLatchAtomically)
{
  const auto directory = std::filesystem::temp_directory_path() /
    "savo_supervisor_state_store_test";
  std::filesystem::remove_all(directory);
  const auto path = directory / "state.json";
  savo_supervisor::SystemStateStore store(path.string());
  std::string error;
  ASSERT_TRUE(store.Save(true, 7U, "core_fault_latched", error)) << error;
  const auto loaded = store.Load();
  EXPECT_TRUE(loaded.valid);
  EXPECT_TRUE(loaded.fault_latched);
  EXPECT_EQ(loaded.generation, 7U);
  EXPECT_EQ(loaded.reason, "core_fault_latched");
  std::filesystem::remove_all(directory);
}

TEST(SystemStateStore, MalformedStateFailsClosedWithoutThrowing)
{
  const auto path = std::filesystem::temp_directory_path() /
    "savo_supervisor_malformed_state.json";
  {
    std::ofstream stream(path);
    stream << "not-json";
  }
  const auto loaded = savo_supervisor::SystemStateStore(path.string()).Load();
  EXPECT_FALSE(loaded.valid);
  EXPECT_FALSE(loaded.error.empty());
  std::filesystem::remove(path);
}

TEST(SystemStateStore, QualifiedFaultEvidenceSurvivesRestartAndExplicitClearStaysClear)
{
  const auto path = std::filesystem::temp_directory_path() /
    "savo_supervisor_qualified_fault_test.json";
  savo_supervisor::SystemStateStore store(path.string());
  savo_supervisor::SystemAuthority authority;
  savo_supervisor::SystemDependencySnapshot healthy;
  healthy.core_ready = true;
  healthy.safety_known = true;
  healthy.startup_dependencies_ready = true;
  savo_supervisor::SystemAuthorityRequest command;
  command.command = savo_supervisor::SystemCommand::kArm;
  command.actor_id = "operator";
  command.request_id = "verified";
  ASSERT_TRUE(authority.Handle(command, healthy).accepted);
  auto critical = healthy;
  critical.core_ready = false;
  critical.core_fault = {savo_supervisor::CoreFaultKind::kCritical,
    "base_battery:ERROR:base_battery_critical"};
  ASSERT_TRUE(authority.Update(critical));
  const auto snapshot = authority.snapshot(critical);
  std::string error;
  ASSERT_TRUE(store.Save(snapshot.fault_latched, snapshot.generation, snapshot.reason, error));
  auto saved = store.Load();
  ASSERT_TRUE(saved.valid);
  EXPECT_EQ(saved.reason, "core_fault_latched:base_battery:ERROR:base_battery_critical");
  savo_supervisor::SystemAuthority restarted;
  restarted.RestoreFaultLatch(saved.fault_latched, saved.generation, saved.reason);
  EXPECT_TRUE(restarted.snapshot(healthy).fault_latched);
  EXPECT_EQ(restarted.snapshot(healthy).reason, saved.reason);
  command.command = savo_supervisor::SystemCommand::kClearFaultLatch;
  ASSERT_TRUE(restarted.Handle(command, healthy).accepted);
  const auto cleared = restarted.snapshot(healthy);
  ASSERT_TRUE(store.Save(cleared.fault_latched, cleared.generation, cleared.reason, error));
  saved = store.Load();
  ASSERT_TRUE(saved.valid);
  restarted.RestoreFaultLatch(saved.fault_latched, saved.generation, saved.reason);
  EXPECT_FALSE(restarted.Update(healthy));
  EXPECT_FALSE(restarted.snapshot(healthy).fault_latched);
  EXPECT_FALSE(restarted.snapshot(healthy).armed);
  std::filesystem::remove(path);
}
