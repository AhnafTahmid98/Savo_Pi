// Copyright 2026 Ahnaf Tahmid
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "savo_supervisor/system_state_store.hpp"

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
