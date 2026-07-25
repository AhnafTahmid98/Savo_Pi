// Copyright 2026 Ahnaf Tahmid
//
// Proprietary Robot Savo source code.

#include <gtest/gtest.h>

#include <string>

#include "savo_bridge/version.hpp"

namespace
{

TEST(SavoBridgeVersion, PackageVersionContract)
{
  EXPECT_EQ(savo_bridge::kVersionMajor, 0);
  EXPECT_EQ(savo_bridge::kVersionMinor, 1);
  EXPECT_EQ(savo_bridge::kVersionPatch, 0);
  EXPECT_EQ(savo_bridge::version_string(), std::string("0.1.0"));
}

}  // namespace
