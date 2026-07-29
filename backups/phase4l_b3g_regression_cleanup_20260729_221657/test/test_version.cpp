#include "savo_mapping/version.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(VersionContract, PackageIdentityIsStable)
{
  EXPECT_EQ(std::string{savo_mapping::version::get_package_name()}, "savo_mapping");
  EXPECT_EQ(std::string{savo_mapping::version::get_robot_name()}, "Robot Savo");
  EXPECT_EQ(std::string{savo_mapping::version::get_project_name()}, "Robot SAVO Project");
  EXPECT_EQ(std::string{savo_mapping::version::get_ros_distro()}, "jazzy");
}

TEST(VersionContract, VersionStringMatchesTuple)
{
  EXPECT_EQ(std::string{savo_mapping::version::get_version()}, "0.1.0");

  const auto [major, minor, patch] = savo_mapping::version::get_version_tuple();

  EXPECT_EQ(major, 0);
  EXPECT_EQ(minor, 1);
  EXPECT_EQ(patch, 0);
}
