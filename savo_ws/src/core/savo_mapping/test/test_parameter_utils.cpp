#include "savo_mapping/parameter_utils.hpp"

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

class ParameterUtilsRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST(ParameterUtilsContract, ValidatesParameterNames)
{
  EXPECT_TRUE(savo_mapping::params::is_valid_parameter_name("publish_rate_hz"));
  EXPECT_TRUE(savo_mapping::params::is_valid_parameter_name("quality.threshold"));
  EXPECT_TRUE(
    savo_mapping::params::is_valid_parameter_name(
      "map_session.root_directory"));

  EXPECT_FALSE(savo_mapping::params::is_valid_parameter_name(""));
  EXPECT_FALSE(savo_mapping::params::is_valid_parameter_name("   "));
  EXPECT_FALSE(savo_mapping::params::is_valid_parameter_name("\t\n"));
  EXPECT_FALSE(savo_mapping::params::is_valid_parameter_name(" map_name "));
  EXPECT_FALSE(
    savo_mapping::params::is_valid_parameter_name(
      "quality threshold"));
}

TEST_F(ParameterUtilsRosTest, DeclaresDefaultAndReturnsExistingValue)
{
  auto node = std::make_shared<rclcpp::Node>("parameter_default_test");

  const double initial =
    savo_mapping::params::declare_or_get<double>(
    *node, "publish_rate_hz", 10.0);

  EXPECT_DOUBLE_EQ(initial, 10.0);
  EXPECT_TRUE(node->has_parameter("publish_rate_hz"));

  const auto result = node->set_parameter(
    rclcpp::Parameter("publish_rate_hz", 20.0));

  ASSERT_TRUE(result.successful);

  const double existing =
    savo_mapping::params::declare_or_get<double>(
    *node, "publish_rate_hz", 10.0);

  EXPECT_DOUBLE_EQ(existing, 20.0);
}

TEST_F(ParameterUtilsRosTest, HonoursParameterOverrides)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("quality_threshold", 0.85)});

  auto node = std::make_shared<rclcpp::Node>(
    "parameter_override_test",
    options);

  const double value =
    savo_mapping::params::declare_or_get<double>(
    *node, "quality_threshold", 0.70);

  EXPECT_DOUBLE_EQ(value, 0.85);
}

TEST_F(ParameterUtilsRosTest, RejectsBlankDeclarationName)
{
  auto node = std::make_shared<rclcpp::Node>("parameter_name_test");

  EXPECT_THROW(
    savo_mapping::params::declare_or_get<double>(*node, "", 1.0),
    std::invalid_argument);

  EXPECT_THROW(
    savo_mapping::params::declare_or_get<double>(*node, "   ", 1.0),
    std::invalid_argument);
}

TEST(ParameterUtilsContract, ValidatesFloatingPointValues)
{
  EXPECT_TRUE(savo_mapping::params::is_finite_number(0.0));
  EXPECT_TRUE(savo_mapping::params::is_positive_number(0.1));
  EXPECT_TRUE(savo_mapping::params::is_non_negative_number(0.0));
  EXPECT_TRUE(
    savo_mapping::params::is_number_in_closed_range(0.80, 0.0, 1.0));

  EXPECT_FALSE(savo_mapping::params::is_positive_number(0.0));
  EXPECT_FALSE(savo_mapping::params::is_non_negative_number(-0.1));
  EXPECT_FALSE(
    savo_mapping::params::is_number_in_closed_range(1.1, 0.0, 1.0));

  EXPECT_FALSE(
    savo_mapping::params::is_finite_number(
      std::numeric_limits<double>::quiet_NaN()));

  EXPECT_FALSE(
    savo_mapping::params::is_finite_number(
      std::numeric_limits<double>::infinity()));
}

TEST(ParameterUtilsContract, RequiresSafeFloatingPointValues)
{
  EXPECT_DOUBLE_EQ(
    savo_mapping::params::require_positive_parameter(
      "publish_rate_hz", 10.0),
    10.0);

  EXPECT_DOUBLE_EQ(
    savo_mapping::params::require_non_negative_parameter(
      "startup_delay_s", 0.0),
    0.0);

  EXPECT_DOUBLE_EQ(
    savo_mapping::params::require_parameter_in_closed_range(
      "quality_threshold", 0.80, 0.0, 1.0),
    0.80);

  EXPECT_THROW(
    savo_mapping::params::require_positive_parameter(
      "publish_rate_hz", 0.0),
    std::out_of_range);

  EXPECT_THROW(
    savo_mapping::params::require_parameter_in_closed_range(
      "quality_threshold", 1.5, 0.0, 1.0),
    std::out_of_range);

  EXPECT_THROW(
    savo_mapping::params::require_parameter_in_closed_range(
      "quality_threshold", 0.8, 1.0, 0.0),
    std::invalid_argument);

  EXPECT_THROW(
    savo_mapping::params::require_finite_parameter(
      "timeout_s",
      std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}

TEST(ParameterUtilsContract, ValidatesIntegerValues)
{
  EXPECT_TRUE(savo_mapping::params::is_positive_integer(1));
  EXPECT_TRUE(savo_mapping::params::is_non_negative_integer(0));
  EXPECT_TRUE(
    savo_mapping::params::is_integer_in_closed_range(10, 1, 100));

  EXPECT_FALSE(savo_mapping::params::is_positive_integer(0));
  EXPECT_FALSE(savo_mapping::params::is_non_negative_integer(-1));
  EXPECT_FALSE(
    savo_mapping::params::is_integer_in_closed_range(101, 1, 100));

  EXPECT_EQ(
    savo_mapping::params::require_positive_integer_parameter(
      "queue_depth", 10),
    10);

  EXPECT_EQ(
    savo_mapping::params::require_integer_parameter_in_closed_range(
      "queue_depth", 10, 1, 100),
    10);

  EXPECT_THROW(
    savo_mapping::params::require_positive_integer_parameter(
      "queue_depth", 0),
    std::out_of_range);

  EXPECT_THROW(
    savo_mapping::params::require_integer_parameter_in_closed_range(
      "queue_depth", 101, 1, 100),
    std::out_of_range);
}
