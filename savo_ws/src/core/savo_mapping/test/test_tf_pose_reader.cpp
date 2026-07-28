#include "savo_mapping/tf_pose_reader.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#ifndef SAVO_MAPPING_TF_POSE_READER_RUNTIME_FIXTURE

#include <gtest/gtest.h>

namespace savo_mapping
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::TransformStamped make_transform()
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "fixture_target";
  transform.child_frame_id = "fixture_source";
  transform.transform.rotation.w = 1.0;
  return transform;
}

TfPoseReaderOptions short_options()
{
  TfPoseReaderOptions options;
  options.target_frame = "fixture_target";
  options.source_frame = "fixture_source";
  options.lookup_timeout_sec = 0.003;
  options.stale_timeout_sec = 1.0;
  return options;
}

class ScriptedBuffer final : public tf2_ros::Buffer
{
public:
  enum class Result
  {
    kTransform,
    kLookupError,
    kTimeout,
    kExtrapolation,
    kConnectivity,
    kInvalidArgument,
    kGenericError,
  };

  explicit ScriptedBuffer(
    const rclcpp::Clock::SharedPtr & clock)
  : tf2_ros::Buffer(clock),
    transform_(make_transform())
  {
  }

  void set_result(const Result result)
  {
    result_ = result;
  }

  void set_transform(
    geometry_msgs::msg::TransformStamped transform)
  {
    transform_ = std::move(transform);
  }

  std::size_t call_count() const
  {
    return call_count_;
  }

  geometry_msgs::msg::TransformStamped
  lookupTransform(
    const std::string &,
    const std::string &,
    const tf2::TimePoint &,
    const tf2::Duration) const override
  {
    ++call_count_;

    switch (result_) {
      case Result::kTransform:
        return transform_;
      case Result::kLookupError:
        throw tf2::LookupException(
                "fixture lookup error");
      case Result::kTimeout:
        throw tf2::TimeoutException(
                "fixture timeout");
      case Result::kExtrapolation:
        throw tf2::ExtrapolationException(
                "fixture extrapolation");
      case Result::kConnectivity:
        throw tf2::ConnectivityException(
                "fixture connectivity");
      case Result::kInvalidArgument:
        throw tf2::InvalidArgumentException(
                "fixture invalid argument");
      case Result::kGenericError:
        throw tf2::TransformException(
                "fixture generic error");
    }

    throw tf2::TransformException(
            "unreachable fixture result");
  }

private:
  Result result_{Result::kTransform};
  geometry_msgs::msg::TransformStamped transform_;
  mutable std::size_t call_count_{0};
};

class TfPoseReaderTest : public ::testing::Test
{
protected:
  static rclcpp::Clock::SharedPtr make_clock()
  {
    return std::make_shared<rclcpp::Clock>(
      RCL_SYSTEM_TIME);
  }

  static TfPoseSnapshot evaluate(
    const geometry_msgs::msg::TransformStamped &
    transform,
    const double now_sec = 10.0,
    const double stale_timeout_sec = 1.0)
  {
    return TfPoseReader::evaluate_transform(
      transform,
      rclcpp::Time(
        static_cast<std::int64_t>(
          now_sec * 1.0e9),
        RCL_SYSTEM_TIME),
      stale_timeout_sec);
  }
};

TEST_F(TfPoseReaderTest, RejectsEmptyTargetFrame)
{
  TfPoseReaderOptions options;
  options.target_frame.clear();

  EXPECT_EQ(
    "tf_pose_target_frame_empty",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsWhitespaceOnlyTargetFrame)
{
  TfPoseReaderOptions options;
  options.target_frame = " \t";

  EXPECT_EQ(
    "tf_pose_target_frame_empty",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsEmptySourceFrame)
{
  TfPoseReaderOptions options;
  options.source_frame.clear();

  EXPECT_EQ(
    "tf_pose_source_frame_empty",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsNonFiniteLookupTimeout)
{
  TfPoseReaderOptions options;
  options.lookup_timeout_sec =
    std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(
    "tf_pose_lookup_timeout_invalid",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsInfiniteLookupTimeout)
{
  TfPoseReaderOptions options;
  options.lookup_timeout_sec =
    std::numeric_limits<double>::infinity();

  EXPECT_EQ(
    "tf_pose_lookup_timeout_invalid",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsZeroLookupTimeout)
{
  TfPoseReaderOptions options;
  options.lookup_timeout_sec = 0.0;

  EXPECT_EQ(
    "tf_pose_lookup_timeout_invalid",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsNegativeLookupTimeout)
{
  TfPoseReaderOptions options;
  options.lookup_timeout_sec = -0.1;

  EXPECT_EQ(
    "tf_pose_lookup_timeout_invalid",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsNonFiniteStaleTimeout)
{
  TfPoseReaderOptions options;
  options.stale_timeout_sec =
    std::numeric_limits<double>::quiet_NaN();

  EXPECT_EQ(
    "tf_pose_stale_timeout_invalid",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsInfiniteStaleTimeout)
{
  TfPoseReaderOptions options;
  options.stale_timeout_sec =
    std::numeric_limits<double>::infinity();

  EXPECT_EQ(
    "tf_pose_stale_timeout_invalid",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, RejectsNegativeStaleTimeout)
{
  TfPoseReaderOptions options;
  options.stale_timeout_sec = -0.1;

  EXPECT_EQ(
    "tf_pose_stale_timeout_invalid",
    validate_tf_pose_reader_options(options));
}

TEST_F(TfPoseReaderTest, ZeroStaleTimeoutDisablesFreshnessLimit)
{
  auto transform = make_transform();
  transform.header.stamp.sec = 1;

  const auto snapshot =
    evaluate(transform, 10.0, 0.0);

  EXPECT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.fresh);
  EXPECT_EQ("tf_pose_ready", snapshot.reason);
  EXPECT_DOUBLE_EQ(9.0, snapshot.age_sec);
}

TEST_F(TfPoseReaderTest, ConstructorRejectsNullClock)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);

  EXPECT_THROW(
    TfPoseReader(
      nullptr,
      buffer,
      short_options()),
    std::invalid_argument);
}

TEST_F(TfPoseReaderTest, ConstructorRejectsNullBuffer)
{
  EXPECT_THROW(
    TfPoseReader(
      make_clock(),
      nullptr,
      short_options()),
    std::invalid_argument);
}

TEST_F(TfPoseReaderTest, MissingTransformReturnsUnavailableWithinBound)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);

  buffer->set_result(
    ScriptedBuffer::Result::kLookupError);

  TfPoseReader reader(
    clock,
    buffer,
    short_options());

  const auto snapshot = reader.read();

  EXPECT_FALSE(snapshot.valid);
  EXPECT_FALSE(snapshot.fresh);
  EXPECT_EQ(
    "tf_pose_transform_unavailable",
    snapshot.reason);
  EXPECT_GE(snapshot.lookup_duration_sec, 0.0);
  EXPECT_LT(snapshot.lookup_duration_sec, 0.1);
  EXPECT_GT(buffer->call_count(), 0U);
}

TEST_F(TfPoseReaderTest, ReadsValidIdentityTransform)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);

  TfPoseReader reader(
    clock,
    buffer,
    short_options());

  const auto snapshot = reader.read();

  EXPECT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.fresh);
  EXPECT_EQ("tf_pose_ready", snapshot.reason);
  EXPECT_DOUBLE_EQ(1.0, snapshot.quaternion_w);
  EXPECT_DOUBLE_EQ(0.0, snapshot.yaw_rad);
}

TEST_F(TfPoseReaderTest, ReadsTranslatedTransform)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);
  auto transform = make_transform();
  transform.transform.translation.x = 1.25;
  transform.transform.translation.y = -2.5;
  transform.transform.translation.z = 0.065;
  buffer->set_transform(transform);

  TfPoseReader reader(
    clock,
    buffer,
    short_options());

  const auto snapshot = reader.read();

  EXPECT_TRUE(snapshot.valid);
  EXPECT_DOUBLE_EQ(1.25, snapshot.x_m);
  EXPECT_DOUBLE_EQ(-2.5, snapshot.y_m);
  EXPECT_DOUBLE_EQ(0.065, snapshot.z_m);
}

TEST_F(TfPoseReaderTest, ExtractsNormalizedYaw)
{
  auto transform = make_transform();
  transform.transform.rotation.z =
    std::sin(kPi / 4.0);
  transform.transform.rotation.w =
    std::cos(kPi / 4.0);

  const auto snapshot = evaluate(transform);

  ASSERT_TRUE(snapshot.valid);
  EXPECT_NEAR(kPi / 2.0, snapshot.yaw_rad, 1.0e-12);
}

TEST_F(TfPoseReaderTest, NormalizesNonUnitQuaternion)
{
  auto transform = make_transform();
  transform.transform.rotation.z =
    4.0 * std::sin(kPi / 8.0);
  transform.transform.rotation.w =
    4.0 * std::cos(kPi / 8.0);

  const auto snapshot = evaluate(transform);

  ASSERT_TRUE(snapshot.valid);
  EXPECT_NEAR(
    1.0,
    std::hypot(
      snapshot.quaternion_z,
      snapshot.quaternion_w),
    1.0e-12);
  EXPECT_NEAR(kPi / 4.0, snapshot.yaw_rad, 1.0e-12);
}

TEST_F(TfPoseReaderTest, RejectsZeroNormQuaternion)
{
  auto transform = make_transform();
  transform.transform.rotation.w = 0.0;

  const auto snapshot = evaluate(transform);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(
    "tf_pose_quaternion_invalid",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, RejectsNonFiniteTranslation)
{
  auto transform = make_transform();
  transform.transform.translation.y =
    std::numeric_limits<double>::infinity();

  const auto snapshot = evaluate(transform);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(
    "tf_pose_translation_invalid",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, RejectsNonFiniteQuaternion)
{
  auto transform = make_transform();
  transform.transform.rotation.x =
    std::numeric_limits<double>::quiet_NaN();

  const auto snapshot = evaluate(transform);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(
    "tf_pose_quaternion_invalid",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, NonFiniteYawCannotBeNormalized)
{
  EXPECT_FALSE(
    std::isfinite(
      TfPoseReader::normalize_yaw(
        std::numeric_limits<double>::
        quiet_NaN())));
}

TEST_F(TfPoseReaderTest, PreservesConfiguredFrameNames)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);
  auto options = short_options();
  options.target_frame =
    "fixture_requested_target";
  options.source_frame =
    "fixture_requested_source";

  TfPoseReader reader(clock, buffer, options);
  const auto snapshot = reader.read();

  EXPECT_EQ(
    "fixture_requested_target",
    snapshot.target_frame);
  EXPECT_EQ(
    "fixture_requested_source",
    snapshot.source_frame);
}

TEST_F(TfPoseReaderTest, PreservesTransformTimestamp)
{
  auto transform = make_transform();
  transform.header.stamp.sec = 9;
  transform.header.stamp.nanosec = 125000000U;

  const auto snapshot = evaluate(transform, 10.0);

  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(
    9125000000LL,
    snapshot.transform_stamp.nanoseconds());
}

TEST_F(TfPoseReaderTest, ReportsFiniteNonNegativeAge)
{
  auto transform = make_transform();
  transform.header.stamp.sec = 9;
  transform.header.stamp.nanosec = 500000000U;

  const auto snapshot = evaluate(transform, 10.0);

  ASSERT_TRUE(snapshot.valid);
  EXPECT_TRUE(std::isfinite(snapshot.age_sec));
  EXPECT_DOUBLE_EQ(0.5, snapshot.age_sec);
}

TEST_F(TfPoseReaderTest, RejectsStaleTransform)
{
  auto transform = make_transform();
  transform.header.stamp.sec = 8;

  const auto snapshot = evaluate(
    transform,
    10.0,
    0.5);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_FALSE(snapshot.fresh);
  EXPECT_DOUBLE_EQ(2.0, snapshot.age_sec);
  EXPECT_EQ(
    "tf_pose_transform_stale",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, AcceptsZeroTimestampAsTimeless)
{
  const auto snapshot = evaluate(
    make_transform(),
    0.0,
    0.1);

  EXPECT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.fresh);
  EXPECT_DOUBLE_EQ(0.0, snapshot.age_sec);
}

TEST_F(TfPoseReaderTest, RejectsFutureTimestampDeterministically)
{
  auto transform = make_transform();
  transform.header.stamp.sec = 11;

  const auto snapshot = evaluate(transform, 10.0);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(
    "tf_pose_timestamp_invalid",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, RejectsMalformedTimestamp)
{
  auto transform = make_transform();
  transform.header.stamp.nanosec = 1000000000U;

  const auto snapshot = evaluate(transform);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(
    "tf_pose_timestamp_invalid",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, RejectsInvalidClockValue)
{
  const auto snapshot =
    TfPoseReader::evaluate_transform(
    make_transform(),
    rclcpp::Time(
      static_cast<std::int64_t>(-1),
      RCL_SYSTEM_TIME),
    1.0);

  EXPECT_FALSE(snapshot.valid);
  EXPECT_EQ(
    "tf_pose_clock_invalid",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, MapsTimeoutException)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);
  buffer->set_result(
    ScriptedBuffer::Result::kTimeout);

  const auto snapshot =
    TfPoseReader(
    clock,
    buffer,
    short_options()).read();

  EXPECT_EQ(
    "tf_pose_lookup_timeout",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, MapsExtrapolationException)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);
  buffer->set_result(
    ScriptedBuffer::Result::kExtrapolation);

  const auto snapshot =
    TfPoseReader(
    clock,
    buffer,
    short_options()).read();

  EXPECT_EQ(
    "tf_pose_extrapolation_error",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, MapsConnectivityException)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);
  buffer->set_result(
    ScriptedBuffer::Result::kConnectivity);

  const auto snapshot =
    TfPoseReader(
    clock,
    buffer,
    short_options()).read();

  EXPECT_EQ(
    "tf_pose_connectivity_error",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, MapsInvalidArgumentException)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);
  buffer->set_result(
    ScriptedBuffer::Result::kInvalidArgument);

  const auto snapshot =
    TfPoseReader(
    clock,
    buffer,
    short_options()).read();

  EXPECT_EQ(
    "tf_pose_invalid_argument",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, GenericTfFailureIsUnavailable)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);
  buffer->set_result(
    ScriptedBuffer::Result::kGenericError);

  const auto snapshot =
    TfPoseReader(
    clock,
    buffer,
    short_options()).read();

  EXPECT_EQ(
    "tf_pose_transform_unavailable",
    snapshot.reason);
}

TEST_F(TfPoseReaderTest, RepeatedReadsAreDeterministic)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);
  TfPoseReader reader(
    clock,
    buffer,
    short_options());

  const auto first = reader.read();
  const auto second = reader.read();

  EXPECT_EQ(first.valid, second.valid);
  EXPECT_EQ(first.fresh, second.fresh);
  EXPECT_EQ(first.reason, second.reason);
  EXPECT_DOUBLE_EQ(first.x_m, second.x_m);
  EXPECT_DOUBLE_EQ(first.yaw_rad, second.yaw_rad);
  EXPECT_DOUBLE_EQ(first.age_sec, second.age_sec);
}

TEST_F(TfPoseReaderTest, DefaultsFollowCentralMappingFrames)
{
  const TfPoseReaderOptions options;

  EXPECT_EQ(slam::MAP_FRAME, options.target_frame);
  EXPECT_EQ(slam::BASE_FRAME, options.source_frame);
  EXPECT_DOUBLE_EQ(0.20, options.lookup_timeout_sec);
  EXPECT_DOUBLE_EQ(1.00, options.stale_timeout_sec);
}

TEST_F(TfPoseReaderTest, ReaderOwnsNoNodeOrRosEndpoints)
{
  auto clock = make_clock();
  auto buffer =
    std::make_shared<ScriptedBuffer>(clock);

  EXPECT_NO_THROW(
    TfPoseReader(
      clock,
      buffer,
      short_options()));
}

}  // namespace
}  // namespace savo_mapping

#else

namespace
{

struct FixtureOptions
{
  std::string target_frame;
  std::string source_frame;
  double lookup_timeout_sec{0.20};
  double stale_timeout_sec{1.00};
  double wait_sec{0.50};
};

bool parse_fixture_options(
  const int argc,
  char ** argv,
  FixtureOptions & options)
{
  for (int index = 1; index < argc; ++index) {
    const std::string argument{argv[index]};

    if (
      argument == "--target-frame" &&
      index + 1 < argc)
    {
      options.target_frame = argv[++index];
      continue;
    }

    if (
      argument == "--source-frame" &&
      index + 1 < argc)
    {
      options.source_frame = argv[++index];
      continue;
    }

    if (
      argument == "--lookup-timeout-sec" &&
      index + 1 < argc)
    {
      options.lookup_timeout_sec =
        std::stod(argv[++index]);
      continue;
    }

    if (
      argument == "--stale-timeout-sec" &&
      index + 1 < argc)
    {
      options.stale_timeout_sec =
        std::stod(argv[++index]);
      continue;
    }

    if (
      argument == "--wait-sec" &&
      index + 1 < argc)
    {
      options.wait_sec = std::stod(argv[++index]);
      continue;
    }

    return false;
  }

  return
    !options.target_frame.empty() &&
    !options.source_frame.empty() &&
    std::isfinite(options.wait_sec) &&
    options.wait_sec >= 0.0;
}

}  // namespace

int main(int argc, char ** argv)
{
  FixtureOptions fixture_options;

  try {
    if (!parse_fixture_options(
        argc,
        argv,
        fixture_options))
    {
      std::cerr <<
        "fixture argument error" << std::endl;
      return 2;
    }
  } catch (const std::exception & exception) {
    std::cerr << exception.what() << std::endl;
    return 2;
  }

  rclcpp::init(argc, argv);
  int result = 0;

  try {
    const auto node = std::make_shared<rclcpp::Node>(
      "tf_pose_reader_runtime_fixture");

    const auto buffer =
      std::make_shared<tf2_ros::Buffer>(
      node->get_clock());

    tf2_ros::TransformListener listener(
      *buffer,
      node,
      true);

    std::this_thread::sleep_for(
      std::chrono::duration<double>(
        fixture_options.wait_sec));

    savo_mapping::TfPoseReaderOptions options;
    options.target_frame =
      fixture_options.target_frame;
    options.source_frame =
      fixture_options.source_frame;
    options.lookup_timeout_sec =
      fixture_options.lookup_timeout_sec;
    options.stale_timeout_sec =
      fixture_options.stale_timeout_sec;

    const savo_mapping::TfPoseReader reader(
      node->get_clock(),
      buffer,
      options);

    const auto snapshot = reader.read();

    std::cout <<
      std::setprecision(17) <<
      "tf_pose_result" <<
      " valid=" << (snapshot.valid ? 1 : 0) <<
      " fresh=" << (snapshot.fresh ? 1 : 0) <<
      " reason=" << snapshot.reason <<
      " target=" << snapshot.target_frame <<
      " source=" << snapshot.source_frame <<
      " x_m=" << snapshot.x_m <<
      " y_m=" << snapshot.y_m <<
      " z_m=" << snapshot.z_m <<
      " yaw_rad=" << snapshot.yaw_rad <<
      " age_sec=" << snapshot.age_sec <<
      " lookup_duration_sec=" <<
      snapshot.lookup_duration_sec <<
      std::endl;
  } catch (const std::exception & exception) {
    std::cerr <<
      "fixture failure: " <<
      exception.what() <<
      std::endl;

    result = 3;
  }

  rclcpp::shutdown();
  return result;
}

#endif
