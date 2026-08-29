#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <memory>
#include <regex>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "fake_i2c_bus.hpp"
#include "savo_localization/bno055_calibration.hpp"

namespace savo_localization
{
namespace
{

namespace fs = std::filesystem;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    const fs::path pattern = fs::temp_directory_path() /
      "savo_bno055_calibration_test_XXXXXX";
    const std::string pattern_text = pattern.string();
    std::vector<char> path_buffer(pattern_text.begin(), pattern_text.end());
    path_buffer.push_back('\0');
    const char * result = ::mkdtemp(path_buffer.data());
    if (result == nullptr) {
      throw std::runtime_error("failed to create calibration test directory");
    }
    path_ = result;
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    fs::remove_all(path_, error);
  }

  fs::path path() const
  {
    return path_;
  }

private:
  fs::path path_;
};

BNO055CalibrationProfile valid_profile(const int delta = 0)
{
  return BNO055CalibrationProfile{
    static_cast<int16_t>(-100 + delta), 20, 30,
    -400, 500, -600,
    -70, 80, -90,
    1000, 480};
}

BNO055CalibrationMetadata valid_metadata()
{
  return BNO055CalibrationMetadata{
    1,
    0x28,
    BNO055Mode::NDOF,
    "2026-08-29T12:34:56Z"};
}

BNO055CalibrationDocument valid_document(const int delta = 0)
{
  BNO055CalibrationDocument document;
  document.metadata = valid_metadata();
  document.calibration = valid_profile(delta);
  return document;
}

struct DriverFixture
{
  explicit DriverFixture(const bool operational = false)
  {
    auto owned_bus = std::make_unique<test::FakeI2CBus>();
    bus = owned_bus.get();
    driver = std::make_unique<BNO055Driver>(std::move(owned_bus));
    EXPECT_TRUE(driver->initialize_config_mode(false));
    if (operational) {
      driver->set_mode(BNO055Mode::NDOF);
    }
  }

  void load_profile(const BNO055CalibrationProfile & profile)
  {
    const auto encoded = encode_bno055_calibration_profile(profile);
    bus->load_block(
      bno055_registers::ACC_OFFSET_X_LSB,
      std::vector<uint8_t>(encoded.begin(), encoded.end()));
  }

  test::FakeI2CBus * bus{nullptr};
  std::unique_ptr<BNO055Driver> driver;
};

TEST(BNO055CalibrationStoreTest, ParsesValidVersionedYaml)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  BNO055CalibrationStore store(profile_path.string());
  std::string error;
  ASSERT_TRUE(store.save_atomic(valid_document(), error)) << error;

  const auto loaded = store.load();
  ASSERT_EQ(loaded.status, CalibrationProfileLoadStatus::LOADED) << loaded.error;
  EXPECT_EQ(loaded.document.schema_version, 1);
  EXPECT_EQ(loaded.document.sensor, "bno055");
  EXPECT_EQ(loaded.document.metadata.i2c_bus, 1);
  EXPECT_EQ(loaded.document.metadata.i2c_address, 0x28);
  EXPECT_EQ(loaded.document.metadata.operational_mode, BNO055Mode::NDOF);
  EXPECT_EQ(loaded.document.metadata.captured_at, "2026-08-29T12:34:56Z");
  EXPECT_EQ(loaded.document.calibration, valid_profile());
}

TEST(BNO055CalibrationStoreTest, RejectsMalformedYaml)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  std::ofstream(profile_path) << "schema_version: [not valid\n";

  const auto loaded = BNO055CalibrationStore(profile_path.string()).load();
  EXPECT_EQ(loaded.status, CalibrationProfileLoadStatus::INVALID);
  EXPECT_FALSE(loaded.error.empty());
}

TEST(BNO055CalibrationStoreTest, RejectsUnsupportedSchema)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  std::ofstream stream(profile_path);
  stream << "schema_version: 2\n"
         << "sensor: bno055\n"
         << "metadata: {}\n"
         << "calibration: {}\n";
  stream.close();

  const auto loaded = BNO055CalibrationStore(profile_path.string()).load();
  EXPECT_EQ(loaded.status, CalibrationProfileLoadStatus::INVALID);
  EXPECT_FALSE(loaded.error.empty());
}

TEST(BNO055CalibrationStoreTest, MissingProfileIsDistinctFromInvalidProfile)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "missing.yaml";

  const auto loaded = BNO055CalibrationStore(profile_path.string()).load();
  EXPECT_EQ(loaded.status, CalibrationProfileLoadStatus::MISSING);
  EXPECT_TRUE(loaded.error.empty());
}

TEST(BNO055CalibrationStoreTest, AtomicReplacementLeavesNoTemporaryFile)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  BNO055CalibrationStore store(profile_path.string());
  std::string error;
  ASSERT_TRUE(store.save_atomic(valid_document(), error)) << error;
  ASSERT_TRUE(store.save_atomic(valid_document(1), error)) << error;

  const auto loaded = store.load();
  ASSERT_EQ(loaded.status, CalibrationProfileLoadStatus::LOADED);
  EXPECT_EQ(loaded.document.calibration, valid_profile(1));
  for (const auto & entry : fs::directory_iterator(temporary.path())) {
    EXPECT_EQ(entry.path().filename(), "profile.yaml");
  }
}

TEST(BNO055CalibrationStoreTest, FailedSavePreservesExistingValidProfile)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  BNO055CalibrationStore store(profile_path.string());
  std::string error;
  ASSERT_TRUE(store.save_atomic(valid_document(), error)) << error;

  auto invalid = valid_document(1);
  invalid.sensor = "not_bno055";
  EXPECT_FALSE(store.save_atomic(invalid, error));
  const auto loaded = store.load();
  ASSERT_EQ(loaded.status, CalibrationProfileLoadStatus::LOADED);
  EXPECT_EQ(loaded.document.calibration, valid_profile());
}

TEST(BNO055CalibrationRuntimeTest, RestoreDisabledDoesNotAccessHardwareOrProfile)
{
  TemporaryDirectory temporary;
  DriverFixture fixture;
  BNO055CalibrationRuntime runtime((temporary.path() / "profile.yaml").string());

  runtime.restore(*fixture.driver, false, true, valid_metadata());
  EXPECT_EQ(runtime.state().status, "disabled");
  EXPECT_FALSE(runtime.state().restore_attempted);
  EXPECT_TRUE(fixture.bus->block_writes.empty());
}

TEST(BNO055CalibrationRuntimeTest, MissingProfileContinuesWithoutRestoreAttempt)
{
  TemporaryDirectory temporary;
  DriverFixture fixture;
  BNO055CalibrationRuntime runtime((temporary.path() / "missing.yaml").string());

  runtime.restore(*fixture.driver, true, true, valid_metadata());
  EXPECT_EQ(runtime.state().status, "profile_missing");
  EXPECT_FALSE(runtime.state().profile_present);
  EXPECT_FALSE(runtime.state().restore_failed);
  EXPECT_FALSE(runtime.state().restore_attempted);
}

TEST(BNO055CalibrationRuntimeTest, MalformedProfileReportsRestoreFailure)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  std::ofstream(profile_path) << "schema_version: [broken\n";
  DriverFixture fixture;
  BNO055CalibrationRuntime runtime(profile_path.string());

  runtime.restore(*fixture.driver, true, true, valid_metadata());
  EXPECT_TRUE(runtime.state().profile_present);
  EXPECT_FALSE(runtime.state().profile_loaded);
  EXPECT_FALSE(runtime.state().restore_attempted);
  EXPECT_TRUE(runtime.state().restore_failed);
  EXPECT_EQ(runtime.state().status, "restore_failed");
}

TEST(BNO055CalibrationRuntimeTest, StartupRestoresBeforeOperationalModeAndVerifiesStatus)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  BNO055CalibrationStore store(profile_path.string());
  std::string error;
  ASSERT_TRUE(store.save_atomic(valid_document(), error)) << error;
  DriverFixture fixture;
  BNO055CalibrationRuntime runtime(profile_path.string());

  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::CONFIG);
  runtime.restore(*fixture.driver, true, true, valid_metadata());
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::CONFIG);
  EXPECT_TRUE(runtime.state().profile_loaded);
  EXPECT_TRUE(runtime.state().restore_attempted);
  EXPECT_TRUE(runtime.state().profile_verified);
  fixture.driver->set_mode(BNO055Mode::NDOF);
  runtime.verify_operational_status(*fixture.driver);

  EXPECT_EQ(runtime.state().status, "restored");
  EXPECT_TRUE(runtime.state().operational_status_verified);
  EXPECT_FALSE(runtime.state().restore_failed);
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);
}

TEST(BNO055CalibrationRuntimeTest, ReadbackMismatchIsNeverAccepted)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  BNO055CalibrationStore store(profile_path.string());
  std::string error;
  ASSERT_TRUE(store.save_atomic(valid_document(), error)) << error;
  DriverFixture fixture;
  fixture.bus->corrupt_next_calibration_readback = true;
  BNO055CalibrationRuntime runtime(profile_path.string());

  runtime.restore(*fixture.driver, true, true, valid_metadata());
  EXPECT_TRUE(runtime.state().restore_attempted);
  EXPECT_TRUE(runtime.state().restore_failed);
  EXPECT_FALSE(runtime.state().profile_verified);
  EXPECT_TRUE(runtime.state().verification_required);
}

TEST(BNO055CalibrationRuntimeTest, PostRestoreSystemErrorIsReported)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  BNO055CalibrationStore store(profile_path.string());
  std::string error;
  ASSERT_TRUE(store.save_atomic(valid_document(), error)) << error;
  DriverFixture fixture;
  BNO055CalibrationRuntime runtime(profile_path.string());

  runtime.restore(*fixture.driver, true, true, valid_metadata());
  fixture.driver->set_mode(BNO055Mode::NDOF);
  fixture.bus->registers[0x3A] = 9;
  runtime.verify_operational_status(*fixture.driver);

  EXPECT_TRUE(runtime.state().profile_verified);
  EXPECT_FALSE(runtime.state().operational_status_verified);
  EXPECT_TRUE(runtime.state().restore_failed);
}

TEST(BNO055CalibrationRuntimeTest, NonRequiredVerificationStillReportsFailure)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  BNO055CalibrationStore store(profile_path.string());
  std::string error;
  ASSERT_TRUE(store.save_atomic(valid_document(), error)) << error;
  DriverFixture fixture;
  fixture.bus->corrupt_next_calibration_readback = true;
  BNO055CalibrationRuntime runtime(profile_path.string());

  runtime.restore(*fixture.driver, true, false, valid_metadata());
  EXPECT_TRUE(runtime.state().restore_failed);
  EXPECT_FALSE(runtime.state().verification_required);
  EXPECT_EQ(runtime.state().status, "restore_failed");
}

TEST(BNO055CalibrationRuntimeTest, IncompatibleMetadataBlocksHardwareWrite)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  BNO055CalibrationStore store(profile_path.string());
  std::string error;
  ASSERT_TRUE(store.save_atomic(valid_document(), error)) << error;
  DriverFixture fixture;
  BNO055CalibrationRuntime runtime(profile_path.string());
  auto expected = valid_metadata();
  expected.i2c_address = 0x29;

  runtime.restore(*fixture.driver, true, true, expected);
  EXPECT_TRUE(runtime.state().profile_loaded);
  EXPECT_TRUE(runtime.state().restore_failed);
  EXPECT_FALSE(runtime.state().restore_attempted);
  EXPECT_TRUE(fixture.bus->block_writes.empty());
}

TEST(BNO055CalibrationCaptureTest, RejectsSaveUnlessEveryLiveLevelIsExactlyThree)
{
  TemporaryDirectory temporary;
  DriverFixture fixture(true);
  BNO055CalibrationRuntime runtime((temporary.path() / "profile.yaml").string());

  fixture.bus->registers[0x35] = 0xFE;
  const auto incomplete = runtime.capture(*fixture.driver, valid_metadata());
  EXPECT_FALSE(incomplete.success);
  EXPECT_NE(incomplete.message.find("3/3/3/3"), std::string::npos);
}

TEST(BNO055CalibrationCaptureTest, FullyCalibratedCaptureSavesDataAndMetadata)
{
  TemporaryDirectory temporary;
  const fs::path profile_path = temporary.path() / "profile.yaml";
  DriverFixture fixture(true);
  fixture.load_profile(valid_profile());
  fixture.bus->registers[0x35] = 0xFF;
  BNO055CalibrationRuntime runtime(profile_path.string());

  const auto result = runtime.capture(*fixture.driver, valid_metadata());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.profile, valid_profile());
  EXPECT_EQ(fixture.driver->mode(), BNO055Mode::NDOF);

  const auto loaded = BNO055CalibrationStore(profile_path.string()).load();
  ASSERT_EQ(loaded.status, CalibrationProfileLoadStatus::LOADED) << loaded.error;
  EXPECT_EQ(loaded.document.metadata.i2c_bus, 1);
  EXPECT_EQ(loaded.document.metadata.i2c_address, 0x28);
  EXPECT_EQ(loaded.document.metadata.operational_mode, BNO055Mode::NDOF);
  EXPECT_EQ(loaded.document.metadata.captured_at, "2026-08-29T12:34:56Z");
}

TEST(BNO055CalibrationMetadataTest, GeneratesUtcTimestamp)
{
  const std::regex iso8601_utc(
    R"(^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z$)");
  EXPECT_TRUE(std::regex_match(bno055_calibration_timestamp_utc(), iso8601_utc));
}

}  // namespace
}  // namespace savo_localization
