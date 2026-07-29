#ifndef SAVO_LOCATIONS__SQLITE_STORE_HPP_
#define SAVO_LOCATIONS__SQLITE_STORE_HPP_

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

struct sqlite3;

namespace savo_locations
{

enum class StorageCode : std::uint8_t
{
  kOk = 0U,
  kInvalidArgument,
  kNotOpen,
  kOpenFailed,
  kConfigurationFailed,
  kSqlError,
  kMigrationFailed,
  kSchemaTooNew,
  kIntegrityFailed,
  kTransactionAlreadyActive,
  kTransactionNotActive,
  kTransactionOwnerMismatch,
};


struct StorageResult
{
  bool success{false};
  StorageCode code{StorageCode::kSqlError};
  int sqlite_code{0};
  std::string reason;
};


struct StorageConfiguration
{
  bool foreign_keys_enabled{false};
  std::string journal_mode;
  int busy_timeout_ms{0};
};


struct SchemaStatus
{
  std::uint32_t previous_version{0U};
  std::uint32_t current_version{0U};
  std::uint32_t supported_version{0U};
  bool migration_applied{false};
};


struct IntegrityReport
{
  bool healthy{false};
  std::vector<std::string> integrity_messages;
  std::vector<std::string> foreign_key_violations;
};


[[nodiscard]]
std::string_view to_string(
  StorageCode code) noexcept;


class SqliteStore
{
public:
  explicit SqliteStore(
    std::string database_path);

  ~SqliteStore();

  SqliteStore(
    const SqliteStore &) = delete;

  SqliteStore & operator=(
    const SqliteStore &) = delete;

  SqliteStore(
    SqliteStore &&) = delete;

  SqliteStore & operator=(
    SqliteStore &&) = delete;

  [[nodiscard]]
  StorageResult open();

  [[nodiscard]]
  StorageResult close();

  [[nodiscard]]
  bool is_open() const noexcept;

  [[nodiscard]]
  const std::string &
  database_path() const noexcept;

  [[nodiscard]]
  StorageConfiguration
  configuration() const;

  [[nodiscard]]
  StorageResult migrate(
    SchemaStatus * status);

  [[nodiscard]]
  StorageResult schema_version(
    std::uint32_t * version) const;

  [[nodiscard]]
  StorageResult integrity_check(
    IntegrityReport * report) const;

  [[nodiscard]]
  StorageResult begin_immediate();

  [[nodiscard]]
  StorageResult commit();

  [[nodiscard]]
  StorageResult rollback();

  [[nodiscard]]
  bool in_transaction() const noexcept;

  [[nodiscard]]
  StorageResult set_metadata(
    std::string_view key,
    std::string_view value);

  [[nodiscard]]
  StorageResult get_metadata(
    std::string_view key,
    std::optional<std::string> * value) const;

  [[nodiscard]]
  StorageResult table_exists(
    std::string_view table_name,
    bool * exists) const;

private:
  friend class SqliteRepository;

  [[nodiscard]]
  StorageResult execute_locked(
    std::string_view sql) const;

  [[nodiscard]]
  StorageResult query_single_int_locked(
    std::string_view sql,
    int * value) const;

  [[nodiscard]]
  StorageResult query_single_text_locked(
    std::string_view sql,
    std::string * value) const;

  [[nodiscard]]
  StorageResult check_open_locked() const;

  [[nodiscard]]
  StorageResult check_transaction_owner_locked()
    const;

  [[nodiscard]]
  StorageResult rollback_locked();

  std::string database_path_;

  mutable std::mutex mutex_;
  sqlite3 * database_{nullptr};

  StorageConfiguration configuration_;

  bool transaction_active_{false};
  std::thread::id transaction_owner_;
};

}  // namespace savo_locations

#endif  // SAVO_LOCATIONS__SQLITE_STORE_HPP_
