#pragma once

#include <string>

namespace savo_base
{

struct WatchdogStatus
{
  bool has_command{false};
  bool stale{true};
  bool tripped{true};
  double age_s{1.0e9};
  double timeout_s{0.30};
  std::string reason{"no_command"};
};

class TimeoutWatchdog
{
public:
  explicit TimeoutWatchdog(double timeout_s = 0.30);

  void set_timeout(double timeout_s);
  double timeout() const;

  WatchdogStatus evaluate(bool has_command, double command_age_s) const;

private:
  double timeout_s_{0.30};
};

}  // namespace savo_base
