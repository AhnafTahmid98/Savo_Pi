#pragma once

#include <string>

namespace savo_base
{

struct SafetySnapshot
{
  bool has_command{false};
  double command_age_s{0.0};
  bool command_stale{true};
  bool safety_stop{false};
  double slowdown_factor{1.0};
};

struct SafetyDecision
{
  bool force_zero{true};
  bool blocked{false};
  std::string reason{"no_command"};
  double slowdown_factor{1.0};
};

class BaseSafety
{
public:
  explicit BaseSafety(double watchdog_timeout_s = 0.30);

  void set_watchdog_timeout(double watchdog_timeout_s);
  double watchdog_timeout() const;

  SafetyDecision evaluate(const SafetySnapshot & snapshot) const;

  static double clamp_slowdown(double value);

private:
  double watchdog_timeout_s_{0.30};
};

}  // namespace savo_base
