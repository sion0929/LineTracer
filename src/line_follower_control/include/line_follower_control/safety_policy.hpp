#pragma once

#include <limits>
#include <string>

namespace line_follower_control
{

// All timeouts/limits are placeholders until measured (design §14.3).
struct SafetyLimits
{
  double max_linear{0.2};           // [m/s]
  double max_angular{1.5};          // [rad/s]
  double max_linear_accel{0.5};     // [m/s^2]
  double max_angular_accel{3.0};    // [rad/s^2]
  double mission_timeout{0.5};      // [s]
  double candidate_timeout{0.2};    // [s]
  double path_status_timeout{0.5};  // [s]
  double perception_timeout{0.5};   // [s]
  double odom_timeout{0.5};         // [s]
};

struct SafetyInputs
{
  static constexpr double kNever = std::numeric_limits<double>::infinity();

  bool drive_enabled{false};
  double mission_age{kNever};

  double candidate_age{kNever};
  bool candidate_expired{true};
  double candidate_linear{0.0};
  double candidate_angular{0.0};

  bool path_valid{false};
  double path_status_age{kNever};

  bool perception_healthy{false};
  double perception_age{kNever};

  double odom_age{kNever};
};

struct SafetyDecision
{
  double linear{0.0};
  double angular{0.0};
  bool approved{false};
  std::string reason;
};

// Final command gate (design §9.2). Any missing permission or stale input yields an immediate zero.
// Approved commands are clamped and rate-limited against the previously published command.
SafetyDecision evaluate_safety(
  const SafetyInputs & in, const SafetyLimits & limits,
  double prev_linear, double prev_angular, double dt);

}  // namespace line_follower_control
