#include "line_follower_control/safety_policy.hpp"

#include <algorithm>
#include <cmath>

namespace line_follower_control
{

namespace
{
SafetyDecision stop(const std::string & reason)
{
  return SafetyDecision{0.0, 0.0, false, reason};
}

double rate_limit(double target, double prev, double max_rate, double dt)
{
  const double step = max_rate * dt;
  return std::clamp(target, prev - step, prev + step);
}
}  // namespace

SafetyDecision evaluate_safety(
  const SafetyInputs & in, const SafetyLimits & lim,
  double prev_linear, double prev_angular, double dt)
{
  if (!(in.mission_age <= lim.mission_timeout)) {
    return stop("mission state stale");
  }
  if (!in.drive_enabled) {
    return stop("drive not enabled");
  }
  if (!(in.odom_age <= lim.odom_timeout)) {
    return stop("odometry stale");
  }
  if (!(in.perception_age <= lim.perception_timeout)) {
    return stop("perception stale");
  }
  if (!in.perception_healthy) {
    return stop("perception unhealthy");
  }
  if (!(in.path_status_age <= lim.path_status_timeout)) {
    return stop("path status stale");
  }
  if (!in.path_valid) {
    return stop("path invalid");
  }
  if (!(in.candidate_age <= lim.candidate_timeout) || in.candidate_expired) {
    return stop("command candidate stale");
  }
  if (!std::isfinite(in.candidate_linear) || !std::isfinite(in.candidate_angular)) {
    return stop("command candidate not finite");
  }

  SafetyDecision d;
  d.approved = true;
  d.linear = std::clamp(in.candidate_linear, -lim.max_linear, lim.max_linear);
  d.angular = std::clamp(in.candidate_angular, -lim.max_angular, lim.max_angular);
  if (dt > 0.0) {
    d.linear = rate_limit(d.linear, prev_linear, lim.max_linear_accel, dt);
    d.angular = rate_limit(d.angular, prev_angular, lim.max_angular_accel, dt);
  }
  return d;
}

}  // namespace line_follower_control
