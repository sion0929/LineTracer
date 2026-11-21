#include "line_follower_control/rotate_pid.hpp"

#include <algorithm>

namespace line_follower_control
{

double RotatePid::update(double heading_error, double dt)
{
  if (dt <= 0.0) {
    return 0.0;
  }
  integral_ += heading_error * dt;
  const double derivative = has_prev_ ? (heading_error - prev_error_) / dt : 0.0;
  prev_error_ = heading_error;
  has_prev_ = true;
  const double out = kp_ * heading_error + ki_ * integral_ + kd_ * derivative;
  return std::clamp(out, -max_output_, max_output_);
}

void RotatePid::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
  has_prev_ = false;
}

}  // namespace line_follower_control
