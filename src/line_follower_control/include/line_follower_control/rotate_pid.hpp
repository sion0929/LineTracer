#pragma once

namespace line_follower_control
{

// Heading PID used in ROTATE_IN_PLACE after the robot has stopped (design §8.1).
// Gains are placeholders until tuned in Gazebo (design §14.3).
class RotatePid
{
public:
  RotatePid(double kp, double ki, double kd, double max_output)
  : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output) {}

  double update(double heading_error, double dt);
  void reset();

private:
  double kp_, ki_, kd_, max_output_;
  double integral_{0.0};
  double prev_error_{0.0};
  bool has_prev_{false};
};

}  // namespace line_follower_control
