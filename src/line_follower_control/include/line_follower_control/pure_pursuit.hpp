#pragma once

namespace line_follower_control
{
bool forward_progress(double dx,double dy,double heading);
double image_pursuit_angular(double offset,double heading,double linear,double max_angular);

// Pure Pursuit angular velocity for a target (x, y) in the robot frame: omega = v * 2y / L^2.
// TODO(D-09): corner-geometry preservation (design §8.1), curvature-based speed coupling.
double pure_pursuit_angular(double target_x, double target_y, double linear, double max_angular);

}  // namespace line_follower_control
