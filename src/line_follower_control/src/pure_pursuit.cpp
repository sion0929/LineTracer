#include "line_follower_control/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>

namespace line_follower_control
{

bool forward_progress(double dx,double dy,double heading)
{
  return dx*std::cos(heading)+dy*std::sin(heading)>.03;
}

double pure_pursuit_angular(double target_x, double target_y, double linear, double max_angular)
{
  const double l2 = target_x * target_x + target_y * target_y;
  if (l2 < 1e-6) {
    return 0.0;
  }
  return std::clamp(linear * 2.0 * target_y / l2, -max_angular, max_angular);
}

double image_pursuit_angular(double offset,double heading,double linear,double max_angular)
{
  // A virtual image servo target, never published/stored as measured ground geometry.
  return pure_pursuit_angular(.08,-.2*offset+.15*heading,linear,max_angular);
}

}  // namespace line_follower_control
