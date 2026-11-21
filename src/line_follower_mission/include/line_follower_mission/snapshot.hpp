#pragma once
#include <cmath>
#include "geometry_msgs/msg/pose.hpp"
#include "line_follower_interfaces/msg/local_path.hpp"

namespace line_follower_mission
{
inline line_follower_interfaces::msg::LocalPath path_in_odom(
  const line_follower_interfaces::msg::LocalPath & input, const geometry_msgs::msg::Pose & pose)
{
  auto result = input;
  result.target_heading = std::atan2(std::sin(result.target_heading), std::cos(result.target_heading));
  if (input.header.frame_id == "odom") {return result;}
  const auto & q = pose.orientation;
  const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & point : result.points) {
    const double x = point.x, y = point.y;
    point.x = pose.position.x + c*x - s*y;
    point.y = pose.position.y + s*x + c*y;
  }
  const double x = result.target_point.x, y = result.target_point.y;
  result.target_point.x = pose.position.x + c*x - s*y;
  result.target_point.y = pose.position.y + s*x + c*y;
  result.target_heading = std::atan2(std::sin(result.target_heading+yaw),
                                    std::cos(result.target_heading+yaw));
  result.header.frame_id = "odom";
  return result;
}
}  // namespace line_follower_mission
